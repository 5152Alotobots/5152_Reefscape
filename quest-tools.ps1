#!/usr/bin/env pwsh
# Quest Development Tools PowerShell Script

$APP_PACKAGE = "com.DerpyCatAviationLLC.QuestNav"
$QUEST_PORT = 5555

# Colors for output
function Write-ColorOutput {
    param (
        [Parameter(Mandatory=$true)]
        [string]$Message,
        
        [Parameter(Mandatory=$true)]
        [string]$Color
    )
    
    $originalColor = $host.UI.RawUI.ForegroundColor
    $host.UI.RawUI.ForegroundColor = $Color
    Write-Output $Message
    $host.UI.RawUI.ForegroundColor = $originalColor
}

function Print-Help {
    Write-ColorOutput "Quest Development Tools" "Yellow"
    Write-Output "Usage: .\quest-tools.ps1 [command] [team_number]"
    Write-Output ""
    Write-Output "Commands:"
    Write-Output "  connect <team>    - Find and connect to Quest on robot network (e.g., 5152)"
    Write-Output "  status          - Check ADB connection status (shows USB and network devices)"
    Write-Output "  restart         - Restart the QuestNav app"
    Write-Output "  reboot          - Reboot the Quest"
    Write-Output "  enableWifi      - Enable wireless debugging (USB connection only)"
    Write-Output "  kill            - Force kill ADB server"
}

function Check-Adb {
    try {
        $null = Get-Command adb -ErrorAction Stop
        return $true
    } catch {
        Write-ColorOutput "Error: ADB not found. Please install Android platform tools." "Red"
        return $false
    }
}

function Ensure-Connected {
    $job = Start-Job -ScriptBlock { & adb devices }
    $completed = Wait-Job -Job $job -Timeout 5
    
    if ($null -eq $completed) {
        Write-ColorOutput "ADB command timed out checking connection." "Red"
        Stop-Job -Job $job
        Remove-Job -Job $job -Force
        return $false
    }
    
    $devices = Receive-Job -Job $job
    Remove-Job -Job $job
    
    if (-not ($devices -match "device$")) {
        Write-ColorOutput "Error: No Quest connected. Check with 'status' command first." "Red"
        return $false
    }
    return $true
}

function Force-Kill-Adb {
    Write-ColorOutput "Force killing all ADB processes..." "Yellow"
    
    # Try graceful kill first
    & adb kill-server > $null
    
    # Find and kill any remaining ADB processes
    $processes = Get-Process | Where-Object { $_.Name -eq "adb" }
    if ($processes) {
        foreach ($process in $processes) {
            Write-Output "Killing ADB process with ID $($process.Id)"
            Stop-Process -Id $process.Id -Force
        }
    }
    
    # Give it a moment
    Start-Sleep -Seconds 2
    
    # Check if any ADB processes remain
    $processes = Get-Process | Where-Object { $_.Name -eq "adb" }
    if ($processes) {
        Write-ColorOutput "Warning: Some ADB processes could not be killed." "Red"
        return $false
    } else {
        Write-ColorOutput "All ADB processes killed successfully." "Green"
        return $true
    }
}

function Find-Quest {
    param (
        [Parameter(Mandatory=$true)]
        [string]$Team
    )

    $subnet1 = "10." + $Team.Substring(0,2)
    $subnet2 = $Team.Substring(2,2)
    $subnetFull = "$subnet1.$subnet2"
    
    Write-ColorOutput "Scanning network $subnetFull.0/24 for Quest..." "Green"
    
    # Check if nmap is installed
    try {
        $null = Get-Command nmap -ErrorAction Stop
    } catch {
        Write-ColorOutput "Error: nmap not found. Please install nmap first." "Red"
        return $false
    }
    
    # Kill any existing ADB server first
    Force-Kill-Adb > $null
    
    # Start ADB server with timeout
    $serverJob = Start-Job -ScriptBlock { & adb start-server }
    $serverCompleted = Wait-Job -Job $serverJob -Timeout 10
    
    if ($null -eq $serverCompleted) {
        Write-ColorOutput "ADB server start timed out." "Red"
        Stop-Job -Job $serverJob
        Remove-Job -Job $serverJob -Force
        Force-Kill-Adb
        return $false
    }
    
    Receive-Job -Job $serverJob > $null
    Remove-Job -Job $serverJob
    
    # Use nmap to quickly scan network, excluding specific addresses
    Write-Output "Running quick network scan..."
    $scanExclude = "$subnetFull.1,$subnetFull.2"
    $scanTarget = "$subnetFull.0/24"
    $scan = & nmap -n -sn --exclude $scanExclude $scanTarget | Select-String "Nmap scan report for"
    
    foreach ($line in $scan) {
        # Extract IP address - the last word in the line
        $ip = $line -replace '.*Nmap scan report for ',''
        
        Write-ColorOutput "Found device at $ip, attempting ADB connection..." "Yellow"
        
        # Try ADB connect with timeout
        $connectJob = Start-Job -ScriptBlock {
            param($ip, $port)
            & adb connect "$ip`:$port"
        } -ArgumentList $ip, $QUEST_PORT
        
        $connectCompleted = Wait-Job -Job $connectJob -Timeout 10
        
        if ($null -eq $connectCompleted) {
            Write-ColorOutput "Connection attempt timed out, moving to next device..." "Red"
            Stop-Job -Job $connectJob
            Remove-Job -Job $connectJob -Force
            continue
        }
        
        Receive-Job -Job $connectJob > $null
        Remove-Job -Job $connectJob
        
        # Check if device connected with timeout
        $deviceJob = Start-Job -ScriptBlock { & adb devices }
        $deviceCompleted = Wait-Job -Job $deviceJob -Timeout 5
        
        if ($null -eq $deviceCompleted) {
            Write-ColorOutput "Device check timed out, moving to next device..." "Red"
            Stop-Job -Job $deviceJob
            Remove-Job -Job $deviceJob -Force
            continue
        }
        
        $devices = Receive-Job -Job $deviceJob
        Remove-Job -Job $deviceJob
        
        if ($devices -match "$ip`:$QUEST_PORT") {
            Write-ColorOutput "Successfully connected to Quest!" "Green"
            return $true
        }
        
        # Disconnect with timeout
        $disconnectJob = Start-Job -ScriptBlock {
            param($ip, $port)
            & adb disconnect "$ip`:$port"
        } -ArgumentList $ip, $QUEST_PORT
        
        Wait-Job -Job $disconnectJob -Timeout 5 > $null
        Stop-Job -Job $disconnectJob
        Remove-Job -Job $disconnectJob -Force
    }
    
    Write-ColorOutput "Could not find Quest on network" "Red"
    return $false
}

function Restart-App {
    Write-Output "Restarting QuestNav..."
    
    # Store connection information with timeout
    $deviceJob = Start-Job -ScriptBlock { & adb devices }
    $deviceCompleted = Wait-Job -Job $deviceJob -Timeout 5
    
    if ($null -eq $deviceCompleted) {
        Write-ColorOutput "Device check timed out." "Red"
        Stop-Job -Job $deviceJob
        Remove-Job -Job $deviceJob -Force
        return $false
    }
    
    $devices = Receive-Job -Job $deviceJob
    Remove-Job -Job $deviceJob
    
    # Determine connection type (USB or Network)
    $isUSB = $false
    $isNetwork = $false
    $deviceId = $null
    $deviceIp = $null
    
    foreach ($line in $devices) {
        # Check for USB connection - device ID without port
        if ($line -match "^([0-9A-Za-z]+)[^:]*device$") {
            $isUSB = $true
            $deviceId = $matches[1]
            Write-Output "Detected USB-connected device: $deviceId"
            break
        }
        # Check for network connection
        elseif ($line -match "([0-9.]+):$QUEST_PORT") {
            $isNetwork = $true
            $deviceIp = $matches[1]
            Write-Output "Detected network-connected device: $deviceIp`:$QUEST_PORT"
            break
        }
    }
    
    if (-not ($isUSB -or $isNetwork)) {
        Write-ColorOutput "Error: No connected Quest devices found." "Red"
        return $false
    }
    
    # Force stop the app with timeout
    Write-Output "Stopping QuestNav app..."
    $stopJob = Start-Job -ScriptBlock {
        param($package)
        try {
            # Redirect output to avoid PowerShell treating it as an error
            $psi = New-Object System.Diagnostics.ProcessStartInfo
            $psi.FileName = "adb"
            $psi.Arguments = "shell am force-stop $package"
            $psi.UseShellExecute = $false
            $psi.RedirectStandardOutput = $true
            $psi.RedirectStandardError = $true
            
            $process = [System.Diagnostics.Process]::Start($psi)
            $stdout = $process.StandardOutput.ReadToEnd()
            $stderr = $process.StandardError.ReadToEnd()
            $process.WaitForExit()
            
            return @{
                ExitCode = $process.ExitCode
                StdOut = $stdout
                StdErr = $stderr
            }
        } catch {
            return @{
                ExitCode = 1
                StdOut = ""
                StdErr = "Error: $_"
            }
        }
    } -ArgumentList $APP_PACKAGE
    
    $stopCompleted = Wait-Job -Job $stopJob -Timeout 10
    
    if ($null -eq $stopCompleted) {
        Write-ColorOutput "App stop command timed out." "Red"
        Stop-Job -Job $stopJob
        Remove-Job -Job $stopJob -Force
        return $false
    }
    
    $stopResult = Receive-Job -Job $stopJob
    Remove-Job -Job $stopJob
    
    if ($stopResult.ExitCode -ne 0) {
        Write-ColorOutput "Error stopping app: $($stopResult.StdErr)" "Red"
    }
    
    # Give a moment for the app to stop
    Start-Sleep -Seconds 3
    
    # Start the app directly with timeout
    Write-Output "Starting QuestNav app..."
    $startJob = Start-Job -ScriptBlock {
        param($package)
        try {
            # Redirect output to avoid PowerShell treating it as an error
            $psi = New-Object System.Diagnostics.ProcessStartInfo
            $psi.FileName = "adb"
            $psi.Arguments = "shell monkey -p $package 1"
            $psi.UseShellExecute = $false
            $psi.RedirectStandardOutput = $true
            $psi.RedirectStandardError = $true
            
            $process = [System.Diagnostics.Process]::Start($psi)
            $stdout = $process.StandardOutput.ReadToEnd()
            $stderr = $process.StandardError.ReadToEnd()
            $process.WaitForExit()
            
            # Return a custom object with exit code and output
            return @{
                ExitCode = $process.ExitCode
                StdOut = $stdout
                StdErr = $stderr
            }
        } catch {
            return @{
                ExitCode = 1
                StdOut = ""
                StdErr = "Error: $_"
            }
        }
    } -ArgumentList $APP_PACKAGE
    
    $startCompleted = Wait-Job -Job $startJob -Timeout 10
    
    if ($null -eq $startCompleted) {
        Write-ColorOutput "App start command timed out." "Red"
        Stop-Job -Job $startJob
        Remove-Job -Job $startJob -Force
        return $false
    }
    
    $result = Receive-Job -Job $startJob
    Remove-Job -Job $startJob
    
    # Check for real errors, not just the normal monkey output
    if ($result.ExitCode -ne 0 -or $result.StdErr -match "Error:") {
        Write-ColorOutput "Error starting app: $($result.StdErr)" "Red"
        return $false
    }
    
    Write-Output "App launch command sent successfully."
    if ($result.StdOut -match "args:") {
        Write-Output "Note: The 'args' output is normal diagnostic info from the monkey tool, not an error."
    }
    
    # Wait a moment for the app to start
    Start-Sleep -Seconds 5
    
    # Check device status after restart
    $checkJob = Start-Job -ScriptBlock { & adb devices }
    $checkCompleted = Wait-Job -Job $checkJob -Timeout 5
    
    if ($null -eq $checkCompleted) {
        Write-ColorOutput "Device check timed out." "Red"
        Stop-Job -Job $checkJob
        Remove-Job -Job $checkJob -Force
        return $false
    }
    
    $devices = Receive-Job -Job $checkJob
    Remove-Job -Job $checkJob
    
    # Check connection based on original connection type
    if ($isNetwork) {
        # For network connection, check if we need to reconnect
        if ($devices -notmatch "$QUEST_PORT") {
            Write-Output "Network reset detected, attempting to reconnect..."
            
            $disconnectJob = Start-Job -ScriptBlock { & adb disconnect }
            Wait-Job -Job $disconnectJob -Timeout 5 > $null
            Stop-Job -Job $disconnectJob
            Remove-Job -Job $disconnectJob -Force
            
            Start-Sleep -Seconds 2
            
            # Try to reconnect multiple times with increasing delays
            for ($i = 1; $i -le 5; $i++) {
                Write-Output "Reconnection attempt $i..."
                
                $reconnectJob = Start-Job -ScriptBlock {
                    param($ip, $port)
                    & adb connect "$ip`:$port"
                } -ArgumentList $deviceIp, $QUEST_PORT
                
                $reconnectCompleted = Wait-Job -Job $reconnectJob -Timeout 10
                
                if ($null -eq $reconnectCompleted) {
                    Write-ColorOutput "Reconnection attempt $i timed out." "Red"
                    Stop-Job -Job $reconnectJob
                    Remove-Job -Job $reconnectJob -Force
                    Start-Sleep -Seconds ($i * 2)
                    continue
                }
                
                Receive-Job -Job $reconnectJob > $null
                Remove-Job -Job $reconnectJob
                
                # Increasing delay between attempts
                Start-Sleep -Seconds ($i * 2)
                
                $checkJob = Start-Job -ScriptBlock { & adb devices }
                $checkCompleted = Wait-Job -Job $checkJob -Timeout 5
                
                if ($null -eq $checkCompleted) {
                    Write-ColorOutput "Device check timed out on attempt $i." "Red"
                    Stop-Job -Job $checkJob
                    Remove-Job -Job $checkJob -Force
                    continue
                }
                
                $devices = Receive-Job -Job $checkJob
                Remove-Job -Job $checkJob
                
                if ($devices -match "$QUEST_PORT") {
                    Write-ColorOutput "Successfully reconnected!" "Green"
                    Write-ColorOutput "App restarted successfully" "Green"
                    return $true
                }
            }
            
            Write-ColorOutput "Failed to reconnect. The network interface might need more time to stabilize." "Red"
            Write-Output "You can try: .\quest-tools.ps1 connect [team] to reconnect manually."
            return $false
        }
    } 
    elseif ($isUSB) {
        # For USB connection, just check if the device is still connected
        if ($devices -notmatch "device$") {
            Write-ColorOutput "USB connection lost after restart." "Red"
            Write-Output "Check if the Quest is still connected via USB."
            return $false
        }
    }
    
    Write-ColorOutput "App restarted successfully" "Green"
    return $true
}

function Check-Device-Status {
    Write-ColorOutput "Checking for connected devices..." "Green"
    
    if (-not (Check-Adb)) {
        return $false
    }
    
    # Make sure ADB server is running
    $serverJob = Start-Job -ScriptBlock { & adb start-server }
    $serverCompleted = Wait-Job -Job $serverJob -Timeout 10
    
    if ($null -eq $serverCompleted) {
        Write-ColorOutput "ADB server start timed out." "Red"
        Stop-Job -Job $serverJob
        Remove-Job -Job $serverJob -Force
        
        # Try to kill and restart
        Write-Output "Attempting to kill and restart ADB server..."
        Force-Kill-Adb
        
        $restartJob = Start-Job -ScriptBlock { & adb start-server }
        $restartCompleted = Wait-Job -Job $restartJob -Timeout 10
        
        if ($null -eq $restartCompleted) {
            Write-ColorOutput "ADB server restart also timed out. Try running 'kill' command." "Red"
            Stop-Job -Job $restartJob
            Remove-Job -Job $restartJob -Force
            return $false
        }
        
        Receive-Job -Job $restartJob > $null
        Remove-Job -Job $restartJob
    } else {
        Receive-Job -Job $serverJob > $null
        Remove-Job -Job $serverJob
    }
    
    # Check for connected devices with timeout
    $devicesJob = Start-Job -ScriptBlock { & adb devices }
    $devicesCompleted = Wait-Job -Job $devicesJob -Timeout 10
    
    if ($null -eq $devicesCompleted) {
        Write-ColorOutput "ADB devices command timed out." "Red"
        Stop-Job -Job $devicesJob
        Remove-Job -Job $devicesJob -Force
        Write-ColorOutput "Try running the 'kill' command and then check status again." "Red"
        return $false
    }
    
    $devices = Receive-Job -Job $devicesJob
    Remove-Job -Job $devicesJob
    
    Write-Output "Connected devices:"
    Write-Output $devices
    
    $foundUSB = $false
    $foundNetwork = $false
    
    # Check for USB devices and network devices
    foreach ($line in $devices) {
        if ($line -match "device$" -and $line -notmatch ":") {
            Write-ColorOutput "USB device detected." "Green"
            $foundUSB = $true
        } elseif ($line -match "$QUEST_PORT") {
            Write-ColorOutput "Network device detected." "Green"
            $foundNetwork = $true
        }
    }
    
    if (-not ($foundUSB -or $foundNetwork)) {
        Write-ColorOutput "No Quest devices found. Make sure the Quest is connected and USB debugging is enabled." "Yellow"
        Write-Output "Tips:"
        Write-Output "  1. For USB connection: Verify the Quest is connected via USB and USB debugging is enabled"
        Write-Output "  2. For network connection: Use 'connect [team]' to establish a network connection"
        return $false
    }
    
    return $true
}

function Enable-Wireless-Debugging {
    Write-ColorOutput "Enabling wireless debugging..." "Green"
    
    if (-not (Check-Adb)) {
        return $false
    }
    
    # Check for USB connection
    $deviceJob = Start-Job -ScriptBlock { & adb devices }
    $deviceCompleted = Wait-Job -Job $deviceJob -Timeout 5
    
    if ($null -eq $deviceCompleted) {
        Write-ColorOutput "Device check timed out." "Red"
        Stop-Job -Job $deviceJob
        Remove-Job -Job $deviceJob -Force
        return $false
    }
    
    $devices = Receive-Job -Job $deviceJob
    Remove-Job -Job $deviceJob
    
    $isUSB = $false
    $deviceId = $null
    
    foreach ($line in $devices) {
        # Check for USB connection - device ID without port
        if ($line -match "^([0-9A-Za-z]+)[^:]*device$") {
            $isUSB = $true
            $deviceId = $matches[1]
            break
        }
    }
    
    if (-not $isUSB) {
        Write-ColorOutput "Error: No USB-connected Quest found. Wireless debugging can only be enabled via USB." "Red"
        Write-Output "Please connect your Quest via USB first."
        return $false
    }
    
    # Prompt user to connect Ethernet
    Write-ColorOutput "IMPORTANT: Please connect your Quest to Ethernet network now." "Yellow"
    Write-Output "This will allow wireless debugging over the robot's network."
    Write-Output "Press Enter once the Quest is connected to Ethernet..."
    $null = Read-Host
    
    # Wait for Ethernet connection to stabilize
    Write-Output "Waiting for Ethernet connection to stabilize..."
    Start-Sleep -Seconds 5
    
    # Get device IP address (checking Ethernet interfaces)
    Write-Output "Getting device IP address..."
    $ipJob = Start-Job -ScriptBlock {
        # Try to get ethernet IP address - check multiple possible interfaces
        $output = & adb shell "ip addr show | grep 'inet ' | grep '10.51.52'"
        return $output
    }
    $ipCompleted = Wait-Job -Job $ipJob -Timeout 10
    
    if ($null -eq $ipCompleted) {
        Write-ColorOutput "IP address command timed out." "Red"
        Stop-Job -Job $ipJob
        Remove-Job -Job $ipJob -Force
        return $false
    }
    
    $ipAddressOutput = Receive-Job -Job $ipJob
    Remove-Job -Job $ipJob
    
    # Extract IP address from the output
    $ipAddress = $null
    if ($ipAddressOutput -match "inet\s+(\d+\.\d+\.\d+\.\d+)") {
        $ipAddress = $matches[1]
    }
    
    if (-not $ipAddress) {
        Write-ColorOutput "Error: Could not find an IP address on the 10.51.52.X subnet." "Red"
        Write-Output "Output from IP check: $ipAddressOutput"
        Write-Output "Please make sure the Quest is connected to the robot's Ethernet network."
        
        # Let's try to list all interfaces to help debugging
        Write-Output "Available network interfaces:"
        $interfacesJob = Start-Job -ScriptBlock {
            & adb shell "ip addr"
        }
        $interfacesCompleted = Wait-Job -Job $interfacesJob -Timeout 10
        if ($null -ne $interfacesCompleted) {
            $interfaces = Receive-Job -Job $interfacesJob
            Write-Output $interfaces
            Remove-Job -Job $interfacesJob
        } else {
            Write-Output "Could not retrieve network interfaces."
            Stop-Job -Job $interfacesJob
            Remove-Job -Job $interfacesJob -Force
        }
        
        return $false
    }
    
    Write-ColorOutput "Device IP address: $ipAddress" "Green"
    
    # Enable TCP/IP mode
    Write-Output "Enabling TCP/IP mode on port $QUEST_PORT..."
    $tcpipJob = Start-Job -ScriptBlock {
        param($port)
        & adb tcpip $port
    } -ArgumentList $QUEST_PORT
    
    $tcpipCompleted = Wait-Job -Job $tcpipJob -Timeout 10
    
    if ($null -eq $tcpipCompleted) {
        Write-ColorOutput "TCP/IP command timed out." "Red"
        Stop-Job -Job $tcpipJob
        Remove-Job -Job $tcpipJob -Force
        return $false
    }
    
    $tcpipOutput = Receive-Job -Job $tcpipJob
    Remove-Job -Job $tcpipJob
    
    Write-Output $tcpipOutput
    
    # Wait for TCP/IP mode to initialize
    Start-Sleep -Seconds 3
    
    Write-ColorOutput "Wireless debugging enabled successfully!" "Green"
    Write-Output "You can now disconnect the USB cable and connect wirelessly with:"
    Write-Output ".\quest-tools.ps1 connect 5152"  # Using the specific team number for this use case
    Write-Output ""
    Write-Output "Or use ADB directly:"
    Write-Output "adb connect $ipAddress`:$QUEST_PORT"
    
    # Attempt immediate connection to the wireless device
    Write-Output "Attempting to connect to the Quest wirelessly now..."
    Start-Sleep -Seconds 2
    
    $connectJob = Start-Job -ScriptBlock {
        param($ip, $port)
        & adb connect "$ip`:$port"
    } -ArgumentList $ipAddress, $QUEST_PORT
    
    $connectCompleted = Wait-Job -Job $connectJob -Timeout 10
    
    if ($null -eq $connectCompleted) {
        Write-ColorOutput "Wireless connection attempt timed out." "Yellow"
        Stop-Job -Job $connectJob
        Remove-Job -Job $connectJob -Force
    } else {
        $connectResult = Receive-Job -Job $connectJob
        Remove-Job -Job $connectJob
        Write-Output $connectResult
        
        if ($connectResult -match "connected") {
            Write-ColorOutput "Successfully connected to Quest wirelessly!" "Green"
        } else {
            Write-ColorOutput "Failed to establish wireless connection. Try connecting manually." "Yellow"
        }
    }
    
    return $true
}

# Main command handler
$command = $args[0]
$param = $args[1]

switch ($command) {
    "connect" {
        if ([string]::IsNullOrEmpty($param)) {
            Write-ColorOutput "Error: Parameter required (team number)" "Red"
            break
        }
        if (Check-Adb) {
            Find-Quest $param
        }
    }
    "restart" {
        if (Check-Adb -and (Ensure-Connected)) {
            Restart-App
        }
    }
    "reboot" {
        if (Check-Adb -and (Ensure-Connected)) {
            $rebootJob = Start-Job -ScriptBlock { & adb reboot }
            $completed = Wait-Job -Job $rebootJob -Timeout 10
            
            if ($null -eq $completed) {
                Write-ColorOutput "Reboot command timed out." "Red"
                Stop-Job -Job $rebootJob
                Remove-Job -Job $rebootJob -Force
            } else {
                Receive-Job -Job $rebootJob > $null
                Remove-Job -Job $rebootJob
                Write-ColorOutput "Reboot command sent successfully." "Green"
            }
        }
    }
    "status" {
        Check-Device-Status
    }
    "enableWifi" {
        Enable-Wireless-Debugging
    }
    "kill" {
        Force-Kill-Adb
    }
    default {
        Print-Help
    }
}
