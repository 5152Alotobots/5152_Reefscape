#!/bin/bash

APP_PACKAGE="com.DerpyCatAviationLLC.QuestNav"
QUEST_PORT=5555

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_help() {
    echo -e "${YELLOW}Quest Development Tools${NC}"
    echo "Usage: ./quest-tools.sh [command] [team_number]"
    echo ""
    echo "Commands:"
    echo "  connect <team>    - Find and connect to Quest on robot network (e.g., 5152)"
    echo "  stopwireless     - Disable WiFi and Bluetooth and reboot"
    echo "  setup           - Disable guardian and restart app"
    echo "  restart         - Restart the QuestNav app"
    echo "  redeploy <path>   - Install APK from path and restart app"
    echo "  screen          - Launch scrcpy for screen mirroring"
    echo "  reboot          - Reboot the Quest"
    echo "  shutdown        - Shutdown the Quest"
    echo "  status          - Check ADB connection status"
    echo "  logs [filter]    - Show app logs, optionally filtered by string"
}

check_adb() {
    if ! command -v adb &> /dev/null; then
        echo -e "${RED}Error: ADB not found. Please install Android platform tools.${NC}"
        exit 1
    fi
}

ensure_connected() {
    if ! adb devices | grep -q "$QUEST_PORT"; then
        echo -e "${RED}Error: Quest not connected. Use 'connect' command first.${NC}"
        exit 1
    fi
}

find_quest() {
    local team=$1
    local subnet="10.${team:0:2}.${team:2:2}"

    echo -e "${GREEN}Scanning network ${subnet}.0/24 for Quest...${NC}"

    # Check if nmap is installed
    if ! command -v nmap &> /dev/null; then
        echo -e "${RED}Error: nmap not found. Please install nmap first.${NC}"
        exit 1
    fi

    # Kill any existing ADB server first
    adb kill-server &> /dev/null
    adb start-server &> /dev/null

    # Use nmap to quickly scan network, excluding .1 and .2
    echo "Running quick network scan..."
    local devices=$(nmap -n -sn --exclude ${subnet}.1,${subnet}.2 ${subnet}.0/24 | grep "report for" | cut -d " " -f 5)

    for ip in $devices; do
        echo -e "${YELLOW}Found device at ${ip}, attempting ADB connection...${NC}"

        # Try ADB connect with timeout
        timeout 3 adb connect "${ip}:$QUEST_PORT" &> /dev/null

        # Quick check if device is connected
        if timeout 1 adb devices | grep -q "${ip}:$QUEST_PORT"; then
            echo -e "${GREEN}Successfully connected to Quest!${NC}"
            return 0
        fi

        adb disconnect "${ip}:$QUEST_PORT" &> /dev/null
    done

    echo -e "${RED}Could not find Quest on network${NC}"
    return 1
}

stopwireless() {
    ensure_connected
    echo "Disabling wireless connections..."
    adb shell settings put global bluetooth_on 0
    adb shell settings put global wifi_on 0
    echo "Rebooting..."
    adb reboot
}

setup_quest() {
    ensure_connected
    echo "Disabling Guardian..."
    adb shell setprop debug.oculus.guardian_pause 1
    restart_app
}

restart_app() {
    ensure_connected
    echo "Restarting QuestNav..."

    # Store last known device IP
    local device_ip=$(adb devices | grep -m 1 "${QUEST_PORT}" | cut -f1 | cut -d: -f1)

    # Force stop the app
    adb shell am force-stop $APP_PACKAGE

    # Give USB Ethernet interface time to settle
    sleep 3

    # Start the app directly
    adb shell monkey -p $APP_PACKAGE 1

    # Wait for potential network reset
    sleep 5

    # Check if we need to reconnect
    if ! adb devices | grep -q "$QUEST_PORT"; then
        echo "Network reset detected, reconnecting..."
        adb disconnect
        sleep 2

        # Try to reconnect multiple times with increasing delays
        for i in {1..5}; do
            echo "Reconnection attempt $i..."
            adb connect "${device_ip}:${QUEST_PORT}"
            sleep $(($i * 2))  # Increasing delay between attempts
            if adb devices | grep -q "$QUEST_PORT"; then
                echo -e "${GREEN}Successfully reconnected!${NC}"
                return 0
            fi
        done
        echo -e "${RED}Failed to reconnect. The network interface might need more time to stabilize.${NC}"
        echo "You can try: ./quest-tools.sh connect <team> to reconnect manually."
        return 1
    fi

    echo -e "${GREEN}App restarted successfully${NC}"
}

redeploy() {
    ensure_connected
    local apk_path="$1"

    if [ -z "$apk_path" ]; then
        echo -e "${RED}Error: APK path required${NC}"
        echo "Usage: ./quest-tools.sh redeploy path/to/app.apk"
        exit 1
    fi

    if [ ! -f "$apk_path" ]; then
        echo -e "${RED}Error: APK file not found: $apk_path${NC}"
        exit 1
    fi

    echo "Installing APK from: $apk_path"
    adb install -r -d "$apk_path"
    restart_app
}

launch_scrcpy() {
    ensure_connected
    if ! command -v scrcpy &> /dev/null; then
        echo -e "${RED}Error: scrcpy not found. Please install it first.${NC}"
        exit 1
    fi
    scrcpy --crop 1920:1080:0:0 -b 10M
}

tail_logs() {
    ensure_connected
    local filter="$1"
    echo "Starting log stream..."
    if [ -z "$filter" ]; then
        # No filter, show all Unity logs
        adb logcat -s Unity:*
    else
        # Use exact working command format
        adb logcat -s Unity:* | grep "\\${filter}"
    fi
}

# Main command handler
case "$1" in
    "connect")
        if [ -z "$2" ]; then
            echo -e "${RED}Error: Team number required${NC}"
            exit 1
        fi
        check_adb
        find_quest "$2"
        ;;
    "stopwireless")
        check_adb
        stopwireless
        ;;
    "setup")
        check_adb
        setup_quest
        ;;
    "restart")
        check_adb
        restart_app
        ;;
    "redeploy")
        check_adb
        redeploy "$2"
        ;;
    "screen")
        check_adb
        launch_scrcpy
        ;;
    "reboot")
        check_adb
        ensure_connected
        adb reboot
        ;;
    "shutdown")
        check_adb
        ensure_connected
        adb shell reboot -p
        ;;
    "status")
        check_adb
        adb devices
        ;;
    "logs")
        check_adb
        tail_logs "$2"
        ;;
    *|"help")
        print_help
        ;;
esac
