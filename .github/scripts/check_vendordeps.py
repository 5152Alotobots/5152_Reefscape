import requests
import json
from pathlib import Path
import os
from typing import Dict, Optional
from packaging import version

class VendorDepChecker:
    def __init__(self):
         self.github_sources = {
             'PathplannerLib': 'https://api.github.com/repos/mjansen4857/pathplanner/releases',
             'AdvantageKit': 'https://api.github.com/repos/Mechanical-Advantage/AdvantageKit/releases',
             'photonlib': 'https://api.github.com/repos/PhotonVision/PhotonVision/releases',
             'WPILib': 'https://api.github.com/repos/wpilibsuite/allwpilib/releases'
         }
         self.phoenix_urls = {
             'version': 'https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest',
             'json': 'https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-beta-latest.json'
         }

         self.github_token = os.getenv('GITHUB_TOKEN')
         self.headers = {
             'Accept': 'application/vnd.github.v3+json'
         }
         if self.github_token:
             self.headers['Authorization'] = f'token {self.github_token}'

    def parse_version(self, ver_str: str) -> version.Version:
        """Convert version string to comparable version object."""
        # Remove 'v' prefix if present
        ver_str = ver_str.lstrip('v')
        try:
            return version.parse(ver_str)
        except version.InvalidVersion:
            # Handle special cases like 'beta' or 'rc' if needed
            return version.parse('0.0.0')

    def get_github_latest(self, repo_url: str, is_wpilib: bool = False) -> Optional[Dict]:
        """Get latest release version from GitHub, including betas."""
        try:
            print(f"\nChecking releases from: {repo_url}")
            response = requests.get(repo_url, headers=self.headers)
            response.raise_for_status()
            releases = response.json()

            print(f"Found {len(releases)} total releases")

            # For WPILib, filter releases to only include current year
            if is_wpilib:
                releases = [r for r in releases if '2025' in r['tag_name']]
                print(f"Found {len(releases)} 2025 releases")
                for r in releases[:5]:  # Show first 5
                    print(f"Release: {r['name']} (Tag: {r['tag_name']})")

            # Convert all versions to comparable objects
            parsed_releases = []
            for release in releases:
                tag = release['tag_name'].lstrip('v')
                try:
                    ver = self.parse_version(tag)
                    parsed_releases.append({
                        'version': ver,
                        'tag': tag,
                        'is_beta': release.get('prerelease', False) or 'beta' in tag.lower(),
                        'url': release['html_url'],
                        'body': release.get('body', '')
                    })
                    print(f"Successfully parsed version: {tag} -> {ver}")
                except version.InvalidVersion:
                    print(f"Skipping invalid version: {tag}")
                    continue

            # Sort by version, highest first
            parsed_releases.sort(key=lambda x: x['version'], reverse=True)

            if parsed_releases:
                latest = parsed_releases[0]
                print(f"\nSelected latest version: {latest['tag']} (Beta: {latest['is_beta']})")
                return {
                    'version': latest['tag'],
                    'is_beta': latest['is_beta'],
                    'url': latest['url'],
                    'notes': latest['body']
                }
            print("No valid releases found")
            return None

        except Exception as e:
            print(f"Error fetching from GitHub: {e}")
            return None

        except Exception as e:
            print(f"Error fetching from GitHub: {e}")
            return None

    def get_phoenix_latest(self) -> Dict:
        """Get latest Phoenix version info from JSON."""
        try:
            json_data = requests.get(self.phoenix_urls['json']).json()
            return {
                'version': json_data.get('version'),  # Get version directly from JSON
                'data': json_data,
                'is_beta': 'beta' in json_data.get('version', '').lower()
            }
        except Exception as e:
            print(f"Error fetching Phoenix version: {e}")
            return None

    def get_current_version(self, file_path: Path) -> Dict:
        """Get current version from vendordep file."""
        try:
            with open(file_path) as f:
                data = json.load(f)
                current_ver = data.get('version')
                if current_ver:
                    return {
                        'version': current_ver,
                        'is_beta': 'beta' in current_ver.lower()
                    }
                return None
        except Exception as e:
            print(f"Error reading {file_path}: {e}")
            return None

    def get_current_version(self, file_path: Path) -> Dict:
        """Get current version from vendordep file."""
        try:
            with open(file_path) as f:
                data = json.load(f)
                current_ver = data.get('version')
                return {
                    'version': current_ver,
                    'is_beta': 'beta' in current_ver.lower()
                }
        except Exception as e:
            print(f"Error reading {file_path}: {e}")
            return None

    def get_wpilib_current_version(self) -> Dict:
        """Get current WPILib version from build.gradle."""
        try:
            gradle_props_path = Path('build.gradle')
            if not gradle_props_path.exists():
                return None

            with open(gradle_props_path) as f:
                content = f.read()
                # Look for GradleRIO version in plugins block
                for line in content.split('\n'):
                    if 'id "edu.wpi.first.GradleRIO"' in line and 'version' in line:
                        # Extract version from line like: id "edu.wpi.first.GradleRIO" version "2025.1.1-beta-2"
                        version_str = line.split('version')[1].strip().strip('"').strip("'")
                        return {
                            'version': version_str,
                            'is_beta': 'beta' in version_str.lower()
                        }
            return None
        except Exception as e:
            print(f"Error reading WPILib version: {e}")
            return None

    def is_newer_version(self, current: Dict, latest: Dict) -> bool:
        """Compare versions, considering betas as newer than releases."""
        current_ver = self.parse_version(current['version'])
        latest_ver = self.parse_version(latest['version'])

        # If versions are equal, prefer beta over stable
        if current_ver == latest_ver:
            return latest['is_beta'] and not current['is_beta']

        return latest_ver > current_ver

    def check_all_updates(self):
        """Check all vendordeps for updates."""
        updates = []
        vendordeps_dir = Path('vendordeps')

        # Check GitHub-based vendordeps
        for name, url in self.github_sources.items():
            if name == 'WPILib':
                current = self.get_wpilib_current_version()
                latest = self.get_github_latest(url, is_wpilib=True)
            else:
                current_file = next(vendordeps_dir.glob(f"{name}*.json"), None)
                if not current_file:
                    continue
                current = self.get_current_version(current_file)
                latest = self.get_github_latest(url)

            if current and latest and self.is_newer_version(current, latest):
                updates.append({
                    'name': name,
                    'current': current['version'],
                    'current_is_beta': current['is_beta'],
                    'latest': latest['version'],
                    'latest_is_beta': latest['is_beta'],
                    'source': 'GitHub',
                    'url': latest.get('url'),
                    'notes': latest.get('notes') if name == 'WPILib' else None
                })

        # Check Phoenix
        phoenix_file = next(vendordeps_dir.glob("Phoenix6*.json"), None)
        if phoenix_file:
            current = self.get_current_version(phoenix_file)
            latest = self.get_phoenix_latest()

            if current and latest and self.is_newer_version(current, latest):
                updates.append({
                    'name': 'Phoenix6',
                    'current': current['version'],
                    'current_is_beta': current['is_beta'],
                    'latest': latest['version'],
                    'latest_is_beta': latest['is_beta'],
                    'source': 'CTRE Maven'
                })

        return updates

def main():
    checker = VendorDepChecker()
    updates = checker.check_all_updates()

    if updates:
        print("\n=== Updates Available ===")
        for update in updates:
            print(f"\n{update['name']}:")
            print(f"  Current: {update['current']} ({'beta' if update['current_is_beta'] else 'stable'})")
            print(f"  Latest:  {update['latest']} ({'beta' if update['latest_is_beta'] else 'stable'})")
            print(f"  Source:  {update['source']}")
            if 'url' in update and update['url']:
                print(f"  URL:     {update['url']}")
            if update.get('notes'):
                print(f"  Release Notes Preview: {update['notes'][:200]}...")

        # Set GitHub Actions output if running in GHA
        if os.getenv('GITHUB_OUTPUT'):
            with open(os.environ['GITHUB_OUTPUT'], 'a') as f:
                f.write(f"has_updates=true\n")
                # Properly escape the JSON string for GitHub Actions
                updates_json = json.dumps(updates).replace('%', '%25').replace('\n', '%0A').replace('\r', '%0D')
                f.write(f"updates={updates_json}\n")
    else:
        print("\nAll dependencies are up to date!")
        if os.getenv('GITHUB_OUTPUT'):
            with open(os.environ['GITHUB_OUTPUT'], 'a') as f:
                f.write("has_updates=false\n")
                f.write("updates=[]\n")
if __name__ == "__main__":
    main()