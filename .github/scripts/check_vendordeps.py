import requests
import json
from pathlib import Path
import os
from typing import Dict, Optional
from packaging import version
import sys

class VendorDepChecker:
    def __init__(self):
        self.github_sources = {
            'PathplannerLib': 'https://api.github.com/repos/mjansen4857/pathplanner/releases',
            'AdvantageKit': 'https://api.github.com/repos/Mechanical-Advantage/AdvantageKit/releases',
            'photonlib': 'https://api.github.com/repos/PhotonVision/PhotonVision/releases',
            'WPILib': 'https://api.github.com/repos/wpilibsuite/allwpilib/releases',
            'Phoenix6': 'https://api.github.com/repos/CrossTheRoadElec/Phoenix-Releases/releases'
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

    def sanitize_text(self, text: str) -> str:
        """Remove any @mentions from text content."""
        import re
        # Match @username or @org/repo style mentions
        text = re.sub(r'@[\w-]+(?:/[\w-]+)?', '[mentioned]', text)
        return text

    def get_github_latest(self, repo_url: str, is_wpilib: bool = False) -> Optional[Dict]:
        """Get latest release version from GitHub, including betas."""
        import sys
        try:
            print(f"\nChecking releases from: {repo_url}", file=sys.stderr)
            response = requests.get(repo_url, headers=self.headers)
            response.raise_for_status()
            releases = response.json()

            print(f"Found {len(releases)} total releases", file=sys.stderr)

            # For WPILib, filter releases to only include current year
            if is_wpilib:
                releases = [r for r in releases if '2025' in r['tag_name']]
                print(f"Found {len(releases)} 2025 releases", file=sys.stderr)
                for r in releases[:5]:  # Show first 5
                    print(f"Release: {r['name']} (Tag: {r['tag_name']})", file=sys.stderr)

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
                        'url': release['html_url']
                    })
                    print(f"Successfully parsed version: {tag} -> {ver}", file=sys.stderr)
                except version.InvalidVersion:
                    print(f"Skipping invalid version: {tag}", file=sys.stderr)
                    continue

            # Sort by version, highest first
            parsed_releases.sort(key=lambda x: x['version'], reverse=True)

            if parsed_releases:
                latest = parsed_releases[0]
                print(f"\nSelected latest version: {latest['tag']} (Beta: {latest['is_beta']})", file=sys.stderr)
                return {
                    'version': latest['tag'],
                    'is_beta': latest['is_beta'],
                    'url': latest['url']
                }
            print("No valid releases found", file=sys.stderr)
            return None

        except Exception as e:
            print(f"Error fetching from GitHub: {e}", file=sys.stderr)
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
                for line in content.split('\n'):
                    if 'id "edu.wpi.first.GradleRIO"' in line and 'version' in line:
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
        import sys
        current_ver = self.parse_version(current['version'])
        latest_ver = self.parse_version(latest['version'])

        print(f"\nComparing versions:", file=sys.stderr)
        print(f"Current: {current['version']} (Beta: {current['is_beta']}) -> Parsed as: {current_ver}", file=sys.stderr)
        print(f"Latest: {latest['version']} (Beta: {latest['is_beta']}) -> Parsed as: {latest_ver}", file=sys.stderr)

        # If versions are equal, prefer beta over stable
        if current_ver == latest_ver:
            result = latest['is_beta'] and not current['is_beta']
            print(f"Versions equal, checking beta status -> Update needed: {result}", file=sys.stderr)
            return result

        result = latest_ver > current_ver
        print(f"Comparing versions -> Update needed: {result}", file=sys.stderr)
        return result

    def check_all_updates(self):
        """Check all vendordeps for updates."""
        import sys
        updates = []
        vendordeps_dir = Path('vendordeps')

        print("Starting dependency check...", file=sys.stderr)

        # Check all GitHub-based vendordeps (including Phoenix6)
        for name, url in self.github_sources.items():
            print(f"\nChecking {name}...", file=sys.stderr)

            if name == 'WPILib':
                current = self.get_wpilib_current_version()
                latest = self.get_github_latest(url, is_wpilib=True)
            else:
                current_file = next(vendordeps_dir.glob(f"{name}*.json"), None)
                if not current_file:
                    print(f"No vendordep file found for {name}", file=sys.stderr)
                    continue
                print(f"Found vendordep file: {current_file}", file=sys.stderr)
                current = self.get_current_version(current_file)
                latest = self.get_github_latest(url)

            if current and latest:
                print(f"Current version: {current['version']}", file=sys.stderr)
                print(f"Latest version: {latest['version']}", file=sys.stderr)

                if self.is_newer_version(current, latest):
                    print(f"Update available for {name}", file=sys.stderr)
                    updates.append({
                        'name': name,
                        'current': current['version'],
                        'current_is_beta': current['is_beta'],
                        'latest': latest['version'],
                        'latest_is_beta': latest['is_beta'],
                        'source': 'GitHub',
                        'url': latest.get('url')
                    })
            else:
                print(f"Failed to get version info for {name}", file=sys.stderr)

        return updates

if __name__ == "__main__":
    checker = VendorDepChecker()
    updates = checker.check_all_updates()

    # Set output for GitHub Actions
    if updates:
        print("\nFound updates:", file=sys.stderr)
        for update in updates:
            print(f"- {update['name']}: {update['current']} -> {update['latest']}", file=sys.stderr)

        # GitHub Actions output syntax
        result = {
            "has_updates": True,
            "updates": updates
        }
        print(json.dumps(result))
    else:
        print("\nNo updates found.", file=sys.stderr)
        result = {
            "has_updates": False,
            "updates": []
        }
        print(json.dumps(result))