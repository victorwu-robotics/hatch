"""
Package Resolver - Resolves ROS-style package references to filesystem paths.

Searches HATCH_PACKAGE_PATH (defaulting to ~/hatch/assets/) for packages
containing URDF files, meshes, and other resources.

Principle: Everything in URDF. Package paths must be portable.
"""

import os
import logging
from pathlib import Path
from typing import Optional, List

logger = logging.getLogger(__name__)


class PackageResolver:
    """
    Resolves package:// URIs and $(find package) references to filesystem paths.
    
    Searches directories defined by HATCH_PACKAGE_PATH environment variable,
    defaulting to ~/hatch/assets/ if not set.
    Search is recursive — packages can be in subdirectories like robots/, sensors/, tools/.
    Handles incorrectly-generated URIs where 'robots/' etc. got baked into package names.
    """

    def __init__(self, package_path: Optional[str] = None):
        """
        Initialize the package resolver.

        Args:
            package_path: Colon-separated (Unix) or semicolon-separated (Windows)
                         list of directories to search for packages.
                         Defaults to HATCH_PACKAGE_PATH env var or ~/hatch/assets/
        """
        if package_path is None:
            package_path = os.environ.get('HATCH_PACKAGE_PATH', '')

        if not package_path:
            package_path = str(Path.home() / 'hatch' / 'assets')

        # Split by platform-appropriate separator
        if ';' in package_path:
            self.package_dirs = [Path(p.strip()).expanduser().resolve()
                                 for p in package_path.split(';') if p.strip()]
        else:
            self.package_dirs = [Path(p.strip()).expanduser().resolve()
                                 for p in package_path.split(':') if p.strip()]

        # Ensure default assets directory exists in search path
        default_assets = Path.home() / 'hatch' / 'assets'
        if default_assets.exists() and default_assets not in self.package_dirs:
            self.package_dirs.append(default_assets)

        logger.debug(f"PackageResolver initialized with dirs: {self.package_dirs}")

    def find_package(self, package_name: str) -> Optional[Path]:
        """
        Find a package directory by name.
        Searches recursively through subdirectories of all package_dirs.

        Args:
            package_name: Name of the package (e.g., 'realsense2_description')

        Returns:
            Path to the package directory, or None if not found
        """
        # Strip any subdirectory prefix (robots/, sensors/, tools/, scenes/)
        clean_name = self._strip_prefix(package_name)

        for base_dir in self.package_dirs:
            # Direct match first
            candidate = base_dir / clean_name
            if candidate.is_dir():
                logger.debug(f"Found package '{package_name}' at: {candidate}")
                return candidate

            # Recursive search
            if base_dir.exists():
                result = self._search_recursive(base_dir, clean_name, depth=3)
                if result:
                    logger.debug(f"Found package '{package_name}' at: {result}")
                    return result

        return None

    def _strip_prefix(self, package_name: str) -> str:
        """Strip known subdirectory prefixes from a package name."""
        known_prefixes = ['robots/', 'sensors/', 'tools/', 'scenes/']
        for prefix in known_prefixes:
            if package_name.startswith(prefix):
                return package_name[len(prefix):]
        return package_name

    def _search_recursive(self, directory: Path, package_name: str, depth: int) -> Optional[Path]:
        """Recursively search for a package directory."""
        if depth <= 0:
            return None

        try:
            for item in directory.iterdir():
                if item.is_dir():
                    if item.name == package_name:
                        return item
                    result = self._search_recursive(item, package_name, depth - 1)
                    if result:
                        return result
        except (PermissionError, OSError):
            pass

        return None

    def resolve_package_path(self, package_name: str, relative_path: str) -> Optional[Path]:
        """
        Resolve a path relative to a package root.

        Args:
            package_name: Name of the package
            relative_path: Path relative to the package root

        Returns:
            Full path to the resource, or None if not found
        """
        package_dir = self.find_package(package_name)
        if package_dir is None:
            return None

        resolved = (package_dir / relative_path).resolve()
        if resolved.exists():
            return resolved
        return None

    def resolve_package_uri(self, uri: str) -> Optional[Path]:
        """
        Resolve a package:// URI to a filesystem path.

        Args:
            uri: URI in format 'package://package_name/path/to/file'

        Returns:
            Full path to the resource, or None if not found
        """
        if not uri.startswith('package://'):
            return None

        path_part = uri[10:]  # len('package://') == 10
        parts = path_part.split('/', 1)
        if len(parts) != 2:
            logger.warning(f"Invalid package URI: {uri}")
            return None

        package_name, relative_path = parts
        return self.resolve_package_path(package_name, relative_path)

    def get_all_package_dirs(self) -> List[Path]:
        """Get all package search directories."""
        return self.package_dirs.copy()