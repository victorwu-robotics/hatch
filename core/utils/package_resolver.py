import os
from pathlib import Path
from typing import List, Optional

class PackageResolver:
    """Resolves package names to file paths using user-defined search paths."""
    
    def __init__(self, search_paths: Optional[List[str]] = None):
        """
        Initialize the resolver.
        
        Args:
            search_paths: Explicit list of directories to search.
                          If None, reads from HATCH_PACKAGE_PATH env var.
        """
        if search_paths is not None:
            self.search_paths = [
                Path(p).expanduser().resolve()
                for p in search_paths
            ]
        else:
            self.search_paths = self._get_default_paths()
    
    def _get_default_paths(self) -> List[Path]:
        """Determine search paths from environment or fallback to CWD."""
        env = os.environ.get("HATCH_PACKAGE_PATH")
        if env:
            return [
                Path(p).expanduser().resolve()
                for p in env.split(":")
            ]
        # Fallback: current working directory
        return [Path.cwd()]
    
    def resolve(self, package_name: str) -> Optional[Path]:
        """Find a package directory by name (searching one level deep)."""
        for base in self.search_paths:
            # Search directly in the base directory
            candidate = base / package_name
            if candidate.is_dir():
                return candidate
            
            # Search one level deep (immediate subdirectories)
            for subdir in base.iterdir():
                if subdir.is_dir():
                    candidate = subdir / package_name
                    if candidate.is_dir():
                        return candidate
        
        return None
    
    def resolve_file(self, package_name: str, relative_path: str) -> Optional[Path]:
        """Resolve a file within a package."""
        pkg_dir = self.resolve(package_name)
        if not pkg_dir:
            return None
        candidate = pkg_dir / relative_path
        return candidate if candidate.exists() else None