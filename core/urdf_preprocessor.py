"""
URDF Preprocessor - Loads URDF and xacro files, outputs plain URDF XML.

Orchestrates the preprocessing pipeline:
1. Detects whether the file is xacro or plain URDF
2. Expands xacro using XacroExpander if needed
3. Returns plain URDF XML string ready for KinematicModel

Principle: Everything in URDF. The preprocessor is invisible to the user.
"""

import logging
from pathlib import Path
from typing import Optional

from utils.package_resolver import PackageResolver
from utils.xacro_expander import XacroExpander

logger = logging.getLogger(__name__)


class URDFPreprocessor:
    """
    Loads a URDF or xacro file and produces plain URDF XML.
    
    Handles both .urdf (pass-through) and .xacro / .urdf.xacro (expansion).
    All path references (package://, $(find ...), relative paths)
    are preserved as-is for KinematicModel to resolve later.
    """

    def __init__(self, package_resolver: Optional[PackageResolver] = None):
        """
        Initialize the preprocessor.

        Args:
            package_resolver: Shared PackageResolver instance.
                             If None, creates a default one.
        """
        self.package_resolver = package_resolver or PackageResolver()
        self._expander = XacroExpander(self.package_resolver)

    def process(self, filepath: str) -> str:
        """
        Process a URDF or xacro file and return plain URDF XML string.

        Args:
            filepath: Path to the .urdf, .xacro, or .urdf.xacro file.

        Returns:
            Plain URDF XML as a string, ready for KinematicModel to parse.

        Raises:
            FileNotFoundError: If the file doesn't exist.
        """
        filepath = Path(filepath).expanduser().resolve()

        if not filepath.exists():
            raise FileNotFoundError(f"URDF file not found: {filepath}")

        logger.info(f"Processing: {filepath}")

        # Detect file type
        if self._is_xacro(filepath):
            logger.info(f"Detected xacro file, expanding...")
            return self._expander.expand(str(filepath))
        else:
            logger.info(f"Plain URDF file, reading directly...")
            return filepath.read_text(encoding='utf-8')

    def _is_xacro(self, filepath: Path) -> bool:
        """
        Detect whether a file is a xacro file.
        
        Checks:
        1. File extension (.xacro or .urdf.xacro)
        2. File content (contains xmlns:xacro or xacro: tags)
        """
        # Check by extension
        if filepath.suffix == '.xacro':
            return True
        if filepath.name.endswith('.urdf.xacro'):
            return True

        # Check by content
        try:
            content = filepath.read_text(encoding='utf-8')
            if 'xmlns:xacro' in content or '<xacro:' in content:
                return True
        except Exception:
            pass

        return False

    def get_expander(self) -> XacroExpander:
        """Get the underlying XacroExpander for direct access if needed."""
        return self._expander