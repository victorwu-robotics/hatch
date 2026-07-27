import subprocess
import tempfile
import os
from pathlib import Path
from utils.package_resolver import PackageResolver

class XacroExpander:
    """Expands .xacro files to plain URDF XML using the system xacro tool."""
    
    def __init__(self, package_resolver: PackageResolver):
        self.package_resolver = package_resolver
        self.env = os.environ.copy()
        # Set ROS_PACKAGE_PATH so xacro can resolve $(find ...)
        self.env["ROS_PACKAGE_PATH"] = ":".join(
            str(p) for p in self.package_resolver.search_paths
        )
    
    def expand(self, xacro_path: str) -> str:
        """Expand a .xacro file to plain URDF XML."""
        if not Path(xacro_path).exists():
            raise FileNotFoundError(f"XACRO file not found: {xacro_path}")
        
        with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as tmp:
            output_path = tmp.name
        
        try:
            cmd = ["xacro", xacro_path, "-o", output_path]
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                env=self.env,
                timeout=30
            )
            
            if result.returncode != 0:
                raise RuntimeError(
                    f"xacro expansion failed (return code {result.returncode}):\n"
                    f"STDERR: {result.stderr}"
                )
            
            with open(output_path, "r", encoding="utf-8") as f:
                urdf_xml = f.read()
            
            return urdf_xml
        
        finally:
            Path(output_path).unlink()