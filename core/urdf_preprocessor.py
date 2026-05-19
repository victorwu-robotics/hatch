import math
import re
import logging
from pathlib import Path
from typing import Dict, List, Optional
import xml.etree.ElementTree as ET

logger = logging.getLogger(__name__)


class URDFPreprocessor:
    """
    Minimal URDF preprocessor with xacro-compatible syntax.
    
    Supports the features that real-world URDF packages from
    ROS-Industrial and manufacturers actually use:
    
        $(find package_name)                       — Package path resolution
        package://package_name/relative/path       — Standard mesh URI scheme
        <xacro:include filename="..."/>            — Include another file
        <xacro:property name="x" value="y"/>       — Define a variable
        ${property_name}                           — Variable substitution
        ${pi}, ${deg2rad}, ${rad2deg}              — Built-in math constants
        <xacro:macro name="m" params="a b">        — Define a reusable block
        <xacro:m param="value"/>                   — Instantiate a macro
    
    Does NOT support:
        Python expressions in ${}
        Complex math operations
        Conditional blocks (<xacro:if>, <xacro:unless>)
        ROS-specific features beyond $(find)
        Nested macro calls beyond one level
    """

    XACRO_NS = "xacro"

    def __init__(self, package_dirs: List[str]):
        """
        Initialize the preprocessor.

        Args:
            package_dirs: List of directories to search for $(find ...) packages
                         and package:// URIs.
        """
        self.package_dirs = [Path(d).expanduser().resolve() for d in package_dirs]
        self._properties: Dict[str, str] = {}
        self._macros: Dict[str, ET.Element] = {}

        # Built-in math constants (standard xacro provides these)
        self._properties.update({
            'pi': math.pi,        # float, not string
            'PI': math.pi,
            'deg2rad': math.pi / 180.0,
            'rad2deg': 180.0 / math.pi,
            'M_PI': math.pi,
        })

    # =================================================================
    # Public API
    # =================================================================

    def process(self, filepath: str) -> str:
        """
        Process a URDF or xacro file and return plain URDF XML string.

        Args:
            filepath: Path to the .urdf or .xacro file.

        Returns:
            Plain URDF XML as a string, ready for KinematicModel to parse.
        """
        filepath = Path(filepath).expanduser().resolve()

        if not filepath.exists():
            raise FileNotFoundError(f"URDF file not found: {filepath}")

        logger.info(f"Processing: {filepath}")

        tree = ET.parse(str(filepath))
        root = tree.getroot()

        self._macros.clear()
        self._process_element(root, filepath.parent)

        self._strip_xacro_namespace(root)

        return self._element_to_string(root)

    # =================================================================
    # Element Processing
    # =================================================================

    def _process_element(self, element: ET.Element, current_dir: Path):
        """Process an XML element and its children."""
        tag = self._tag_name(element)

        if tag == f"{self.XACRO_NS}:include":
            self._handle_include(element, current_dir)
            return

        if tag == f"{self.XACRO_NS}:property":
            self._handle_property(element)
            return

        if tag == f"{self.XACRO_NS}:macro":
            self._handle_macro_definition(element)
            return

        macro = self._macros.get(tag)
        if macro is not None:
            self._handle_macro_call(element, macro)
            return

        self._substitute_attributes(element)

        # Track parent for each child
        children = list(element)
        for child in children:
            child.set('_parent', element)
            self._process_element(child, current_dir)

    def _handle_include(self, element: ET.Element, current_dir: Path):
        """Handle <xacro:include filename="..."/>"""
        filename = element.get('filename', '')
        if not filename:
            logger.warning("<xacro:include> missing filename attribute")
            return

        filename = self._substitute_string(filename)
        resolved = self._resolve_path(filename, current_dir)
        
        if resolved is None:
            logger.warning(f"Could not resolve include: {filename}")
            return
        if not resolved.exists():
            logger.warning(f"Included file not found: {resolved}")
            return

        logger.info(f"Including: {resolved}")

        included_tree = ET.parse(str(resolved))
        included_root = included_tree.getroot()

        self._process_element(included_root, resolved.parent)
        self._strip_xacro_namespace(included_root)

        parent = self._get_parent(element)
        if parent is not None:
            children = list(parent)
            try:
                idx = children.index(element)
            except ValueError:
                idx = len(children)
            
            if included_root.tag == 'robot':
                # Insert all children of the included robot
                included_children = list(included_root)
                for i, child in enumerate(included_children):
                    parent.insert(idx + i, child)
            else:
                parent.insert(idx, included_root)
            
            parent.remove(element)
        else:
            logger.error(f"No parent found for include: {filename}")

    def _handle_property(self, element: ET.Element):
        """Handle <xacro:property name="x" value="y"/>"""
        name = element.get('name')
        value = element.get('value', '')

        if name:
            self._properties[name] = value
            logger.debug(f"Property: {name} = {value}")

        parent = self._get_parent(element)
        if parent is not None:
            parent.remove(element)

    def _handle_macro_definition(self, element: ET.Element):
        """Handle <xacro:macro name="m" params="a b">"""
        name = element.get('name')
        if not name:
            logger.warning("<xacro:macro> missing name attribute")
            return

        # Store with xacro: prefix so lookup by tag works
        macro_key = f"{self.XACRO_NS}:{name}"
        self._macros[macro_key] = element
        logger.debug(f"Macro defined: {name} (key: {macro_key})")

        parent = self._get_parent(element)
        if parent is not None:
            parent.remove(element)

    def _handle_macro_call(self, element: ET.Element, macro_def: ET.Element):
        """Handle <xacro:macro_name param1="val1"/>"""
        params_str = macro_def.get('params', '')
        param_names = [p.strip() for p in params_str.split() if p.strip()]

        param_values: Dict[str, str] = {}
        for pname in param_names:
            val = element.get(pname, '')
            if not val:
                val = element.get(pname.split(':')[-1] if ':' in pname else pname, '')
            param_values[pname] = val

        macro_children = list(macro_def)
        new_children = []
        
        # First pass: collect property definitions
        temp_properties = {}
        for child in macro_children:
            tag = self._tag_name(child)
            if tag == f"{self.XACRO_NS}:property":
                name = child.get('name')
                value = child.get('value', '')
                if name:
                    # Substitute macro params in the property value
                    value = self._substitute_string(value)
                    for pname, pvalue in param_values.items():
                        value = value.replace(f"${{{pname}}}", pvalue or '')
                    temp_properties[name] = value
        
        # Temporarily add these properties
        old_properties = {}
        for name, value in temp_properties.items():
            old_properties[name] = self._properties.get(name)
            self._properties[name] = value
        
        # Second pass: copy and substitute non-property children
        for child in macro_children:
            tag = self._tag_name(child)
            if tag == f"{self.XACRO_NS}:property":
                continue  # Skip property definitions
            new_child = self._deep_copy_element(child)
            self._substitute_macro_params(new_child, param_values)
            new_children.append(new_child)
        
        # Restore old property values (or remove if they didn't exist)
        for name, old_value in old_properties.items():
            if old_value is None:
                self._properties.pop(name, None)
            else:
                self._properties[name] = old_value

        # Insert into parent
        parent = self._get_parent(element)
        if parent is not None:
            parent_children = list(parent)
            try:
                idx = parent_children.index(element)
            except ValueError:
                idx = len(parent_children)
            parent.remove(element)
            for i, child in enumerate(new_children):
                parent.insert(idx + i, child)

    # =================================================================
    # Variable Substitution
    # =================================================================

    def _substitute_attributes(self, element: ET.Element):
        """Substitute ${property} and $(find package) in all attributes."""
        for attr_name, attr_value in element.attrib.items():
            new_value = self._substitute_string(attr_value)
            if new_value != attr_value:
                element.set(attr_name, new_value)

    def _substitute_macro_params(self, element: ET.Element, param_values: Dict[str, str]):
        """Substitute macro parameters in an element."""
        for attr_name, attr_value in element.attrib.items():
            new_value = attr_value
            for pname, pvalue in param_values.items():
                new_value = new_value.replace(f"${{{pname}}}", pvalue or '')
            new_value = self._substitute_string(new_value)
            if new_value != attr_value:
                element.set(attr_name, new_value)

        for child in element:
            self._substitute_macro_params(child, param_values)

    def _substitute_string(self, text: str) -> str:
        """
        Substitute ${property_name}, $(find package_name), and simple math
        expressions like ${pi/2}, ${-pi/2}, ${pi/180.0}.
        """
        if '$' not in text and '(' not in text:
            return text

        # Handle $(find package_name)
        text = self._resolve_find_in_text(text)

        # Handle ${...} patterns
        pattern = re.compile(r'\$\{([^}]+)\}')

        def replace_match(match):
            expr = match.group(1).strip()

            # Try to evaluate as a math expression with known properties
            try:
                # Build a namespace with known properties
                namespace = {}
                namespace.update(self._properties)
                # Add common math functions
                namespace.update({
                    'abs': abs, 'min': min, 'max': max,
                    'round': round, 'int': int, 'float': float,
                })
                result = eval(expr, {"__builtins__": {}}, namespace)
                # Format result nicely
                if isinstance(result, float):
                    return str(result)
                return str(result)
            except Exception:
                pass

            # Fallback: treat as a simple property name
            if expr in self._properties:
                return self._properties[expr]

            return match.group(0)

        return pattern.sub(replace_match, text)

    def _resolve_find_in_text(self, text: str) -> str:
        """
        Replace $(find package_name) with the package's path.

        Searches package_dirs for a directory named package_name.
        If found, replaces with the absolute path.
        """
        pattern = re.compile(r'\$\(find\s+([^)]+)\)')

        def replace_find(match):
            package_name = match.group(1).strip()
            resolved = self._resolve_package_dir(package_name)
            if resolved:
                return str(resolved)
            logger.warning(f"Could not resolve $(find {package_name})")
            return match.group(0)

        return pattern.sub(replace_find, text)

    def _resolve_package_dir(self, package_name: str) -> Optional[Path]:
        """Find a package directory by name in package_dirs."""
        for pkg_dir in self.package_dirs:
            candidate = pkg_dir / package_name
            if candidate.is_dir():
                return candidate
        return None

    # =================================================================
    # Path Resolution
    # =================================================================

    def _resolve_path(self, filename: str, current_dir: Path) -> Optional[Path]:
        """Resolve a filename to an absolute path."""
        # Substitute any properties or $(find) in the filename first
        filename = self._substitute_string(filename)

        # Handle package:// URIs
        if filename.startswith('package://'):
            return self._resolve_package_uri(filename)

        # Handle file:// URIs
        if filename.startswith('file://'):
            file_path = filename[7:]
            return Path(file_path).expanduser().resolve()

        # Handle absolute paths
        path = Path(filename)
        if path.is_absolute():
            return path if path.exists() else None

        # Handle relative paths
        resolved = (current_dir / filename).resolve()
        return resolved if resolved.exists() else None

    def _resolve_package_uri(self, filename: str) -> Optional[Path]:
        """Resolve package://package_name/relative/path"""
        package_path = filename[10:]
        parts = package_path.split('/', 1)

        if len(parts) != 2:
            return None

        package_name, relative_path = parts

        # First try to find the package directory
        package_dir = self._resolve_package_dir(package_name)
        if package_dir:
            candidate = package_dir / relative_path
            if candidate.exists():
                return candidate

        # Fallback: search package_dirs directly
        for pkg_dir in self.package_dirs:
            candidate = pkg_dir / package_name / relative_path
            if candidate.exists():
                return candidate

        return None

    # =================================================================
    # XML Helpers
    # =================================================================

    def _tag_name(self, element: ET.Element) -> str:
        """Get the full tag name including namespace."""
        tag = element.tag
        if '}' in tag:
            ns_url, local_name = tag.split('}', 1)
            if 'xacro' in ns_url.lower():
                return f"xacro:{local_name}"
        return tag

    def _strip_xacro_namespace(self, element: ET.Element):
        """Remove xacro namespace from element tags."""
        tag = element.tag
        if 'xacro' in tag.lower():
            parent = self._get_parent(element)
            if parent is not None:
                parent.remove(element)
            return

        for child in list(element):
            self._strip_xacro_namespace(child)

    def _get_parent(self, element: ET.Element) -> Optional[ET.Element]:
        """Get the parent of an XML element."""
        return element.get('_parent')

    def _deep_copy_element(self, element: ET.Element) -> ET.Element:
        """Create a deep copy of an XML element."""
        xml_str = ET.tostring(element, encoding='unicode')
        return ET.fromstring(xml_str)

    @staticmethod
    def _element_to_string(element: ET.Element) -> str:
        """Convert an XML element to a pretty-printed string."""
        import xml.dom.minidom as minidom
        
        # Remove internal _parent attributes from all elements
        for el in element.iter():
            if '_parent' in el.attrib:
                del el.attrib['_parent']
        
        rough_string = ET.tostring(element, encoding='unicode')
        reparsed = minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ")