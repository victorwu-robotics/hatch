"""
Xacro Expander - Expands xacro macros and evaluates xacro expressions.

Handles the xacro-specific features that real-world URDF packages use:
- xacro:property with dependency tracking
- xacro:macro definition and instantiation
- xacro:include with $(find ...) resolution
- xacro:if / xacro:unless conditional blocks
- xacro:insert_block for macro parameter blocks
- ${...} math and string expressions

Pure Python. Cross-platform. No ROS dependencies.
"""

import math
import re
import logging
from pathlib import Path
from typing import Dict, List, Optional, Set, Any
import xml.etree.ElementTree as ET

from .package_resolver import PackageResolver

logger = logging.getLogger(__name__)


class XacroExpander:
    """
    Expands xacro files into plain URDF XML.
    
    Handles the full set of xacro features used by real-world packages
    from ROS-Industrial and manufacturers (Intel RealSense, FAIRINO, etc.)
    """

    XACRO_NS = "xacro"

    def __init__(self, package_resolver: Optional[PackageResolver] = None):
        """
        Initialize the xacro expander.

        Args:
            package_resolver: PackageResolver for $(find ...) resolution.
                             If None, creates a default one.
        """
        self.package_resolver = package_resolver or PackageResolver()
        self._properties: Dict[str, str] = {}
        self._macros: Dict[str, ET.Element] = {}
        self._evaluated_properties: Set[str] = set()
        self._macro_block_stacks: Dict[str, List[List[ET.Element]]] = {}
        self._eval_namespace: Dict[str, Any] = {}

        self._init_builtins()

    def _init_builtins(self):
        """Initialize built-in math constants and functions."""
        builtins = {
            'pi': math.pi,
            'PI': math.pi,
            'M_PI': math.pi,
            'deg2rad': math.pi / 180.0,
            'rad2deg': 180.0 / math.pi,
            'abs': abs,
            'min': min,
            'max': max,
            'round': round,
            'int': int,
            'float': float,
            'str': str,
            'len': len,
            'sin': math.sin,
            'cos': math.cos,
            'tan': math.tan,
            'asin': math.asin,
            'acos': math.acos,
            'atan': math.atan,
            'atan2': math.atan2,
            'sqrt': math.sqrt,
            'radians': math.radians,
            'degrees': math.degrees,
        }
        self._eval_namespace.update(builtins)
        for name, value in builtins.items():
            self._properties[name] = str(value)
            self._evaluated_properties.add(name)

    # =================================================================
    # Public API
    # =================================================================

    def expand(self, filepath: str) -> str:
        """Expand a xacro file into plain URDF XML."""
        filepath = Path(filepath).expanduser().resolve()

        if not filepath.exists():
            raise FileNotFoundError(f"File not found: {filepath}")

        logger.info(f"Expanding: {filepath}")

        tree = ET.parse(str(filepath))
        root = tree.getroot()

        self._macros.clear()
        self._properties.clear()
        self._evaluated_properties.clear()
        self._macro_block_stacks.clear()
        self._init_builtins()

        # PASS 1: Collect all property definitions (raw strings)
        self._collect_properties(root, filepath.parent)

        # PASS 2: Evaluate properties in dependency order
        self._evaluate_all_properties()

        # PASS 3: Process elements with all properties available
        self._process_element(root, filepath.parent)

        # PASS 4: Final sweep — substitute any remaining ${...} in all attributes
        self._final_substitution_sweep(root)

        self._strip_xacro_namespace(root)
        return self._element_to_string(root)


    def _final_substitution_sweep(self, element: ET.Element):
        """Final pass: substitute any remaining ${...} in all attributes."""
        for attr_name, attr_value in element.attrib.items():
            if attr_name == '_parent':
                continue
            if '$' in attr_value:
                new_value = self._substitute_string(attr_value)
                if new_value != attr_value:
                    element.set(attr_name, new_value)

        for child in element:
            self._final_substitution_sweep(child)

    # =================================================================
    # Property Collection and Evaluation
    # =================================================================

    def _collect_properties(self, element: ET.Element, current_dir: Path):
        """Pass 1: Collect all xacro:property definitions as raw strings."""
        tag = self._tag_name(element)

        if tag == f"{self.XACRO_NS}:property":
            name = element.get('name')
            value = element.get('value', '')
            if name and name not in self._properties:
                self._properties[name] = value
                logger.debug(f"Property collected (raw): {name} = {value}")

        if tag == f"{self.XACRO_NS}:include":
            filename = element.get('filename', '')
            if filename:
                filename = self._resolve_find(filename)
                resolved = self._resolve_include_path(filename, current_dir)
                if resolved and resolved.exists():
                    included_tree = ET.parse(str(resolved))
                    self._collect_properties(included_tree.getroot(), resolved.parent)

        for child in element:
            self._collect_properties(child, current_dir)

    def _evaluate_all_properties(self):
        """Pass 2: Evaluate properties in dependency order."""
        max_iterations = len(self._properties) + 1

        for _ in range(max_iterations):
            made_progress = False

            for name, value in list(self._properties.items()):
                if name in self._evaluated_properties:
                    continue

                if '$' not in value:
                    self._evaluated_properties.add(name)
                    made_progress = True
                else:
                    evaluated = self._substitute_string(value)
                    if '$' not in evaluated:
                        self._properties[name] = evaluated
                        self._evaluated_properties.add(name)
                        made_progress = True
                        logger.debug(f"Property evaluated: {name} = {evaluated}")

            if not made_progress:
                unevaluated = [n for n in self._properties if n not in self._evaluated_properties]
                if unevaluated:
                    logger.warning(f"Could not evaluate: {unevaluated}")
                break

    # =================================================================
    # Element Processing
    # =================================================================

    def _process_element(self, element: ET.Element, current_dir: Path):
        """Pass 3: Process an element and its children."""
        tag = self._tag_name(element)

        # Handle conditionals first — they control whether children are processed
        if tag == f"{self.XACRO_NS}:if":
            self._handle_conditional(element, current_dir, condition=True)
            return

        if tag == f"{self.XACRO_NS}:unless":
            self._handle_conditional(element, current_dir, condition=False)
            return

        # Handle other xacro directives
        if tag == f"{self.XACRO_NS}:include":
            self._handle_include(element, current_dir)
            return

        if tag == f"{self.XACRO_NS}:property":
            self._handle_property(element)
            return

        if tag == f"{self.XACRO_NS}:macro":
            self._handle_macro_definition(element)
            return

        if tag == f"{self.XACRO_NS}:insert_block":
            self._handle_insert_block(element)
            return

        # Check if this is a macro call
        macro = self._macros.get(tag)
        if macro is not None:
            self._handle_macro_call(element, macro, current_dir)
            return

        # Regular element: substitute attributes and process children
        self._substitute_attributes(element)

        for child in list(element):
            child.set('_parent', element)
            self._process_element(child, current_dir)

    # =================================================================
    # Conditional Blocks
    # =================================================================

    def _handle_conditional(self, element: ET.Element, current_dir: Path, condition: bool):
        """Handle <xacro:if value="..."> and <xacro:unless value="...">"""
        value_str = element.get('value', 'false')
        evaluated = self._evaluate_condition(value_str)

        # For xacro:if, include when true. For xacro:unless, include when false.
        should_include = evaluated if condition else not evaluated

        parent = self._get_parent(element)
        if parent is None:
            return

        try:
            idx = list(parent).index(element)
        except ValueError:
            idx = len(list(parent))

        if should_include:
            children = list(element)
            for i, child in enumerate(children):
                child_copy = self._deep_copy_element(child)
                parent.insert(idx + i, child_copy)
                child_copy.set('_parent', parent)
                self._process_element(child_copy, current_dir)

        parent.remove(element)

    def _evaluate_condition(self, value_str: str) -> bool:
        """Evaluate a condition string to boolean."""
        evaluated = self._substitute_string(value_str)

        if evaluated.lower() in ('true', '1'):
            return True
        if evaluated.lower() in ('false', '0'):
            return False

        try:
            return float(evaluated) != 0
        except (ValueError, TypeError):
            pass

        return False

    # =================================================================
    # Include Handling
    # =================================================================

    def _handle_include(self, element: ET.Element, current_dir: Path):
        """Handle <xacro:include filename="..."/>"""
        filename = element.get('filename', '')
        if not filename:
            return

        filename = self._substitute_string(filename)
        filename = self._resolve_find(filename)
        resolved = self._resolve_include_path(filename, current_dir)

        if resolved is None or not resolved.exists():
            logger.warning(f"Include not found: {filename}")
            return

        logger.debug(f"Including: {resolved}")

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
                for i, child in enumerate(list(included_root)):
                    parent.insert(idx + i, child)
            else:
                parent.insert(idx, included_root)

            parent.remove(element)

    def _resolve_find(self, text: str) -> str:
        """Resolve $(find package_name) to absolute path."""
        pattern = re.compile(r'\$\(find\s+([^)]+)\)')

        def replace_find(match):
            package_name = match.group(1).strip()
            package_dir = self.package_resolver.find_package(package_name)
            if package_dir is not None:
                return str(package_dir)
            logger.warning(f"Could not resolve $(find {package_name})")
            return match.group(0)

        return pattern.sub(replace_find, text)

    def _resolve_include_path(self, filename: str, current_dir: Path) -> Optional[Path]:
        """Resolve include file path."""
        if filename.startswith('file://'):
            file_path = filename[7:]
            path = Path(file_path)
            if path.is_absolute():
                return path if path.exists() else None
            return (current_dir / file_path).resolve()

        path = Path(filename)
        if path.is_absolute():
            return path if path.exists() else None

        return (current_dir / filename).resolve()

    # =================================================================
    # Property Handling
    # =================================================================

    def _handle_property(self, element: ET.Element):
        """Handle <xacro:property name="x" value="y"/>"""
        name = element.get('name')
        if name:
            self._properties[name] = self._properties.get(name, element.get('value', ''))
            self._evaluated_properties.add(name)

        parent = self._get_parent(element)
        if parent is not None:
            parent.remove(element)

    # =================================================================
    # Macro Definition
    # =================================================================

    def _handle_macro_definition(self, element: ET.Element):
        """Handle <xacro:macro name="m" params="a b">"""
        name = element.get('name')
        if not name:
            return

        macro_key = f"{self.XACRO_NS}:{name}"
        self._macros[macro_key] = element
        logger.debug(f"Macro defined: {name}")

        parent = self._get_parent(element)
        if parent is not None:
            parent.remove(element)

    # =================================================================
    # Macro Call
    # =================================================================

    def _handle_macro_call(self, element: ET.Element, macro_def: ET.Element, current_dir: Path):
        """Handle macro instantiation with insert_block support."""
        params_str = macro_def.get('params', '')
        param_names = [p.strip() for p in params_str.split() if p.strip()]

        # Collect parameter values
        param_values: Dict[str, str] = {}
        for pname in param_names:
            if ':=' in pname:
                base_name, default_val = pname.split(':=', 1)
                param_values[base_name] = element.get(base_name, default_val)
            else:
                param_values[pname] = element.get(pname, '')

        # Collect block parameters (for insert_block)
        block_params: Dict[str, List[ET.Element]] = {}
        for child in list(element):
            child_tag = self._tag_name(child)
            if not child_tag.startswith(f"{self.XACRO_NS}:"):
                block_name = child.tag
                if block_name not in block_params:
                    block_params[block_name] = []
                block_params[block_name].append(child)

        # Push block parameters onto stacks
        for block_name, elements in block_params.items():
            if block_name not in self._macro_block_stacks:
                self._macro_block_stacks[block_name] = []
            self._macro_block_stacks[block_name].append(elements)

        # Save and apply macro-local properties
        old_properties = {}
        macro_properties = self._collect_macro_properties(macro_def, param_values)

        for name, value in macro_properties.items():
            old_properties[name] = self._properties.get(name)
            self._properties[name] = value
            self._evaluated_properties.add(name)

        # Process macro children
        parent = self._get_parent(element)
        if parent is not None:
            try:
                idx = list(parent).index(element)
            except ValueError:
                idx = len(list(parent))

            new_children = []
            for child in list(macro_def):
                tag = self._tag_name(child)
                if tag == f"{self.XACRO_NS}:property":
                    continue
                child_copy = self._deep_copy_element(child)
                self._substitute_macro_params(child_copy, param_values)
                new_children.append(child_copy)

            for i, child in enumerate(new_children):
                parent.insert(idx + i, child)
                child.set('_parent', parent)
                self._process_element(child, current_dir)

            parent.remove(element)

        # Restore old properties
        for name, old_value in old_properties.items():
            if old_value is None:
                self._properties.pop(name, None)
                self._evaluated_properties.discard(name)
            else:
                self._properties[name] = old_value
                self._evaluated_properties.add(name)

        # Pop block parameter stacks
        for block_name in block_params:
            if block_name in self._macro_block_stacks:
                self._macro_block_stacks[block_name].pop()
                if not self._macro_block_stacks[block_name]:
                    del self._macro_block_stacks[block_name]

    def _collect_macro_properties(self, macro_def: ET.Element,
                                  param_values: Dict[str, str]) -> Dict[str, str]:
        """Collect property definitions from a macro body."""
        properties = {}
        for child in list(macro_def):
            tag = self._tag_name(child)
            if tag == f"{self.XACRO_NS}:property":
                name = child.get('name')
                value = child.get('value', '')
                if name:
                    value = self._substitute_macro_params_in_string(value, param_values)
                    properties[name] = value
        return properties

    # =================================================================
    # Insert Block
    # =================================================================

    def _handle_insert_block(self, element: ET.Element):
        """Handle <xacro:insert_block name="block_name"/>"""
        block_name = element.get('name', '')
        if not block_name:
            return

        block_stack = self._macro_block_stacks.get(block_name, [])
        if not block_stack:
            logger.warning(f"No block content for '{block_name}'")
            return

        block_content = block_stack[-1]

        parent = self._get_parent(element)
        if parent is None:
            return

        try:
            idx = list(parent).index(element)
        except ValueError:
            idx = len(list(parent))

        for i, block_element in enumerate(block_content):
            block_copy = self._deep_copy_element(block_element)
            # ← ADD THIS: substitute attributes in the block
            self._substitute_attributes(block_copy)
            parent.insert(idx + i, block_copy)
            block_copy.set('_parent', parent)

        parent.remove(element)

    # =================================================================
    # String and Attribute Substitution
    # =================================================================

    def _substitute_attributes(self, element: ET.Element):
        """Substitute ${...} in all attributes."""
        for attr_name, attr_value in element.attrib.items():
            if attr_name == '_parent':
                continue
            new_value = self._substitute_string(attr_value)
            if new_value != attr_value:
                element.set(attr_name, new_value)

    def _substitute_macro_params(self, element: ET.Element, param_values: Dict[str, str]):
        """Substitute macro parameters in an element's attributes."""
        for attr_name, attr_value in element.attrib.items():
            if attr_name == '_parent':
                continue
            new_value = self._substitute_macro_params_in_string(attr_value, param_values)
            if new_value != attr_value:
                element.set(attr_name, new_value)

        for child in element:
            self._substitute_macro_params(child, param_values)

    def _substitute_macro_params_in_string(self, text: str,
                                           param_values: Dict[str, str]) -> str:
        """Replace ${param_name} with macro parameter values."""
        for pname, pvalue in param_values.items():
            text = text.replace(f"${{{pname}}}", pvalue or '')
        return text

    def _substitute_string(self, text: str) -> str:
        """Substitute ${...} expressions including math and string ops."""
        if '$' not in text:
            return text

        pattern = re.compile(r'\$\{([^}]+)\}')

        def replace_match(match):
            expr = match.group(1).strip()

            # Build namespace from evaluated properties
            namespace = dict(self._eval_namespace)
            for k, v in self._properties.items():
                if k in self._evaluated_properties:
                    try:
                        namespace[k] = float(v)
                    except (ValueError, TypeError):
                        namespace[k] = v

            try:
                result = eval(expr, {"__builtins__": {}}, namespace)
                if isinstance(result, str):
                    return result
                if isinstance(result, float):
                    if result == int(result):
                        return str(int(result))
                    return str(result)
                return str(result)
            except Exception:
                pass

            # Fallback: direct property lookup
            if expr in self._properties:
                return str(self._properties[expr])

            # If we get here, we couldn't evaluate — log it once
            logger.warning(f"Could not evaluate expression: ${{{expr}}}")
            return match.group(0)

        result = pattern.sub(replace_match, text)
        return result

    # =================================================================
    # XML Helpers
    # =================================================================

    def _tag_name(self, element: ET.Element) -> str:
        """Get full tag name including xacro namespace."""
        tag = element.tag
        if '}' in tag:
            ns_url, local_name = tag.split('}', 1)
            if 'xacro' in ns_url.lower():
                return f"xacro:{local_name}"
        return tag

    def _strip_xacro_namespace(self, element: ET.Element):
        """Remove xacro namespace elements from the tree."""
        tag = element.tag
        if 'xacro' in tag.lower():
            parent = self._get_parent(element)
            if parent is not None:
                parent.remove(element)
            return

        for child in list(element):
            self._strip_xacro_namespace(child)

    def _get_parent(self, element: ET.Element) -> Optional[ET.Element]:
        """Get parent element via _parent attribute."""
        return element.get('_parent')

    def _deep_copy_element(self, element: ET.Element) -> ET.Element:
        """Create a deep copy of an XML element."""
        xml_str = ET.tostring(element, encoding='unicode')
        return ET.fromstring(xml_str)

    @staticmethod
    def _element_to_string(element: ET.Element) -> str:
        """Convert an XML element to a pretty-printed string."""
        import xml.dom.minidom as minidom

        for el in element.iter():
            if '_parent' in el.attrib:
                del el.attrib['_parent']

        rough_string = ET.tostring(element, encoding='unicode')
        reparsed = minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ")