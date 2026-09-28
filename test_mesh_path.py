import os
import sys

sys.path.insert(0, os.path.expanduser('~/hatch'))

os.environ['HATCH_PACKAGE_PATH'] = os.path.expanduser('~/hatch/assets')

from core.urdf_preprocessor import URDFPreprocessor
from utils.package_resolver import PackageResolver

resolver = PackageResolver()
preprocessor = URDFPreprocessor(resolver)

urdf_xml = preprocessor.process(os.path.expanduser('~/hatch/assets/scenes/my_arm.urdf.xacro'))

import re
mesh_refs = re.findall(r'filename="([^"]+)"', urdf_xml)
print(f"\nMesh references in URDF:")
for ref in mesh_refs:
    print(f"  {ref}")