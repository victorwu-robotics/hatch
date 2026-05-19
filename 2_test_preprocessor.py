import sys
sys.path.insert(0, '/home/victor/hatch')
from core.urdf_preprocessor import URDFPreprocessor

preprocessor = URDFPreprocessor(
    package_dirs=[
        "/home/victor/hatch/assets/robots",
        "/home/victor/hatch/assets/sensors",
        "/home/victor/hatch/assets/tools",
    ]
)

# Test Keyence file directly
result = preprocessor.process("/home/victor/hatch/assets/sensors/keyence_experimental/urdf/lj_v7200_macro.xacro")
print(result[:500])