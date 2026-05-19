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

result = preprocessor.process("/home/victor/hatch/assets/scenes/my_arm.urdf.xacro")

# Show what the preprocessor produced
print("=== PROCESSED URDF ===")
print(result)
print("=== END ===")