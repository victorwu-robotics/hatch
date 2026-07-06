#!/usr/bin/env python3
import subprocess
import sys

# Read requirements
with open('requirements.txt', 'r') as f:
    requirements = [line.strip() for line in f if line.strip()]

# Check each package
pypi_packages = []
ros_packages = []

for req in requirements:
    # Extract package name (before ==, >=, etc.)
    pkg_name = req.split('==')[0].split('>=')[0].split('<=')[0].split('>')[0].split('<')[0].strip()
    
    # Try to search PyPI for this package
    try:
        result = subprocess.run(
            ['pip', 'search', pkg_name],
            capture_output=True,
            text=True,
            timeout=5
        )
        if 'No matching distribution found' in result.stdout:
            ros_packages.append(req)
        else:
            pypi_packages.append(req)
    except:
        # If pip search fails, check if it's a ROS package by name
        ros_keywords = ['msgs', 'ros', 'rcl', 'action', 'geometry', 'sensor', 'tf2', 'urdf', 'robot']
        if any(keyword in pkg_name.lower() for keyword in ros_keywords):
            ros_packages.append(req)
        else:
            pypi_packages.append(req)

# Write outputs
with open('requirements_pypi.txt', 'w') as f:
    f.write('\n'.join(pypi_packages))

with open('requirements_ros.txt', 'w') as f:
    f.write('\n'.join(ros_packages))

print(f"✅ PyPI packages: {len(pypi_packages)}")
print(f"📦 ROS packages: {len(ros_packages)}")
print("")
print("PyPI packages:", pypi_packages)
print("ROS packages:", ros_packages)
