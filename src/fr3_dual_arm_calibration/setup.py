from pathlib import Path
from setuptools import find_packages, setup

package_name = 'fr3_dual_arm_calibration'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'README.md']),
]

for directory in ('config', 'launch'):
    for path in sorted(Path(directory).rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            data_files.append(('share/' + package_name + '/' + path.parent.as_posix(), [str(path)]))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=False,
    maintainer='FR3 dual-arm maintainer',
    maintainer_email='maintainer@example.com',
    description='Camera and hand-eye calibration package',
    license='MIT',
)
