import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'r1d1_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name), glob('launch/*.py')),
        # Include model and simulation files and add them in urdf dir
        (os.path.join('share', package_name, "urdf"), glob('urdf/*')),
        # Include stl files and add them in meshes dir
        (os.path.join('share', package_name, "meshes"), glob('meshes/*')),
        (os.path.join('share', package_name, "config"), glob('config/*')),
        (os.path.join('share', package_name, "world"), glob('world/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rahul mishra',
    maintainer_email='mishrarahul1997@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
