from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'Sensors-and-Control-Integrated-Project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

         # ─── ADD THIS LINE TO TRACK YOUR LAUNCH FILES ───
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        
        # ─── ADD THIS LINE IF YOU HAVE A MODELS FOLDER FOR YOUR SDF ───
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),

        # adds scripts to the package
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'parameters'), glob('parameters/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liamd',
    maintainer_email='liam.dascalos@student.uts.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
