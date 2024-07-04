from setuptools import setup
import os
from glob import glob

package_name = 'py_launch_husky'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index/resource_index/packages'),
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Launch files for py_pubsub package',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
