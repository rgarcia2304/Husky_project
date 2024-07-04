from setuptools import find_packages, setup

package_name = 'husky_sensors_monitoring'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rgarcia',
    maintainer_email='rgarcia22141@gmail.com',
    description='Sample',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = scripts.digitalStateDisplay:main',
                'experiment= scripts.huskyStatePublisher:main',
                'hardware= scripts.ledStackController:main',
        ],
    },
)
