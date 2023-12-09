from setuptools import find_packages, setup
import glob

package_name = 'ddmr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sebrahimi',
    maintainer_email='shaheen.m.ebrahimi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_translator_node = ddmr.velocity_translator_node:main',
            'simulator_node = ddmr.simulator_node:main',
            'navigation_controller_node = ddmr.navigation_controller_node:main'
        ],
    },
)
