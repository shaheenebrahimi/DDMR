from setuptools import find_packages, setup
import glob

package_name = 'project_4'

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
    maintainer='lgtom',
    maintainer_email='lgtomotaki@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_translator_node = project_4.velocity_translator_node:main',
            'simulator_node = project_4.simulator_node:main',
            'navigation_controller_node = project_4.navigation_controller_node:main'
        ],
    },
)
