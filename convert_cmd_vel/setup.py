from setuptools import find_packages, setup

package_name = 'convert_cmd_vel'

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
    maintainer='chihan',
    maintainer_email='looichihan@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = convert_cmd_vel.teleop_keyboard:main',
            'output_holo_tow = convert_cmd_vel.output_holo_tow:main',
        ],
    },
)
