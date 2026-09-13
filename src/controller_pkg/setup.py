from setuptools import find_packages, setup

package_name = 'controller_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amr Al-Qaffas',
    maintainer_email='amr.alqaffas@gmail.com',
    description='Manual-arm control chain: PS4 input, cylindrical IK, joint controller. Talks to ESP32 over UART.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_input_node        = controller_pkg.arm_input_node:main',
            'cyl_ik_node           = controller_pkg.cyl_ik_node:main',
            'serial_bridge_node    = controller_pkg.serial_bridge_node:main',
        ],
    },
)
