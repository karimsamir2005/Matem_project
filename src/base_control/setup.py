from setuptools import find_packages, setup

package_name = 'base_control'

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
    maintainer='abdo-ros',
    maintainer_email='abdullah.mohamedsaid2005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ik=base_control.ik:main',
            'ser=base_control.serial:main',
            'speed_quantizer=base_control.speed_quantizer:main',
            'manager=base_control.mode_manager:main',
            'auto_listen=base_control.auto_activation:main',
            'auto_control=base_control.autonmous_control:main',
            'odem_send=base_control.odem_sending:main',
            'error_calc=base_control.error_calc:main',
            'pd_control=base_control.pd_control:main',
        ],
    },
)
