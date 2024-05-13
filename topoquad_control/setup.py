from setuptools import find_packages, setup

package_name = 'topoquad_control'

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
    maintainer='Keisuke Nagashima',
    maintainer_email='nagashima@fuzzrobo.com',
    description='topo quad control package',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'neck_joy = topoquad_control.neck_joy_sample:main',
        ],
    },
)
