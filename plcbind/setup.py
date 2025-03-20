from setuptools import find_packages, setup

package_name = 'plcbind'

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
    maintainer='marrinus',
    maintainer_email='m@thelonesomeprogrammer.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bind = plcbind.bind:main',
            'stub = plcbind.plc_stub:main',
            'calc = plcbind.calc_process:main',
            "joint_state_publisher_gui = plcbind.joint_state_publisher_gui:main"
        ],
    },
)
