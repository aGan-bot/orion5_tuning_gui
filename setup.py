from setuptools import setup

package_name = 'orion5_tuning_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/orion5_tuning_gui.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='as',
    maintainer_email='as@example.com',
    description='GUI tuning tool for Orion5 gravity compensation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orion5_tuning_gui = orion5_tuning_gui.tuning_gui:main',
        ],
    },
)
