from setuptools import setup

package_name = 'competition'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Chen',
    maintainer_email='themc@mit.edu',
    description='Package to hold the code necessary to do the competition',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_test = competition.drone_control_test:main',
            'drone_client = competition.drone_service_test:main',
        ],
    },
)
