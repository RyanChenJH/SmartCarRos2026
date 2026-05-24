from setuptools import setup

package_name = 'nav2_waypoint_cycle'

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
    maintainer='noah',
    maintainer_email='noah@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_waypoint_cycle = nav2_waypoint_cycle.waypoint_cycle:main'
        ],
    },
)
