from setuptools import find_packages, setup

package_name = 'navigation_pkg'

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
    maintainer='varun',
    maintainer_email='varun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "path_planning = navigation_pkg.path_planning:main",
            "obstacle_avoidance = navigation_pkg.obstacle_avoidance:main"
        ],
    },
)
