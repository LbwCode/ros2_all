from setuptools import find_packages, setup

package_name = 'ros2_action_python'

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
    maintainer='lbw',
    maintainer_email='lbw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_ce1 = ros2_action_python.action_move_client:main',
            'action_ce2 = ros2_action_python.action_move_server:main',
        ],
    },
)
