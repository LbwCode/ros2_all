from setuptools import find_packages, setup

package_name = 'ros2_img_python'

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
            'lbw_img_ce1 = ros2_img_python.img_ce1:main',
            'lbw_img_ce2 = ros2_img_python.img_ce2:main',
        ],
    },
)
