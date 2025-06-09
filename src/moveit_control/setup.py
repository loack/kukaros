from setuptools import find_packages, setup

package_name = 'moveit_control'

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
    maintainer='lolo',
    maintainer_email='loic.tournier@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'add_boxes = moveit_control.add_boxes:main',
            'plan_trajectory = moveit_control.plan_trajectory:main',
        ],
    },
)
