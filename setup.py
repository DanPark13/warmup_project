from setuptools import setup

package_name = 'warmup_project'

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
    maintainer='deep13',
    maintainer_email='dpark0703@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = warmup_project.teleop:main',
            'marker_sample = warmup_project.marker_sample:main',
            'drive_square = warmup_project.drive_square:main',
            'wall_follower = warmup_project.wall_follower:main',
            'wall_marker = warmup_project.wall_marker:main',
            'person_follower = warmup_project.person_follower:main',
            'range_plotter = warmup_project.range_plotter:main',
            'test_person_follower = warmup_project.test_person_follower:main',
            'position_plotter = warmup_project.position_plotter:main',
            'finite_state_control = warmup_project.finite_state_control:main'
        ],
    },
)
