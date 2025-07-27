from setuptools import find_packages, setup

package_name = 'imu_gps_sync_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_localization.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gps_display.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf_global.yaml']),
        ('share/' + package_name + '/config', ['config/navsat_transform.yaml']),
        ('share/' + package_name + '/config', ['config/ekf_local.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='songsong',
    maintainer_email='songsong@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_repub = imu_gps_sync_py.gps_repub:main',
            'gps_imu_sync = imu_gps_sync_py.gps_imu_sync:main',
            'gps_path_pub = imu_gps_sync_py.gps_path_pub:main',
            'gps_to_point = imu_gps_sync_py.gps_to_point:main',
            'gps_to_web = imu_gps_sync_py.gps_to_web:main',
            'gps_with_grid = imu_gps_sync_py.gps_with_grid:main',
            'gps_to_csv = imu_gps_sync_py.gps_to_csv:main',
            'imu_to_csv = imu_gps_sync_py.imu_to_csv:main',
            'imu_repub = imu_gps_sync_py.imu_repub:main',
        ],
    },
)
