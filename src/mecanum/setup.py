from setuptools import setup

package_name = 'mecanum'

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
    maintainer='ian',
    maintainer_email='ian@sodersjerna.com',
    description='package for mecanum robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'main = mecanum.main:main',
                'backtalk = mecanum.backtalk:main',
                'mecanum = mecanum.mecanum:main',
                'camera = mecanum.camera:main',
                'lidar = mecanum.lidar:main',
        ],
    },
)
