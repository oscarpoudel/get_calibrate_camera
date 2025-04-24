from setuptools import find_packages, setup

package_name = 'get_calibrate_camera'

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
    maintainer='oscar',
    maintainer_email='opoudel27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usbcam_node = get_camera.usbcam_node:main',
            'ip_stream_node = get_camera.ip_stream_node:main', 
            'calibrate_camera=get_camera.calibrate_camera:main'  
        ],
    },
)
