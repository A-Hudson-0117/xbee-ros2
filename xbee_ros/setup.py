from setuptools import find_packages, setup

package_name = 'xbee_ros'

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
    maintainer='Adam',
    maintainer_email='91583630+terminator0117@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbee_node = xbee_ros.xbee_node:main', 
            'xbee_encryption = xbee_ros.xbee_encryption:main',
        ],
    },
)
