from setuptools import setup

package_name = 'ros2serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}/{package_name}_utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_dir={f'{package_name}_utils' : f'{package_name}/{package_name}_utils'},
    install_requires=[
        'setuptools',
        'pyserial'],
    zip_safe=True,
    maintainer='toshi',
    maintainer_email='shikito.aos@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = ros2serial.serial_interface:main'
        ],
    },
)
