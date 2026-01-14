from setuptools import find_packages, setup

package_name = 'firefighter'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['firefighter/firefighter.py']), ('lib/' + package_name, ['firefighter/movement.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Firefighter bot',
    license='GPLv3',
)
