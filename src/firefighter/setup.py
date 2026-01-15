from setuptools import find_packages, setup

package_name = 'firefighter'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    entry_points={
        'console_scripts': [
            'firefighter = firefighter.firefighter:main',
            'movement = firefighter.movement:main',
            'movefire = firefighter.movefire:main',
            'frmvm = firefighter.frmvm:main',
            'camera = firefighter.camera:main'
            'camera2 = firefighter.camera2:main'
            'fin1 = firefighter.fin1:main'
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    description='Firefighter bot',
    license='GPLv3',
)
