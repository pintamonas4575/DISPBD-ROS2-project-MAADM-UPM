from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tortuga_loca'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='robotica@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a_nodo_tortuga       = tortuga_loca.a_nodo_tortuga:main',
            'b_nodo_tortuga_reset = tortuga_loca.b_nodo_tortuga_reset:main',
            'c_nodo_tortuga_giros = tortuga_loca.c_nodo_tortuga_giros:main',
            'd_nodo_tortuga_clean = tortuga_loca.d_nodo_tortuga_clean:main',
            'e_nodo_tortuga_speed = tortuga_loca.e_nodo_tortuga_speed:main',
            'f_nodo_tortuga_log   = tortuga_loca.f_nodo_tortuga_log:main',
        ],
    },

)
