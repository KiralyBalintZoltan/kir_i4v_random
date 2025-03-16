from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'kir_i4v_random'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['kir_i4v_random', 'kir_i4v_random.*']),  # Pontosabb csomagkeresés
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KiralyBalintZoltan',
    maintainer_email='kiralybalintzoltan@gmail.com',
    description='Ez egy bevezető csomag',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw.random = kir_i4v_random/draw.random:main',
        ],
    },
)

