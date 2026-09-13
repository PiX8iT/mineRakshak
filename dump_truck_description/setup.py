from setuptools import setup
import os
from glob import glob

package_name = 'dump_truck_description'

def get_data_files(source_dir):
    data_files = []
    for root, _, files in os.walk(source_dir):
        if files:
            dest_dir = os.path.join('share', package_name, root)
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((dest_dir, file_paths))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ] + get_data_files('models') + get_data_files('meshes'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)