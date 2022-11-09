import os
from glob import glob
from setuptools import setup
import xacro

package_name = 'tolya'


def generate_data_files():
    data_files = []
    data_dirs = ['launch', 'params', 'urdf']

    for dir in data_dirs:
        for file_path in glob(dir + '/**', recursive=True):
            file = os.path.split(file_path)
            if os.path.isfile(file_path):
                data_files.append((os.path.join('share', package_name, file[0]), [file_path]))

    return data_files


def build_urdf():
    robot_file_name = 'tolya.xacro'

    xacro_file = os.path.join('urdf', robot_file_name)   
    assert os.path.exists(xacro_file), "{robot_file_name} doesnt exist in {xacro_file}"

    robot_description_config = xacro.process_file(xacro_file)
    
    with open(os.path.join('urdf', 'tolya.urdf'), 'w') as f:
        f.write(robot_description_config.toxml())

build_urdf()

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + generate_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pizda',
    maintainer_email='mighty.igor@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_detection = tolya.cube_detection:main',
            'movement_controller = tolya.movement_controller:main'
        ],
    },
)
