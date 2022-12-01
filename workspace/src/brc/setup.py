import os
from glob import glob
from setuptools import setup

package_name = 'brc'

def generate_data_files():
    data_files = []
    data_dirs = ['launch', 'worlds', 'models', 'configs', 'maps', 'params']

    for dir in data_dirs:
        for file_path in glob(dir + '/**', recursive=True):
            file = os.path.split(file_path)
            if os.path.isfile(file_path):
                data_files.append((os.path.join('share', package_name, file[0]), [file_path]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml'])
    ] + generate_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pizda',
    maintainer_email='pizda@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_task = brc.main_task:main'
        ],
    },
)
