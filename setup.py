from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'aama_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name + '/urdf'), glob('urdf/*.xacro', recursive=True)),
        (os.path.join('share', package_name + '/worlds'), glob('worlds/*.world'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dogukan',
    maintainer_email='dogukan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'my_node_py = my_package_py.my_node_py:main'
        ],
    },
)
