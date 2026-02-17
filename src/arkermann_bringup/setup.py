from setuptools import setup
import os
from glob import glob

package_name = 'arkermann_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name,
            ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # Install world file
        (os.path.join('share', package_name, 'gazebo'),
            ['gazebo/maze.world']),

        # Install maze model files
        (os.path.join('share', package_name, 'gazebo/maze_model'),
            [
                'gazebo/maze_model/model.config',
                'gazebo/maze_model/model.sdf'
            ]),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kanishq',
    maintainer_email='kanishq@todo.todo',
    description='Bringup package for arkermann robot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

