from setuptools import setup
from setuptools import find_packages
import os

package_name = 'mockbot'
share_path = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (share_path, ['package.xml']),
        (os.path.join(share_path, 'img'), [os.path.join('img', "turtle1.jpg")]),
        (os.path.join(share_path, 'img'), [os.path.join('img', "turtle2.jpg")]),
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [os.path.join('resource', package_name)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mockbot = mockbot.mockbot:main'
        ],
    },
)
