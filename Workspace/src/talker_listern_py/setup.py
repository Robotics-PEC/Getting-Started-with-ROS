from setuptools import find_packages, setup

package_name = 'talker_listern_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jazzer',
    maintainer_email='jazzer@todo.todo',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = talker_listern_py.talker:main',
            'listener = talker_listern_py.listener:main',
        ],
    },
)
