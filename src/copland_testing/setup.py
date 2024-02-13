from setuptools import find_packages, setup

package_name = 'copland_testing'

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
    maintainer='James Hardy',
    maintainer_email='cyanjamesmail@gmail.com',
    description='Testing Scheduler "Copland" for CMP3753 Project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'copland_main = copland_testing.copland_main:main',
            'copland_metrics = copland_testing.copland_metrics:main',
            'copland_environment = copland_testing.copland_environment:main'
        ],
    },
)
