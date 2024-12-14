from setuptools import find_packages, setup

package_name = 'susumu_gtts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'gtts'],
    zip_safe=True,
    maintainer='sato-susumu',
    maintainer_email='75652942+sato-susumu@users.noreply.github.com',
    description='A ROS 2 package for text-to-speech using GTTS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'susumu_gtts = susumu_gtts.susumu_gtts:main',
        ],
    },
)
