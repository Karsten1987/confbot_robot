from setuptools import find_packages
from setuptools import setup

package_name = 'confbot_tools'

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
    author='Karsten Knese',
    author_email='karsten.knese@googlemail.com',
    maintainer='Karsten Knese',
    maintainer_email='karsten.knese@googlemail.com',
    keywords=['confbot', 'ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safe_zone_publisher = confbot_tools.safe_zone_publisher:main',
        ],
    },
)
