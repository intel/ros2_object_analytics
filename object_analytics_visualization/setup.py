from setuptools import setup

package_name = 'object_analytics_visualization'

setup(
    name=package_name,
    version='0.3.0',
    packages=['object_analytics_visualization'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', ['launch/object_analytics_rviz.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Chris Ye',
    author_email='chris.ye@intel.com',
    maintainer='Chris Ye',
    maintainer_email='chris.ye@intel.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Visualizing for object analytics results',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = object_analytics_visualization.image_publisher:main',
            'marker_publisher = object_analytics_visualization.marker_publisher:main',
        ],
    },
)
