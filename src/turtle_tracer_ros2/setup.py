from setuptools import setup, find_packages
package_name = 'turtle_tracer_ros2'
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Turtlesim spiral with PNG logo mask',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'draw_logo_masked_spiral = turtle_tracer_ros2.draw_logo_masked_spiral:main',
        ],
    },
)
