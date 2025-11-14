from setuptools import setup

package_name = 'teleop_arm_twist_keyboard'

setup(
    name=package_name,
    version='2.4.1',
    packages=[],
    py_modules=[
        'teleop_arm_twist_keyboard'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@openrobotics.org',
    author='Graylin Trevor Jay, Austin Hendrix',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A robot-agnostic teleoperation node to convert keyboard'
                'commands to Twist messages.',
    license='BSD',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'teleop_arm_twist_keyboard = teleop_arm_twist_keyboard:main'
        ],
    },
)
