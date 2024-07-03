from setuptools import find_packages, setup

package_name = 'safety_robile'

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
    maintainer='jay',
    maintainer_email='jp52999@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_robile = safety_robile.safety_robile:main',
            'safety_robile_BT = safety_robile.safety_robile_BT:main',
            'safety_robile_SMACH = safety_robile.safety_robile_SMACH:main', 
        
        ],
    },
)
