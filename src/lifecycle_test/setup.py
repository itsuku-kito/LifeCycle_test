from setuptools import find_packages, setup

package_name = 'lifecycle_test'

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
    maintainer='rsdlab',
    maintainer_email='243432007@ccmailg.meijo-u.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'moveit_test1 = lifecycle_test.moveit_test1:main',
        'moveit_test2 = lifecycle_test.moveit_test2:main'
        ],
    },
)
