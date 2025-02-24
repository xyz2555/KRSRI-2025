from setuptools import find_packages, setup

package_name = 'sambergeni_controller'

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
    maintainer='sambergeni',
    maintainer_email='sambergeni@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = sambergeni_controller.testing:main",
            "test_node2 = sambergeni_controller.testing2:main",
            "test_node3 = sambergeni_controller.testing3:main",
            "test_node4 = sambergeni_controller.testing4:main",
        ],
    },
)
