from setuptools import setup

package_name = 'hewo_face_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'hewo-face>=1.0.0',
        'psutil>=7.0.0'
    ],
    zip_safe=True,
    maintainer='Diego Delgado Chaves',
    maintainer_email='diedelcha@gmail.com',
    description='ROS 2 wrapper for launching HeWo face using the hewo-face Python package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hewo_main_node = hewo_face_pkg.main_node:main',
            'hewo_test_node = hewo_face_pkg.node_functionality:main'
        ],
    },
)
