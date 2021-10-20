from setuptools import setup

package_name = 'sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nmvr',
    maintainer_email='nmvr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_builder=sim.tki:main',
            'csv_read=sim.csv_reader:main',
            'csv_write=sim.csv_writer:main',
            'grid_visualiser=sim.grid_visualiser:main',
        ],
    },
)
