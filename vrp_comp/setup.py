from setuptools import setup

package_name = 'vrp_comp'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'test'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='bogdanov_am@spbstu.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual = vrp_comp.manual_control:main',
            'test = vrp_comp.do_task:main',
            'first_task = vrp_comp.first_task:main',
            'second_task = vrp_comp.second_task:main',
            'third_task = vrp_comp.third_task:main',
            'lidar_show = vrp_comp.lidar_show:main',
            'formula0 = vrp_comp.formula0:main',
            'avior = vrp_comp.avior:main',
            'tAdA = vrp_comp.tAdA:main',
            'vorsha = vrp_comp.vorsha:main',
            'seaphoenix = vrp_comp.seaphoenix:main',
            'motors_test = test.motors_test:main',
            'led_show = vrp_comp.led_show:main' 
        ],
    },
)
