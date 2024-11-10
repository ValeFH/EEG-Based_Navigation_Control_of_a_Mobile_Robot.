from setuptools import find_packages, setup

package_name = 'ras_topics'

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
    maintainer='majocabra',
    maintainer_email='majocabrae@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # nombre del ejecutable (como quiero que se llame)
            # ubicacion exacta del paquete
            # nombre del archivo de python 
            # la funcion (main) 
            'robot_EEG_demo = ras_topics.RobotEEGDemo:main',
            'robot_EEG = ras_topics.RobotEEG:main',
            'escuchar = ras_topics.subscriber:main',            
        ],
    },
)
