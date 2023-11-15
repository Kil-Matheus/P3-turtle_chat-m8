import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'meu_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))    
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kil',
    maintainer_email='kil.teixeira@sou.inteli.edu.br',
    description='Pacote Caminho ROS2, primeiro comando de navegação salva a posição atual do robô e as demais posições são relativas a primeira.',
    license='CC0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "nav2_test = meu_ros.nav2_test:main",
            "nav_waypoints = meu_ros.nav_waypoints:main",
            "chatbot = meu_ros.test_chatbot:main",
        ],
    },
)
