from setuptools import find_packages, setup

package_name = 'prova_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'transforms3d', 'numpy', 'rclpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             "listener_nodo = prova_controller.listener_messages:main",
             "EKF = prova_controller.EKF:main",
             "EKF_t = prova_controller.EKF_tmp:main",
             "ascolto_nodo = prova_controller.prova_listener:main",
             "gpt_nodo = prova_controller.gpt_node:main",
             "position_debug_node = prova_controller.debugger_py:main",
             "offboard_controller_node = prova_controller.offboard_controller:main"
             
        ],
    },
)
