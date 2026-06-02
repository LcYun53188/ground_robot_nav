from setuptools import find_packages, setup

package_name = 'nav_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Safety monitor and emergency state publishing',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'odom_jump_guard = nav_safety.odom_jump_guard:main',
        'safety_monitor = nav_safety.safety_monitor:main',
    ]},
)
