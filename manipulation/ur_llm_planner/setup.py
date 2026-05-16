from setuptools import setup

package_name = 'ur_llm_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darsh',
    maintainer_email='darshmenon02@gmail.com',
    description='Motion executor for UR arm via MoveIt Pilz PTP and ros2_control',
    license='Apache-2.0',
    entry_points={},
)
