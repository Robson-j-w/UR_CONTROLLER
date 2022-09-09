from setuptools import setup

package_name = 'grasp_pose_pubsub'

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
    maintainer='kevin',
    maintainer_email='k.j.obuya@sheffield.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_pose_listener = grasp_pose_pubsub.grasp_pose_sub:main',
            'frame_transformer = grasp_pose_pubsub.frame_transformer:main',
        ],
    },
)
