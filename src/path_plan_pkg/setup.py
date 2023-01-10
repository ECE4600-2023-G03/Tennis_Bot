from setuptools import setup

package_name = 'path_plan_pkg'

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
    maintainer='Blane Cypurda',
    maintainer_email='bcypurda286@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'greedy_ball_picker = path_plan_pkg.greedy_ball_picker:main',
            'greedy_ball_picker_pub_test = path_plan_pkg.greedy_ball_picker_pub_test:main'

        ],
    },
)
