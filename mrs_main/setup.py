from setuptools import find_packages, setup

package_name = 'mrs_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'networkx'
        ],
    zip_safe=True,
    maintainer='Mateusz Zembron',
    maintainer_email='mateuszzembron@gmail.com',
    description='Framework for task allocation and coordination in distributed system',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robo_agent = mrs_main.agent_entrypoint:main',
        ],
    },
)