from setuptools import find_packages, setup

package_name = 'fusion_system_display'

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
    maintainer='spencer',
    maintainer_email='20426598+roshambo919@users.noreply.github.com',
    description='TODO: Package description',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "do_nothing = fusion_system_display.do_nothing:main"
        ],
    },
)
