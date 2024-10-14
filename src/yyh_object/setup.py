from setuptools import find_packages, setup

package_name = 'yyh_object'

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
    maintainer='yyh',
    maintainer_email='14899946+Yyh-eagle@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_pub = yyh_object.usb_pub:main',
            'usb_pub2 = yyh_object.usb_pub2:main',
            'Image_gongchuang = yyh_object.Image_gongchuang:main',
        ],
    },
)
