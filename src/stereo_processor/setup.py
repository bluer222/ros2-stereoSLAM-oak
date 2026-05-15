from setuptools import find_packages, setup

package_name = 'stereo_processor'

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
    maintainer='root',
    maintainer_email='57820015+bluer222@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'claude = stereo_processor.claude:main',
            'raft = stereo_processor.raft:main',
            'gemini = stereo_processor.gemini:main',
            'stereo_processor = stereo_processor.stereo_processor:main'
        ],
    },
)
