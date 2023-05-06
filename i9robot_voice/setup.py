from setuptools import setup

package_name = 'i9robot_voice'

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
    maintainer='logan',
    maintainer_email='naidoo.logan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "voice_asr = i9robot_voice.voice_asr:main",
            "voice_tts = i9robot_voice.voice_tts:main",
            "voice_cmd = i9robot_voice.voice_cmd:main",
            "voice_mov = i9robot_voice.voice_mov:main"
        ],
    },
)
