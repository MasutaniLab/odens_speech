from setuptools import find_packages, setup

package_name = 'odens_speech'

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
    maintainer='masutani',
    maintainer_email='masutani@isc.osakac.ac.jp',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_recognition = odens_speech.speech_recognition:main',
            'speech_recognition_server = odens_speech.speech_recognition_server:main',
            'speech_recognition_client = odens_speech.speech_recognition_client:main',
            'speech_synthesis = odens_speech.speech_synthesis:main',
            'speech_synthesis_server = odens_speech.speech_synthesis_server:main',
            'speech_synthesis_client = odens_speech.speech_synthesis_client:main',
            'speech_client = odens_speech.speech_client:main',
            'tmc_talk_hoya_server = odens_speech.tmc_talk_hoya_server:main',
            'tmc_talk_hoya_client = odens_speech.tmc_talk_hoya_client:main'
        ],
    },
)
