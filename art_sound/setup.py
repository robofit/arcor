from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['art_sound'],
    package_dir={
        'art_sound': 'src/art_sound',
    })

setup(**setup_args)
