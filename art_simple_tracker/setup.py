from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['art_simple_tracker'],
    package_dir={
        'art_simple_tracker': 'src/art_simple_tracker',
        })

setup(**setup_args)
