from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['art_projector'],
    package_dir={
        'art_projector': 'src/art_projector'
    })

setup(**setup_args)
