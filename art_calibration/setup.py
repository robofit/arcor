from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['art_calibration'],
    package_dir={
        'art_calibration': 'src/art_calibration',
                })

setup(**setup_args)

