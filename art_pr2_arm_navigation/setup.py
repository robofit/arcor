from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['art_pr2_arm_navigation'],
    package_dir={
        'art_pr2_arm_navigation': 'src/art_pr2_arm_navigation',
        })

setup(**setup_args)
