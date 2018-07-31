from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['art_projected_gui',  'art_projected_gui.gui',  'art_projected_gui.items', 'art_projected_gui.helpers',
              'art_projected_gui.plugins'],
    package_dir={
        'art_projected_gui': 'src/art_projected_gui',
        'art_projected_gui.gui': 'src/art_projected_gui/gui',
        'art_projected_gui.items': 'src/art_projected_gui/items',
        'art_projected_gui.helpers': 'src/art_projected_gui/helpers',
        'art_projected_gui.plugins': 'src/art_projected_gui/plugins'
        })

setup(**setup_args)
