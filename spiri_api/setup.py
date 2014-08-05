from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from distutils.extension import Extension

d = generate_distutils_setup()
d['packages'] = ['spiri_api']

d['scripts'] = ['bin/spiri_create_pkg']
d['package_dir'] = {'': 'src'}


setup(**d)
