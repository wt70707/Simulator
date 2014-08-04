from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['spiri_api']
d['package_dir'] = {'': 'src'}
d['scripts']=['bin/spiri_create_pkg']
setup(**d)