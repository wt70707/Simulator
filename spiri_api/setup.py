from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from distutils.extension import Extension

d = generate_distutils_setup()
d['packages'] = ['spiri_api']
d['package_dir'] = {'': 'src'}
d['scripts']=['bin/spiri_create_pkg']
d['ext_moudles']=[Extension("spiri_api_python",["python_bindings.cpp"],
		  libraries=["boost_python"])]
setup(**d)