from distutils.core import setup, Extension
from Cython.Build import cythonize
import os

os.environ['CFLAGS'] = '-Wall -lpthread -lstdc++ -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -DIKFAST_HAS_LIBRARY'
setup(ext_modules = cythonize(Extension(
           "PyController",                                # the extesion name
           sources=["CythonController.pyx", "ikfast.cpp", "Controller.cpp"], # the Cython source and
                                                  # additional C++ source files        
           extra_link_args=['-llapack'],   
           language="c++",                        # generate and compile C++ code
      )))