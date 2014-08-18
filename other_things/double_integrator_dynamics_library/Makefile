# Usual lib directory
LIBDIR = /usr/local/lib

#-----------------------------------------------------------------------------
# Standard
#-----------------------------------------------------------------------------

# Compiler and linker
CC := gcc
CXX := g++
LDXX := llvm-g++

#-----------------------------------------------------------------------------
# Boost
#-----------------------------------------------------------------------------

# Boost include directory
BOOST_INCLUDE_PATH = /usr/local/include/

# Boost flags
CXXFLAGS_BOOST := -I$(BOOST_INCLUDE_PATH)

# Python include directory
PYTHON_INCLUDE_PATH = /Library/Frameworks/Python.framework/Versions/2.7/include/python2.7

# Boost and Python flags
CXXFLAGS_BOOST_PYTHON := -I$(BOOST_INCLUDE_PATH) -I$(PYTHON_INCLUDE_PATH) -L/usr/local/lib  -L/Library/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib  /usr/local/lib/libboost_iostreams-mt.dylib /usr/local/lib/libboost_python.dylib /usr/local/lib/libboost_thread-mt.dylib /usr/local/lib/libboost_filesystem-mt.dylib /usr/local/lib/libboost_system-mt.dylib /usr/local/lib/libboost_numpy.dylib /Library/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib -Wl,-rpath,/usr/local/lib -Wl,-rpath,/Library/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib 


#-----------------------------------------------------------------------------
# Flags
#-----------------------------------------------------------------------------

FLAGS_I_NEED_FOR_SOME_REASON = 

LDFLAGS_STD = -g -lm -L $(LIBDIR)

LDFLAGS	= $(LDFLAGS_STD) $(CXXFLAGS_BOOST_PYTHON) $(FLAGS_I_NEED_FOR_SOME_REASON)

CFLAGS = -Wall -c 

#-----------------------------------------------------------------------------
# Make the program
#-----------------------------------------------------------------------------

default: 
	$(MAKE) test	

test: test.cpp double_integrator_dynamics.h double_integrator_dynamics.hpp
	$(LDXX) -o test $(LDFLAGS) test.cpp

#-----------------------------------------------------------------------------
# Cleaning
#-----------------------------------------------------------------------------

clean:
	-find ./ -name \*.o -exec rm {} \;
	-find ./ -name \*.gch -exec rm {} \;
	-find ./ -name test -exec rm {} \;