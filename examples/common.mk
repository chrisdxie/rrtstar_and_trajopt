#-----------------------------------------------------------------------------
# PATHS  -- the user may need to modify the following paths appropriately.
#-----------------------------------------------------------------------------

# 1. Usual lib directory
LIBDIR = /usr/local/lib

# 2. Directory where the smp trunk/ is located
SMP_ROOT_PATH = $(shell pwd)/../../../../

#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Boost and Python
#-----------------------------------------------------------------------------

# Boost include directory
BOOST_INCLUDE_PATH = /usr/local/include/

# Python include directory
PYTHON_INCLUDE_PATH = /Library/Frameworks/Python.framework/Versions/2.7/include/python2.7

# Boost libraries
BL1 = /usr/local/lib/libboost_iostreams-mt.dylib
BL2 = /usr/local/lib/libboost_python.dylib
BL3 = /usr/local/lib/libboost_filesystem-mt.dylib
BL4 = /usr/local/lib/libboost_system-mt.dylib
BL5 = /usr/local/lib/libboost_numpy.dylib

# Python libraries
PYTHON_LIBRARIES = /Library/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib

# Boost and Python flags
CXXFLAGS_BOOST_PYTHON := -I$(BOOST_INCLUDE_PATH) -I$(PYTHON_INCLUDE_PATH) -L/usr/local/lib  -L/Library/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib  /usr/local/lib/libboost_iostreams-mt.dylib /usr/local/lib/libboost_python.dylib /usr/local/lib/libboost_thread-mt.dylib /usr/local/lib/libboost_filesystem-mt.dylib /usr/local/lib/libboost_system-mt.dylib /usr/local/lib/libboost_numpy.dylib /Library/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib -Wl,-rpath,/usr/local/lib -Wl,-rpath,/Library/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib 


#-----------------------------------------------------------------------------
# Standard
#-----------------------------------------------------------------------------

# Compiler and linker
CC := gcc
CXX := g++
LDXX := g++


# Standard flags
CXXFLAGS_STD := -g \
	-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE \
	-Wall -Wno-unused-parameter -Wno-sign-compare -D__STDC_FORMAT_MACROS 

LDFLAGS_STD = -lm -L $(LIBDIR)




#-----------------------------------------------------------------------------
# SMP
#-----------------------------------------------------------------------------

# SMP paths
SMP_SRC_PATH  = $(SMP_ROOT_PATH)/src
SMP_BIN_PATH  = $(SMP_ROOT_PATH)/bin
SMP_LIB_PATH  = $(SMP_ROOT_PATH)/lib

# SMP flags
CXXFLAGS_SMP := -I$(SMP_SRC_PATH)
LDFALGS_SMP := 




#-----------------------------------------------------------------------------
# Flags
#-----------------------------------------------------------------------------

CXXFLAGS = $(CXXFLAGS_STD) $(CXXFLAGS_SMP) $(CXXFLAGS_BOOST_PYTHON)

LDFLAGS	= $(LDFLAGS_STD) $(LDFLAGS_SMP) $(CXXFLAGS_BOOST_PYTHON)



#-----------------------------------------------------------------------------
# Objects
#-----------------------------------------------------------------------------

%.o: %.c
	$(CC) -o $@ -c $(CXXFLAGS) $<

%.o: %.C
	$(CXX) -o $@ -c $(CXXFLAGS) $<

%.o: %.cpp %.h
	$(CXX) -o $@ -c $(CXXFLAGS) $<
