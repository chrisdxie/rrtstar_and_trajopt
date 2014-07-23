.SILENT:

all:
	echo
	echo [SMP] NOTHING TO BE COMPILED IN THIS FOLDER.
	echo
	echo [SMP] To compile the examples go to the trunk/examples/platform folder,
	echo       where platform stands for the choice of your robot operating 
	echo       system platform, and type make to compile all examples for that 
	echo       platform. The binaries will appear in the trunk/bin folder. 
	echo       Currently available platforms include libbot. ros and openrave
	echo       bindings are work in progress. standalone examples support 
	echo       Unix-based systems, e.g., Ubuntu and Mac OS X, and work also on
	echo       Windows platforms.
	echo

clean:
	echo [SMP] Removing all object files.
	-find ./ -name \*.o -exec rm {} \;
	echo [SMP] Removing all compiled header files.
	-find ./ -name \*.gch -exec rm {} \;
	echo [SMP] Cleaning the bin directory.
	-rm -f ./bin/*

clean_tmp:
	echo [SMP] Cleaning temporary files
	-find ./ -name \*~ -exec rm {} \;
	-find ./ -name \*# -exec rm {} \;