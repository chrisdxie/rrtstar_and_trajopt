SUBDIRS:= rrtstar/double_integrator rrtstar/double_integrator_trajopt rrtstar/double_integrator_trajopt_collision_detection

.PHONY: all clean 
.SILENT:

all: 
	echo [Make all]
	@for dir in $(SUBDIRS) ; do \
	echo $$dir; $(MAKE) -C $$dir || exit 2; done

clean:
	echo [Clean]
	@for dir in $(SUBDIRS) ; do \
	echo $$dir; $(MAKE) -C $$dir clean || exit 2; done
	rm -f *~ lib/*
