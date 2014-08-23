include ../../../common.mk


#-----------------------------------------------------------------------------
# Program
#-----------------------------------------------------------------------------

SMP = $(SMP_BIN_PATH)/example_standalone_rrtstar_double_integrator_trajopt_collision_detection

SMP_OBJ_COMPILE = standalone_rrtstar_double_integrator_trajopt_collision_detection.o \
	$(SMP_SRC_PATH)/smp/external_libraries/kdtree/kdtree.o \
	double_integrator_QP_solver_CD.c.o -rdynamic -lboost_system-mt -lboost_program_options-mt -ldl -lrt

SMP_OBJ = standalone_rrtstar_double_integrator_trajopt_collision_detection.o \
	$(SMP_SRC_PATH)/smp/external_libraries/kdtree/kdtree.o 




#-----------------------------------------------------------------------------
# Make the program
#-----------------------------------------------------------------------------

all: 
	$(MAKE) targets

default: 
	$(MAKE) targets	

targets:  $(SMP) 


$(SMP): $(SMP_OBJ) 
	$(LDXX) $(SMP_OBJ_COMPILE) -o $(SMP) $(LDFLAGS) 




#-----------------------------------------------------------------------------
# Cleaning
#-----------------------------------------------------------------------------

clean:
	-find ./ -name \*.o -exec rm {} \;
	-find ./ -name \*.gch -exec rm {} \;
