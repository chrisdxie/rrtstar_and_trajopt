#ifndef _SMP_SYSTEM_DOUBLE_INTEGRATOR_TRAJOPT_NOCD_FORCES_HPP_
#define _SMP_SYSTEM_DOUBLE_INTEGRATOR_TRAJOPT_NOCD_FORCES_HPP_

#include <smp/components/extenders/double_integrator_trajopt_FORCES.h>
#include "../../../../other_things/double_integrator_FORCES/SQP/without_CD/double_integrator_noCD_sqp.hpp"

#include <iostream>
#include <cstdlib>

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

using namespace std;

#define VELOCITY_CONSTRAINT_MAX 1.0
#define VELOCITY_CONSTRAINT_MIN -1.0

#define INPUT_CONSTRAINT_MAX 1.0
#define INPUT_CONSTRAINT_MIN -1.0

#define STATE_CONSTRAINT_MAX 10.0
#define STATE_CONSTRAINT_MIN -10.0

template <class typeparams, int NUM_DIMENSIONS>
int smp::extender_double_integrator_trajopt<typeparams,NUM_DIMENSIONS>
::run_FORCES_optimization (state_t *state_from_in, state_t *state_towards_in, 
                          list<state_t*> *list_states, list<input_t*> *list_inputs) {

  // Clear list of states and inputs
  list_states->clear();
  list_inputs->clear();

  // Process the states into Vectors for passing into functions
  Vector4d x_start, x_goal;
  x_start << (*state_from_in)[0],
             (*state_from_in)[1],
             (*state_from_in)[2],
             (*state_from_in)[3];

  x_goal << (*state_towards_in)[0],
            (*state_towards_in)[1],
            (*state_towards_in)[2],
            (*state_towards_in)[3];

  // Instantiate StdVectorsX and StdVectorU
  StdVectorX X(T);
  StdVectorU U(T-1);

  // Init bounds
  bounds_t bounds;

  bounds.u_max = INPUT_CONSTRAINT_MAX;
  bounds.u_min = INPUT_CONSTRAINT_MIN;
  bounds.x_max = STATE_CONSTRAINT_MAX;
  bounds.x_min = STATE_CONSTRAINT_MIN;
  bounds.v_max = VELOCITY_CONSTRAINT_MAX;
  bounds.v_min = VELOCITY_CONSTRAINT_MIN;
  bounds.x_goal = x_goal;
  bounds.x_start = x_start;

  // Initialize pointer to time variable, delta
  double* delta;

  // Call SQP
  int success = solve_double_integrator_noCD_BVP(x_start, x_goal, X, U, delta, bounds);

  if (success == 0) {
    return 0;
  }

  for (int i = 0; i < T-1; i++) {

    // Take care of state first. x_1, ..., x_N. EXCLUDES x_0!
    state_t *state_new = new state_t;
    for (int j = 0; j < 4; j++) {
      (*state_new)[j] = X[i+1](j);
    }
    list_states->push_back(state_new);

    // Take care of inputs next. u_0, ..., u_{N-1} EXCLUDES u_N!
    input_t *input_new = new input_t;
    (*input_new)[0] = *delta;
    for (int j = 1; j < 3; j++) {
      (*input_new)[j] = U[i](j);
    }
    list_inputs->push_back(input_new);
  }

  //---------------------------------------------------------------------
  /* DONE!! */

  return 1;
}



template <class typeparams, int NUM_DIMENSIONS>
int smp::extender_double_integrator_trajopt<typeparams,NUM_DIMENSIONS>
::extend (state_t *state_from_in, state_t *state_towards_in,
	  int *exact_connection_out, trajectory_t *trajectory_out,
	  list<state_t*> *intermediate_vertices_out) {
  
  // cout << "state_from";
  // for (int i = 0; i < 4; i++)
  //   cout << " : " << state_from_in->state_vars[i];
  // cout << endl;

  // cout << "state_towa";
  // for (int i = 0; i < 4; i++)
  //   cout << " : " << state_towards_in->state_vars[i];
  // cout << endl;

  // Looks like this does nothing for now
  intermediate_vertices_out->clear ();

  // 2D ACADO stuffs
  if (run_FORCES_optimization(state_from_in, state_towards_in,
          &(trajectory_out->list_states), &(trajectory_out->list_inputs)) != 1) {
    return 0;
  }

  *exact_connection_out = 1; // Always an exact connection
  
  return 1;
}


#endif


