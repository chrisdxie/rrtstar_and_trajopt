#ifndef _SMP_SYSTEM_DOUBLE_INTEGRATOR_TRAJOPT_HPP_
#define _SMP_SYSTEM_DOUBLE_INTEGRATOR_TRAJOPT_HPP_

#include <smp/components/extenders/double_integrator_trajopt.h>

#define VELOCITY_CONSTRAINT_RANGE 2.0
#define VELOCITY_CONSTRAINT_RANGE_2 1.0
#define VELOCITY_CONSTRAINT_MAX 1.0
#define VELOCITY_CONSTRAINT_MIN -1.0
#define VELOCITY_CONSTRAINT_SQ 1.0
//
#define INPUT_CONSTRAINT_MAX 1.0
#define INPUT_CONSTRAINT_MIN -1.0
//
// This is an upper bound on the time of the individual controls. We optimize over this times the number of states..
// so I guess we can not really worry about this for now. Connect everything exactly by minimizing time.
#define DELTA_T 0.5 
// ACADO stuff
#define D_SAFE 0.05
#define NUM_DISCRETIZATION 20

#include <acado_toolkit.hpp>
#include <eigen3/Eigen/Eigen>
//#include <include/acado_gnuplot/gnuplot_window.hpp>

#include "../../../../other_things/2d_signed_distance_library_cpp/signedDistancePolygons.hpp"

#include <iostream>
#include <cstdlib>

using namespace std;

// collision detection function
void signedDistanceFunction( double *z, double *f, void *user_data ) {

    double x = z[0];
    double y = z[1];

    Matrix<double, 2, 2> point_robot;
    point_robot << x, y, x, y;

    // Hard coded obstacle
    Matrix<double, 4, 2> obs;
    obs << 1, 2,
           1, 1,
           3, 1,
           3, 2;

    Matrix<double, 3, 2> ans = signed_distance_2d::signedDistancePolygons(point_robot, obs);
    f[0] = ans(0,0);

}


template <class typeparams, int NUM_DIMENSIONS>
int smp::extender_double_integrator_trajopt<typeparams,NUM_DIMENSIONS>
::run_ACADO_optimization (state_t *state_from_in, state_t *state_towards_in, 
                          list<state_t*> *list_states, list<input_t*> *list_inputs) {

  // Clear list of states and inputs
  list_states->clear();
  list_inputs->clear();

  // Process the states into double arrays for passing into functions
  double start_x = (*state_from_in)[0];
  double start_y = (*state_from_in)[1];
  double start_xdot = (*state_from_in)[2];
  double start_ydot = (*state_from_in)[3];

  double end_x = (*state_towards_in)[0];
  double end_y = (*state_towards_in)[1];
  double end_xdot = (*state_towards_in)[2];
  double end_ydot = (*state_towards_in)[3];

  //---------------------------------------------------------------------

  USING_NAMESPACE_ACADO

  // Construct ACADO stuff

  DifferentialState x, y, xdot, ydot;
  Control ux, uy;
  Parameter T;
  DifferentialEquation f( 0.0, T );

  // Clear stuff for this run
  x.clearStaticCounters();
  y.clearStaticCounters();
  xdot.clearStaticCounters();
  ydot.clearStaticCounters();
  ux.clearStaticCounters();
  uy.clearStaticCounters();
  T.clearStaticCounters();

  //---------------------------------------------------------------------

  OCP ocp( 0.0, T , NUM_DISCRETIZATION); // time horizon of the optimal control problem is [0, T]
  ocp.minimizeMayerTerm( T ); // Minimize T  

  //---------------------------------------------------------------------

  f << dot(x) == xdot;
  f << dot(y) == ydot;
  f << dot(xdot) == ux;
  f << dot(ydot) == uy;

  //---------------------------------------------------------------------

  IntermediateState z(7);
  z(0) = x; z(1) = y; z(2) = xdot; z(3) = ydot; z(4) = ux; z(5) = uy; z(6) = T;

  CFunction SD( 1, signedDistanceFunction);

  //---------------------------------------------------------------------

  ocp.subjectTo( f );

  ocp.subjectTo( AT_START, x == start_x );
  ocp.subjectTo( AT_START, y == start_y );
  ocp.subjectTo( AT_START, xdot == start_xdot );
  ocp.subjectTo( AT_START, ydot == start_ydot );

  ocp.subjectTo( AT_END, x == end_x );
  ocp.subjectTo( AT_END, y == end_y ); 
  ocp.subjectTo( AT_END, xdot == end_xdot );
  ocp.subjectTo( AT_END, ydot == end_ydot );

  // Obstacles, worry about this alter
  ocp.subjectTo( SD(z) >= D_SAFE );
  //ocp.subjectTo( SWVSD(z) >= D_SAFE );

  ocp.subjectTo( INPUT_CONSTRAINT_MIN <= ux <= INPUT_CONSTRAINT_MAX );
  ocp.subjectTo( INPUT_CONSTRAINT_MIN <= uy <= INPUT_CONSTRAINT_MAX );

  ocp.subjectTo( VELOCITY_CONSTRAINT_MIN <= xdot <= VELOCITY_CONSTRAINT_MAX );
  ocp.subjectTo( VELOCITY_CONSTRAINT_MIN <= ydot <= VELOCITY_CONSTRAINT_MAX );
    
  ocp.subjectTo( 0 <= T );  

  //---------------------------------------------------------------------

  // Initialize time to something, otherwise things get wacky
  double time_init = 20.00e+00;
  VariablesGrid T_init(1, 0, 1, NUM_DISCRETIZATION+1);
  for (int i = 0; i <= NUM_DISCRETIZATION; i++) {
      T_init(i, 0) = time_init;
  }

  // Initialize to straight line with 0 velocities
  VariablesGrid state_init(4, 0, 1, NUM_DISCRETIZATION+1);
  state_init.setZero();
  for (int i = 0; i <= NUM_DISCRETIZATION; i++) {
      state_init(i, 0) = ((float) i)/NUM_DISCRETIZATION * (end_x - start_x) + start_x;
  }
  for (int i = 0; i <= NUM_DISCRETIZATION; i++) {
      state_init(i, 1) = ((float) i)/NUM_DISCRETIZATION * (end_y - start_y) + start_y;
  }

  //---------------------------------------------------------------------

  // GnuplotWindow w2;
  // w2.addSubplot( x, y, "Position" , "", "", PM_POINTS);
  // w2.addSubplot( xdot, "Velocity x" );
  // w2.addSubplot( ydot, "Velocity y" );
  // w2.addSubplot( ux, "Control for x" );
  // w2.addSubplot( uy, "Control for y" );

  OptimizationAlgorithm algorithm( ocp );
  //algorithm << w2;
  algorithm.initializeParameters(T_init);
  algorithm.initializeDifferentialStates(state_init);
  algorithm.set(HESSIAN_APPROXIMATION, BLOCK_BFGS_UPDATE);    // default
  algorithm.set(PRINTLEVEL, NONE); // How to shush the solver
  returnValue r = algorithm.solve();

  if (r != SUCCESSFUL_RETURN) {
    return 0;
  }

  //---------------------------------------------------------------------
  // ACADO solver finished, now grab answers and put into SMP data structures.

  // Grab answers in ACADO data structures
  VariablesGrid states, parameters, controls;
  algorithm.getDifferentialStates(states);
  algorithm.getControls(controls);
  algorithm.getParameters(parameters);

  // Put answers in SMP data structures
  double t = parameters(0,0); // Should be same for all

  for (int i = 0; i < NUM_DISCRETIZATION; i++) {

    // Take care of state first. x_1, ..., x_N. EXCLUDES x_0!
    state_t *state_new = new state_t;
    for (int j = 0; j < 4; j++) {
      (*state_new)[j] = states(i+1,j);
    }
    list_states->push_back(state_new);

    // Take care of inputs next. u_0, ..., u_{N-1} EXCLUDES u_N!
    input_t *input_new = new input_t;
    (*input_new)[0] = t/NUM_DISCRETIZATION;
    for (int j = 0; j < 2; j++) {
      (*input_new)[j+1] = controls(i, j);
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
  if (run_ACADO_optimization(state_from_in, state_towards_in,
          &(trajectory_out->list_states), &(trajectory_out->list_inputs)) != 1) {
    return 0;
  }

  *exact_connection_out = 1; // Always an exact connection
  
  return 1;
}


#endif


