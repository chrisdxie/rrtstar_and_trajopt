/*! \file components/extenders/double_integrator_trajopt.h
  \brief The double integrator system components using TrajOpt. State, input, and extender definitions.
  
  This file implements the state, input, and extender classes for a 
  d-dimensional double integrator system, where d is a template parameter
  when appropriate.
*/

#ifndef _SMP_SYSTEM_DOUBLE_INTEGRATOR_TRAJOPT_ACADO_H_
#define _SMP_SYSTEM_DOUBLE_INTEGRATOR_TRAJOPT_ACADO_H_

#include <smp/components/extenders/double_integrator.h>

#include <eigen3/Eigen/Eigen>

#include <list>

using namespace std;

namespace smp {

    //! Extender function with double integrator dynamics using TrajOpt.
    /*!
      This class implements an extender with double integrator dynamics. It is intended
      that the number of dimensions of the state space is a template argument for the class.
      However, this feature is not implemented yet. 
      
      \ingroup extenders
    */

    template < class typeparams, int NUM_DIMENSIONS >
    class extender_double_integrator_trajopt : public extender_double_integrator <typeparams, NUM_DIMENSIONS> {

        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;

        typedef trajectory< typeparams > trajectory_t;

        int run_ACADO_optimization(state_t *start_state, state_t *end_state, 
                                   list<state_t*> *list_states, list<input_t*> *list_inputs);

    public:

        int extend (state_t *state_from_in, state_t *state_towards_in,
                    int *exact_connection_out, trajectory_t *trajectory_out,
                    list<state_t*> *intermediate_vertices_out);
    
    };


}

#endif
