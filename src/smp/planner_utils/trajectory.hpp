#ifndef _SMP_TRAJECTORY_HPP_
#define _SMP_TRAJECTORY_HPP_

#include <smp/planner_utils/trajectory.h>

template< class typeparams >
smp::trajectory< typeparams >
::trajectory () {

}



template< class typeparams >
smp::trajectory< typeparams >
::~trajectory () {
  
  this->clear_delete ();

}


template< class typeparams >
int smp::trajectory< typeparams >
::clear () {

  // Clear the list of states and the list of inputs.
  list_states.clear();
  list_inputs.clear();

  return 1;
}


template< class typeparams >
int smp::trajectory< typeparams >
::clear_delete () {

  // Free all the memory occupied by the states in the list. 
  for (typename list<state_t*>::iterator iter_state = list_states.begin(); 
       iter_state != list_states.end(); iter_state++) {
    state_t *state_curr = *iter_state;
    delete state_curr;
  }

  // Free all the memory occupied by the inputs in the list.
  for (typename list<input_t*>::iterator iter_input = list_inputs.begin(); 
       iter_input != list_inputs.end(); iter_input++) {
    input_t *input_curr = *iter_input;
    delete input_curr;
  }

  // Clear the list of states and the list of inputs.
  this->clear();
  
  return 1;
}



template< class typeparams >
int smp::trajectory< typeparams >
::push_front (trajectory_t *trajectory_in) {

  for (typename list<state_t*>::reverse_iterator it_state = trajectory_in->list_states.rbegin(); 
       it_state != trajectory_in->list_states.rend(); it_state++) {
    
    state_t *state_curr = *it_state;
    
    this->list_states.push_front (new state_t(*state_curr));
  }


  for (typename list<input_t*>::reverse_iterator it_input = trajectory_in->list_inputs.rbegin(); 
       it_input != trajectory_in->list_inputs.rend(); it_input++) {
    
    input_t *input_curr = *it_input;
    
    this->list_inputs.push_front (new input_t(*input_curr));
  }
  
  return 1;
}



template< class typeparams >
int smp::trajectory< typeparams >
::push_back (trajectory_t *trajectory_in) {


  for (typename list<state_t*>::iterator it_state = trajectory_in->list_states.begin(); 
       it_state != trajectory_in->list_states.end(); it_state++) {
    
    state_t state_curr = *it_state;
    
    this->list_states.push_back (new state_t(*state_curr));
  }


  for (typename list<input_t*>::iterator it_input = trajectory_in->list_inputs.begin(); 
       it_input != trajectory_in->list_inputs.end(); it_input++) {
    
    input_t input_curr = *it_input;
    
    this->list_inputs.push_back (new input_t(*input_curr));
  }
  
  return 1;
}



template< class typeparams >
int smp::trajectory< typeparams >
::push_front (state_t *state_in) {
  
  this->list_states.push_front (new state_t(*state_in));

  return 1;
}



template< class typeparams >
int smp::trajectory< typeparams >
::push_back (state_t *state_in) {
  
  this->list_states.push_back (new state_t(*state_in));

  return 1;
}



template< class typeparams >
int smp::trajectory< typeparams >
::push_front (input_t *input_in) {
  
  this->list_inputs.push_front (new input_t(*input_in));

  return 1;
}



template< class typeparams >
int smp::trajectory< typeparams >
::push_back (input_t *input_in) {
  
  this->list_inputs.push_back (new state_t(*input_in));

  return 1;
}






#endif
