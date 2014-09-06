#ifndef _SMP_SAMPLER_BOOST_RANDOM_UNIFORM_HPP_
#define _SMP_SAMPLER_BOOST_RANDOM_UNIFORM_HPP_

#include <iostream>
#include <cstdlib>

#include <smp/components/samplers/boost_random_uniform.h>

#include <smp/components/samplers/base.hpp>
#include <smp/common/region.hpp>


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_uniform<typeparams,NUM_DIMENSIONS>
::sm_update_insert_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_uniform<typeparams,NUM_DIMENSIONS>
::sm_update_insert_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_uniform<typeparams,NUM_DIMENSIONS>
::sm_update_delete_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_uniform<typeparams,NUM_DIMENSIONS>
::sm_update_delete_edge (edge_t *edge_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
smp::sampler_uniform<typeparams,NUM_DIMENSIONS>
::sampler_uniform (int seed) {
  
  // Initialize the sampling distribution support.
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    support.center[i] = 0.0;
    support.size[i] = 1.0;
  }

  r_eng = new RANDOM_ENGINE(seed);
  dist = new DIST(0, 1);
  sampler = new GENERATOR(*r_eng, *dist);

}


template< class typeparams, int NUM_DIMENSIONS >
smp::sampler_uniform<typeparams,NUM_DIMENSIONS>
::~sampler_uniform () {
    

}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_uniform<typeparams,NUM_DIMENSIONS>
::sample (state_t **state_sample_out) {
  
  if (NUM_DIMENSIONS <= 0)
    return 0;

  state_t *state_new = new state_t;

  // Generate an independent random variable for each axis.
  // for (int i = 0; i < NUM_DIMENSIONS; i++) 
  //   (*state_new)[i] = support.size[i] * rand()/(RAND_MAX + 1.0) - support.size[i]/2.0 + support.center[i];

  // Generate an independent random variable for each axis.
  for (int i = 0; i < NUM_DIMENSIONS; i++) 
    (*state_new)[i] = support.size[i] * (*sampler)() - support.size[i]/2.0 + support.center[i];


  *state_sample_out = state_new;
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_uniform<typeparams,NUM_DIMENSIONS>
::set_support (region_t support_in) {
  
  support = support_in;

  return 1;
}

template< class typeparams, int NUM_DIMENSIONS >
bool smp::sampler_uniform<typeparams,NUM_DIMENSIONS>
::in_support_region (state_t *state) {
  
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    if ((*state)[i] < support.center[i] - .5*support.size[i] || 
        (*state)[i] > support.center[i] + .5*support.size[i]) {
      return false;
    }
  }
  return true;
}

#endif
