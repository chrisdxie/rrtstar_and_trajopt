#ifndef _SMP_INFORMEDRRTSTAR_HPP_
#define _SMP_INFORMEDRRTSTAR_HPP_


#include <smp/planners/informedrrtstar.h>


#include <smp/planners/base_incremental.hpp>
#include <smp/planners/planner_parameters.hpp>

#include <smp/components/cost_evaluators/base.hpp>

#include <math.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>
using namespace Eigen;



template< class typeparams >
smp::informedrrtstar<typeparams>
::informedrrtstar () {
  
  cost_evaluator = NULL;
}


template< class typeparams >
smp::informedrrtstar<typeparams>
::~informedrrtstar () {
  
}



template< class typeparams >
smp::informedrrtstar<typeparams>
::informedrrtstar (sampler_t &sampler_in, distance_evaluator_t &distance_evaluator_in, extender_t &extender_in, 
	   collision_checker_t &collision_checker_in, model_checker_t &model_checker_in, cost_evaluator_t &cost_evaluator_in) :
  planner_incremental<typeparams>(sampler_in, distance_evaluator_in, extender_in, collision_checker_in, model_checker_in),
  cost_evaluator(cost_evaluator_in) {
  
  
  
}


template< class typeparams >
int smp::informedrrtstar<typeparams>
::initialize (state_t *initial_state_in) {

  planner_incremental_t::initialize(initial_state_in);
  
  this->root_vertex->data.total_cost = 0;

  r_eng = new RANDOM_ENGINE();	
  
  // normal dist
  g_dist = new G_DIST(0, 1);
  g_sampler = new G_GENERATOR(*r_eng, *g_dist);

  // uniform dist
  u_dist = new U_DIST(0, 1);
  u_sampler = new U_GENERATOR(*r_eng, *u_dist);

  // Initialize best goal to NULL, and best cost to be really large
  best_goal = new state_t;
  best_cost = 1e10;

  // Initialize Eigen matricies
  L.resize(parameters.get_dimension(), parameters.get_dimension());
  C.resize(parameters.get_dimension(), parameters.get_dimension());
  L.setZero();
  C.setZero();

  // Start with RRT
  parameters.set_phase(0);
  
  return 1;
}

template< class typeparams >
int smp::informedrrtstar<typeparams>
::init_cost_evaluator (cost_evaluator_t &cost_evaluator_in) {
  
  cost_evaluator = cost_evaluator_in;

  return 1;
}


template< class typeparams >
int smp::informedrrtstar<typeparams>
::propagate_cost (vertex_t *vertex_in, double total_cost_new) {
  
  // Update the cost of this vertex
  vertex_in->data.total_cost = total_cost_new;
  
  cost_evaluator.ce_update_vertex_cost (vertex_in);
  
  // Recursively propagate the cost along the edges
  for (typename list<edge_t*>::iterator iter_edge = vertex_in->outgoing_edges.begin(); 
       iter_edge != vertex_in->outgoing_edges.end(); iter_edge++) {


    edge_t *edge_curr = *iter_edge;
    
    vertex_t *vertex_next = edge_curr->vertex_dst;

    if (vertex_next != vertex_in)
      this->propagate_cost (vertex_next, vertex_in->data.total_cost + edge_curr->data.edge_cost);
  }
  
  return 1;
}


template< class typeparams >
int smp::informedrrtstar<typeparams>
::iteration () {
  
  // TODO: Check whether the informedrrtstar is initialized properly (including its base classes)
  
  // 1. Sample a new state from the obstacle-free space
  state_t *state_sample;

  // INFORMED RRT* STUFF
  if (this->model_checker.get_best_cost() >= 0) { // This shows that a solution has been found. Normally it's -1 until we find a solution
  	// do while loop. same until it's in bounds

	// Fill in the x_center vector
	VectorXd x_start(parameters.get_dimension()), x_goal(parameters.get_dimension());
	for(int i = 0; i < parameters.get_dimension(); ++i) {
		x_start(i) = this->root_vertex->state->state_vars[i];
		x_goal(i) = this->best_goal->state_vars[i];
	}
	VectorXd x_center = (x_start + x_goal)/2.0;

	state_sample = new state_t;

  	do {
  		// Sample
  		VectorXd x_ball = sample_from_unit_ball();
  		VectorXd x_sample = C*L*x_ball + x_center;

  		// Fill in state_sample variable
  		for(int i = 0; i < parameters.get_dimension(); ++i) {
  			state_sample->state_vars[i] = x_sample(i);
  		}

  	} while (!this->sampler.in_support_region(state_sample));

  } else {
  	this->sampler.sample (&state_sample);
  }

  if (this->collision_checker.check_collision_state (state_sample) == 0) {
    delete state_sample;
    return 0; 
  }
  
  
  // 2. Find the nearest vertex
  vertex_t *vertex_nearest;
  this->distance_evaluator.find_nearest_vertex (state_sample, (void **)&vertex_nearest);

  
  // 3. Extend the nearest vertex towards the sample
  
  double radius;
  if (parameters.get_fixed_radius() < 0.0) {
    double num_vertices = (double)(this->get_num_vertices());
    radius = parameters.get_gamma() * pow (log(num_vertices)/num_vertices,  1.0 /( (double)(parameters.get_dimension()) )  );

    // if (this->get_num_vertices()%1000 == 0) 
    //   cout << "radius " << radius << endl; 

    if (radius > parameters.get_max_radius())
      radius = parameters.get_max_radius();
  }
  else 
    radius = parameters.get_fixed_radius();

  radius_last = radius;

  int exact_connection = -1;
  trajectory_t *trajectory = new trajectory_t;
  list<state_t*> *intermediate_vertices = new list<state_t*>;
  if (this->extender.extend (vertex_nearest->state, state_sample,
			     &exact_connection, trajectory, intermediate_vertices) == 1) {  // If the extension is successful
    
    
    // 4. Check the new trajectory for collision
    if (check_extended_trajectory_for_collision (vertex_nearest->state, trajectory) == 1) {  // If the trajectory is collision free
      
      // 5. Find the parent state
      vertex_t *vertex_parent = vertex_nearest;
      trajectory_t *trajectory_parent = trajectory;
      list<state_t*> *intermediate_vertices_parent = intermediate_vertices;
      
      double cost_trajectory_from_parent = this->cost_evaluator.evaluate_cost_trajectory (vertex_parent->state, trajectory_parent);
      double cost_parent = vertex_parent->data.total_cost + cost_trajectory_from_parent;

      
      // Define the new variables that are used in both phase 1 and 2.
      list<void*> list_vertices_in_ball;
      state_t *state_extended = NULL;
      
      if (parameters.get_phase() >= 1) {  // Check whether phase 1 should occur.
      
	state_extended = new state_t(*(trajectory_parent->list_states.back())); // Create a copy of the final state
      
	// Compute the set of all nodes that reside in a ball of a certain radius centered at the extended state
	this->distance_evaluator.find_near_vertices_r (state_extended, radius, &list_vertices_in_ball);

	for (typename list<void*>::iterator iter = list_vertices_in_ball.begin(); iter != list_vertices_in_ball.end(); iter++) {
	  vertex_t *vertex_curr = (vertex_t*)(*iter);
	
	  // Skip if current vertex is the same as the nearest vertex
	  if (vertex_curr == vertex_nearest) 
	    continue;
	
	  // Attempt an extension from vertex_curr to the extended state
	  trajectory_t  *trajectory_curr = new trajectory_t;
	  list<state_t*> *intermediate_vertices_curr = new list<state_t*>;
	  exact_connection = -1;
	  if (this->extender.extend (vertex_curr->state, state_extended, 
				     &exact_connection, trajectory_curr, intermediate_vertices_curr) == 1) {

	    if ( (exact_connection == 1) && (check_extended_trajectory_for_collision(vertex_curr->state,trajectory_curr) == 1) ) {

	      // Calculate the cost to get to the extended state with the new trajectory
	      double cost_trajectory_from_curr = 
		this->cost_evaluator.evaluate_cost_trajectory (vertex_parent->state, trajectory_curr);
	      double cost_curr = vertex_curr->data.total_cost + cost_trajectory_from_curr;
	    
	      // Check whether the total cost through the new vertex is less than the parent
	      if (cost_curr < cost_parent) {
	      
		// Make new vertex the parent vertex
		vertex_parent = vertex_curr;

		trajectory_t *trajectory_tmp = trajectory_parent; // Swap trajectory_parent and trajectory_curr
		trajectory_parent = trajectory_curr;              //   to properly free the memory later
		trajectory_curr = trajectory_tmp;
	      
		list<state_t*> *intermediate_vertices_tmp = intermediate_vertices_parent;  // Swap the intermediate vertices
		intermediate_vertices_parent = intermediate_vertices_curr;                   //   to properly free the memory later
		intermediate_vertices_curr = intermediate_vertices_tmp;

		cost_trajectory_from_parent = cost_trajectory_from_curr;
		cost_parent = cost_curr;
	      }
	    }
	  }
	
	  delete trajectory_curr;
	  delete intermediate_vertices_curr;
	}
      }
      
      // Create a new vertex
      this->insert_trajectory (vertex_parent, trajectory_parent, intermediate_vertices_parent);
      
      // Update the cost of the edge and the vertex      
      vertex_t *vertex_last = this->list_vertices.back();
      vertex_last->data.total_cost = cost_parent;
      cost_evaluator.ce_update_vertex_cost (vertex_last);
      
      edge_t *edge_last = vertex_parent->outgoing_edges.back();
      edge_last->data.edge_cost = cost_trajectory_from_parent;

      // Update best goal, L and C matrices here
      if ((this->model_checker.get_best_cost() >= 0) && 	  // min_time_reachability gives -1 otherwise
      	   this->model_checker.get_best_cost() < best_cost) { // The best cost was updated

      	// Update variables to keep track of best goal and state.
      	best_cost = this->model_checker.get_best_cost();
      	
      	// Grab goal node. It would be easier if I could just access min_cost_vertex from model_checker..
        // But that has a private access specifier so dayum 
      	trajectory_t traj;
        this->model_checker.get_solution (traj);
        state_t* best_goal_temp = traj.list_states.back();
        for(int i = 0; i < parameters.get_dimension(); ++i) { // Copy goal node values
        	(*best_goal)[i] = (*best_goal_temp)[i];
        }

      	// Update L matrix
      	double c_best = best_cost*sqrt(2); // Multiply by max speed
    		// This is needed for c_min
    		VectorXd x_start(parameters.get_dimension()), x_goal(parameters.get_dimension());
    		for(int i = 0; i < parameters.get_dimension(); ++i) { // Just get the positions
    			x_start(i) = this->root_vertex->state->state_vars[i];
    			x_goal(i) = this->best_goal->state_vars[i];
    		}
      	double c_min = (x_start.head(2) - x_goal.head(2)).norm()/sqrt(2); // SPECIFIC TO DOUBLE INTEGRATOR
      	for(int i = 0; i < parameters.get_dimension(); ++i) {
      		if (i == 0) {
      			L(i, i) = c_best/2; // Float division
      		} else {
      			L(i, i) = sqrt(pow(c_best,2) - pow(c_min,2))/2.0;
      		}
      	}

      	// Create M matrix
      	VectorXd a1 = x_goal - x_start;
      	a1.normalize(); // I'm assuming x_goal != x_start, otherwise this will produce weird errors
      	MatrixXd M(parameters.get_dimension(), parameters.get_dimension()); M.setZero();
      	M.col(0) = a1;

      	// Perform SVD on M
      	MatrixXd U = M.jacobiSvd(ComputeFullU | ComputeFullV).matrixU();
      	MatrixXd V = M.jacobiSvd(ComputeFullU | ComputeFullV).matrixV();

      	// Create W matrix
      	MatrixXd W(parameters.get_dimension(), parameters.get_dimension()); W.setZero();
      	for(int i = 0; i < parameters.get_dimension(); ++i) {
      		if (i == parameters.get_dimension() - 1) {
      			W(i, i) = U.determinant() * V.determinant();
      		} else {
      			W(i, i) = 1;
      		}
      	}

      	// Multiply to update C matrix
      	C = U * W * V.transpose();
      	// std::cout << "L:\n" << L << "\n";
      	// std::cout << "(C*L*L'*C')^-1:\n" << (C*L*L.transpose()*C.transpose()).inverse() << "\n";

      	// Set phase to 2
      	parameters.set_phase(2);
      }


      if (parameters.get_phase() >= 2) {  // Check whether phase 2 should occur
      

	// 6. Extend from the new vertex to the existing vertices in the ball to rewire the tree
	for (list<void*>::iterator iter = list_vertices_in_ball.begin(); iter != list_vertices_in_ball.end(); iter++) {
	
	  vertex_t *vertex_curr = (vertex_t*)(*iter);

	  if (vertex_curr == vertex_last)
	    continue;
	
	  // Attempt an extension from the extended vertex to the current vertex
	  trajectory_t *trajectory_curr = new trajectory_t;
	  list<state_t*> *intermediate_vertices_curr = new list<state_t*>;
	  bool free_tmp_memory = true;
	  exact_connection = -1;
	  if (this->extender.extend (vertex_last->state, vertex_curr->state,
				     &exact_connection, trajectory_curr, intermediate_vertices_curr) == 1) {
	  
	    if ( (exact_connection == 1) && (check_extended_trajectory_for_collision(vertex_last->state,trajectory_curr) == 1) ) {

	      // Calculate the cost to get to the extended state with the new trajectory
	      double cost_trajectory_to_curr = 
		this->cost_evaluator.evaluate_cost_trajectory (vertex_last->state, trajectory_curr);
	      double cost_curr = vertex_last->data.total_cost + cost_trajectory_to_curr;
	    
	    
	      // Check whether cost of the trajectory through vertex_last is less than the current trajectory
	      if (cost_curr < vertex_curr->data.total_cost) {
	      
	      
		// Delete the edge connecting the old parent and vertex_curr
		edge_t *edge_parent_curr = vertex_curr->incoming_edges.back();
		this->delete_edge (edge_parent_curr);
	      
		// Add vertex_curr's new parent
		this->insert_trajectory (vertex_last, trajectory_curr, intermediate_vertices_curr, vertex_curr);
		edge_t *edge_curr = vertex_curr->incoming_edges.back();
		edge_curr->data.edge_cost = cost_trajectory_to_curr;

		free_tmp_memory = false;
	      
		// Propagate the cost
		this->propagate_cost (vertex_curr, vertex_last->data.total_cost + edge_curr->data.edge_cost);	      

	      }
	    }
	  }

	  if (free_tmp_memory == true) {
	    delete trajectory_curr;
	    delete intermediate_vertices_curr;
	  }
	
	}
      }

      // Completed all phases, return with success
      delete state_sample;
      if (state_extended)
	delete state_extended;
      return 1;
    }
  }
  

  // 7. Handle the error case
  // If the first extension was not successful, or the trajectory was not collision free,
  //     then free the memory and return failure
  delete state_sample;
  delete trajectory;
  delete intermediate_vertices;
  
  return 0;

}


#endif
