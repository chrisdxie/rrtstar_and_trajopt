// Standard header files
#include <iostream>
using namespace std;

#include <boost/python.hpp>
#include <boost/python/numeric.hpp>
#include <boost/python/tuple.hpp>
#include <boost/numpy.hpp>
//#include <boost/filesystem.hpp>

namespace py = boost::python;
namespace np = boost::numpy;

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

// SMP HEADER FILES ------
#include <smp/components/extenders/double_integrator.hpp>
#include <smp/components/samplers/boost_random_uniform.hpp>
#include <smp/components/collision_checkers/standard.hpp>
#include <smp/components/distance_evaluators/kdtree.hpp>
#include <smp/components/multipurpose/minimum_time_reachability.hpp>

#include <smp/planners/rrtstar.hpp>

#include <smp/planner_utils/trajectory.hpp>

#include <string>
#include <stack>
#include <fstream>
#include <iostream>
#include <ctime>
#include <iomanip>


// SMP TYPE DEFINITIONS -------
using namespace smp;

// State, input, vertex_data, and edge_data definitions
typedef state_double_integrator<2> state_t;
typedef input_double_integrator<2> input_t;
typedef minimum_time_reachability_vertex_data vertex_data_t;
typedef minimum_time_reachability_edge_data edge_data_t;

// Create the typeparams structure
typedef struct _typeparams {
  typedef state_t state;
  typedef input_t input;
  typedef vertex_data_t vertex_data;
  typedef edge_data_t edge_data;
} typeparams; 

// Define the trajectory type
typedef trajectory<typeparams> trajectory_t;

// Define all planner component types
typedef sampler_uniform<typeparams,4> sampler_t;
typedef distance_evaluator_kdtree<typeparams,4> distance_evaluator_t;
typedef extender_double_integrator<typeparams,2> extender_t;
typedef collision_checker_standard<typeparams,4> collision_checker_t;
typedef minimum_time_reachability<typeparams,4> min_time_reachability_t;

// Define all algorithm types
typedef rrtstar<typeparams>  rrtstar_t;


// Python plotting stuff
np::ndarray eigen_to_ndarray(const MatrixXd& m) {
  if (m.cols() == 1) {
    py::tuple shape = py::make_tuple(m.rows());
    np::dtype dtype = np::dtype::get_builtin<float>();
    np::ndarray n = np::zeros(shape, dtype);
    for(int i=0; i < m.rows(); ++i) {
      n[py::make_tuple(i)] = m(i);
    }
    return n;
  } else {
    py::tuple shape = py::make_tuple(m.rows(), m.cols());
    np::dtype dtype = np::dtype::get_builtin<float>();
    np::ndarray n = np::zeros(shape, dtype);
    for(int i=0; i < m.rows(); ++i) {
      for(int j=0; j < m.cols(); ++j) {
        n[py::make_tuple(i,j)] = m(i,j);
      }
    }

    return n;
  }
}

py::object init_display() {
    // necessary initializations
  Py_Initialize();
  np::initialize();
  py::numeric::array::set_module_and_type("numpy", "ndarray");

  // use boost to find directory of python_plot.py
  // std::string working_dir = boost::filesystem::current_path().normalize().string();
  // The above won't work with my compiler for some reason...
  std::string working_dir = "/home/chris/rrtstar_and_trajopt";
  working_dir += "/python_plotting/";
  
    // necessary imports
  py::object main_module = py::import("__main__");
  py::object main_namespace = main_module.attr("__dict__");
  py::exec("import sys, os", main_namespace);
  // add working_dir to sys.path
  py::exec(py::str("sys.path.append('"+working_dir+"')"), main_namespace);
  // get python file module
  py::object plot_mod = py::import("python_plot_double_integrator");

    // get function from module
  py::object plotter = plot_mod.attr("plot");
  return plotter;
}

void plot(py::object plotter, np::ndarray all_states, np::ndarray all_parents, np::ndarray goal_states, np::ndarray goal_inputs, np::ndarray obstacles, std::string obs_file,
    np::ndarray goal_region, int iter, double cost, int scene_num) {

  try {
      // pass control to python now
    plotter(all_states, all_parents, goal_states, goal_inputs, obstacles, goal_region, iter, obs_file, cost, scene_num);
  }
  catch(py::error_already_set const &) {
      // will pass python errors to cpp for printing
    PyErr_Print();
  }
}


MatrixXd read_in_obstacle_file(std::string file_name, collision_checker_t& collision_checker) {

  ifstream fin(file_name.c_str());
  std::vector<double> data;

  std::copy(std::istream_iterator<double>(fin), // this denotes "start of stream"
            std::istream_iterator<double>(),   // this denodes "end of stream"
            std::back_inserter<std::vector<double> >(data));

  MatrixXd obstacles(4, data.size()/4); obstacles.setZero();

  int index = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < data.size()/4; j++) {
      obstacles(i, j) = data[index++];
    }
  }

  for (int i = 0; i < obstacles.cols(); i++) {
    region<4> obs;
    obs.center[0] = obstacles(0,i);
    obs.size[0] = obstacles(1,i);
    obs.center[1] = obstacles(2,i);
    obs.size[1] = obstacles(3,i);

    for (int j = 2; j < 4; j++) {
      obs.center[j] = 0.0;
      obs.size[j] = 20.0;
    }  

    collision_checker.add_obstacle (obs);

  }

  return obstacles;

}

/*
 *  Arguments are: <NUM_SECONDS> <RANDOMIZE in {true, false, <seed>}> <SCENE_FILE_NAME> <ID_NUMBER_FOR_STATS> <SCENE_NUM>
 */ 
int
main (int argc, char* argv[]) {


  // Grab number of iterations from command line input
  int MAX_SECONDS = 60; // Default
  if (argc >= 2) {
    MAX_SECONDS = atoi(argv[1]);
  }

  // Expecting something in the form of: executable <NUM_ITERS> <{"random" or some random seed}>
  int seed = 0;
  if (argc >= 3) {
    std::string arg2 = argv[2];
    if (arg2.compare("true") == 0) {
      srand(time(NULL));
      seed = time(NULL);
    } else if (arg2.compare("false") == 0) {
      srand(seed);
    } else {
      //srand(atoi(argv[2])); // seed was passed in
      seed = atoi(argv[2]);
    }
  }
  int stats_id = 1;
  if (argc >= 5) {
    stats_id = atoi(argv[4]);
  }
  int scene_num = atoi(argv[5]);

  // 1. CREATE PLANNING OBJECTS
  
  // 1.a Create the components
  sampler_t sampler(seed);
  distance_evaluator_t distance_evaluator;
  extender_t extender;
  collision_checker_t collision_checker;
  min_time_reachability_t min_time_reachability;

  // 1.b Create the planner algorithm -- Note that the min_time_reachability variable acts both
  //                                       as a model checker and a cost evaluator.
  rrtstar_t planner (sampler, distance_evaluator, extender, collision_checker, 
		     min_time_reachability, min_time_reachability);

  planner.parameters.set_phase (2);   // The phase parameter can be used to run the algorithm as an RRT, 
                                      // See the documentation of the RRT* algorithm for more information.

  planner.parameters.set_gamma (15.0);    // Set this parameter should be set at least to the side length of
                                          //   the (bounded) state space. E.g., if the state space is a box
                                          //   with side length L, then this parameter should be set to at 
                                          //   least L for rapid and efficient convergence in trajectory space.
  planner.parameters.set_dimension (4);       // Double integrator state space is four dimensional.
  planner.parameters.set_max_radius (20.0);   // This parameter should be set to a high enough value. In practice,
                                             //   one can use smaller values of this parameter to get a good 
                                             //   solution quickly, while preserving the asymptotic optimality.






  // 2. INITALIZE PLANNING OBJECTS

  // 2.a Initialize the sampler
  region<4> sampler_support;
  for (int i = 0; i < 2; i++) {
    sampler_support.center[i] = 0.0;
    sampler_support.size[i] = 20.0;
  }
  for (int i = 2; i < 4; i++) {
    sampler_support.center[i] = 0.0;
    sampler_support.size[i] = 2.0;
  }
  sampler.set_support (sampler_support);

  
  // 2.b Initialize the distance evaluator
  //     Nothing to initialize. One could change the kdtree weights.


  // 2.c Initialize the extender

 
  // 2.d Initialize the collision checker
  std::string scene_file_name = argv[3];
  MatrixXd obstacles = read_in_obstacle_file(scene_file_name, collision_checker);

  // region<4> obstacle_1;
  // region<4> obstacle_2;
  // region<4> obstacle_3;

  // obstacle_1.center[0] = 2;
  // obstacle_1.center[1] = 8;
  // obstacle_1.size[0] = 1;
  // obstacle_1.size[1] = 3;
  // obstacle_2.center[0] = 3;
  // obstacle_2.center[1] = 3;
  // obstacle_2.size[0] = 4;
  // obstacle_2.size[1] = 2;
  // obstacle_3.center[0] = 7;
  // obstacle_3.center[1] = 7;
  // obstacle_3.size[0] = 2;
  // obstacle_3.size[1] = 2;
  // for (int i = 2; i < 4; i++) {
  //   obstacle_1.center[i] = 0.0;
  //   obstacle_1.size[i] = 20.0;
  // }
  //   for (int i = 2; i < 4; i++) {
  //   obstacle_2.center[i] = 0.0;
  //   obstacle_2.size[i] = 20.0;
  // }
  //   for (int i = 2; i < 4; i++) {
  //   obstacle_3.center[i] = 0.0;
  //   obstacle_3.size[i] = 20.0;
  // }
  // collision_checker.add_obstacle (obstacle_1);
  // collision_checker.add_obstacle (obstacle_2);
  // collision_checker.add_obstacle (obstacle_3);
  

  // 2.e Initialize the model checker
  region<4> region_goal;
  for (int i = 0; i < 2; i++) {
    region_goal.center[i] = 9.0;
    region_goal.size[i] = 1e-2;
  }
  for (int i = 2; i < 4; i++) {
    region_goal.center[i] = 0.0;
    region_goal.size[i] = 1e-2;
  }
  min_time_reachability.set_goal_region (region_goal);


  // 2.f Initialize the planner
  state_t *state_initial = new state_t;
  for (int i = 0; i < 4; i++) {
    state_initial->state_vars[i] = 0.0;
  }
  planner.initialize (state_initial);


  
  // 3. SETUP FOR PLOTTING

  std::cout << "Initializing display...\n";
  py::object plotter = init_display(); // Initialize python interpreter and pyplot plot

  // Get goal region
  // Same format as obstacles. Only one column, since one goal region
  MatrixXd goal_region(4,1);
  goal_region.col(0) << region_goal.center[0], region_goal.size[0], region_goal.center[1], region_goal.size[1];

  np::ndarray obstacles_np = eigen_to_ndarray(obstacles);
  np::ndarray goal_region_np = eigen_to_ndarray(goal_region);




  // Time it
  std::clock_t start;
  double duration;
  start = std::clock();
  double last_time = start / (double) CLOCKS_PER_SEC;

  // 4. RUN THE PLANNER 
  for (int i = 0; true; i++){

    planner.iteration ();
    
    if (i%100 == 0){
      cout << "Iteration : " << i << endl;

      double report_interval = 1; // Report stats every x secs
      double curr_time = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

      if (curr_time - last_time > report_interval) {
        // Write time, # of nodes, cost to file
        std::ofstream outfile("RRTSTAR_double_integrator_statistics_" + std::to_string(MAX_SECONDS) + "_seconds_run_" + std::to_string(stats_id) + "_iters.txt", ios::app);
        if (outfile.is_open()) {
          outfile << std::setprecision(10) << ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
          outfile << ", " << planner.get_num_vertices();
          outfile << ", " << min_time_reachability.get_best_cost() << std::endl;
          outfile.close();
        }

        last_time = curr_time;

      }

      if (curr_time > MAX_SECONDS) {
        std::cout << "Done\n";
        break;
      }

    }
    if (false) { // Plot every 1k iterations, and save automatically in a folder called "pics"

      trajectory_t trajectory_final;
      min_time_reachability.get_solution (trajectory_final);

      // Get states from best trajectory
      int num_states = trajectory_final.list_states.size();
      MatrixXd states(4, num_states);

      cout << "Plotting states of best trajectory after " << i+1 << " iterations:" << endl;
      int index = 0;
      for (typename list<state_t*>::iterator iter_state = trajectory_final.list_states.begin(); 
           iter_state != trajectory_final.list_states.end(); iter_state++) {
        state_t *traj_state = *iter_state;
        for (int i = 0; i < 4; i++) {
          states(i, index) = traj_state->state_vars[i];
        }
        index++;
      }
      np::ndarray states_np = eigen_to_ndarray(states);

      int num_inputs = trajectory_final.list_inputs.size();
      MatrixXd inputs(3, num_inputs);

      cout << "Printing inputs of best trajectory:" << endl;
      index = 0;
      for (typename list<input_t*>::iterator iter_input = trajectory_final.list_inputs.begin();
           iter_input != trajectory_final.list_inputs.end(); iter_input++) {
        input_t *traj_input = *iter_input;
        for (int i = 0; i < 3; i++) {
          inputs(i, index) = traj_input->input_vars[i];
        }
        index++;
      }
      np::ndarray inputs_np = eigen_to_ndarray(inputs);

      double best_cost = min_time_reachability.get_best_cost();

      // DFS on tree to get states and parents:
      MatrixXd all_states(4, planner.get_num_vertices()); all_states.setZero();
      MatrixXd all_parents(4, planner.get_num_vertices()); all_parents.setZero();
      int k = 0;
      std::stack<vertex<typeparams>*> fringe;
      fringe.push(planner.root_vertex);
      while(fringe.size() > 0) {
        vertex<typeparams>* candidate = fringe.top();
        fringe.pop();
        if (k == 0) {
          for(int j = 0; j < 4; ++j) {
            all_states(j,k) = candidate->state->state_vars[j];
            all_parents(j,k) = candidate->state->state_vars[j];
          }
        } else {
          vertex<typeparams>* parent = (*(candidate->incoming_edges.begin()))->vertex_src;
          for(int j = 0; j < 4; ++j) {
            all_states(j,k) = candidate->state->state_vars[j];
            all_parents(j,k) = parent->state->state_vars[j];
          }
        }
        k++;
        for(std::list<edge<typeparams>*>::iterator it = candidate->outgoing_edges.begin();
            it != candidate->outgoing_edges.end(); it++) {
          fringe.push((*it)->vertex_dst);
        }
      }

      np::ndarray all_states_np = eigen_to_ndarray(all_states);
      np::ndarray all_parents_np = eigen_to_ndarray(all_parents);

      plot(plotter, all_states_np, all_parents_np, states_np, inputs_np, obstacles_np, scene_file_name, goal_region_np, i+1, best_cost, scene_num);
    }
  }



  
  trajectory_t trajectory_final;
  min_time_reachability.get_solution (trajectory_final);

  // Get states from best trajectory
  int num_states = trajectory_final.list_states.size();
  MatrixXd states(4, num_states);

  int index = 0;
  for (typename list<state_t*>::iterator iter_state = trajectory_final.list_states.begin(); 
       iter_state != trajectory_final.list_states.end(); iter_state++) {
    state_t *traj_state = *iter_state;
    for (int i = 0; i < 4; i++) {
      states(i, index) = traj_state->state_vars[i];
    }
    index++;
  }

  np::ndarray states_np = eigen_to_ndarray(states);

  int num_inputs = trajectory_final.list_inputs.size();
  MatrixXd inputs(3, num_inputs);

  index = 0;
  for (typename list<input_t*>::iterator iter_input = trajectory_final.list_inputs.begin();
      iter_input != trajectory_final.list_inputs.end(); iter_input++) {
    input_t *traj_input = *iter_input;
    for (int i = 0; i < 3; i++) {
      inputs(i, index) = traj_input->input_vars[i];
    }
    index++;
  }
  np::ndarray inputs_np = eigen_to_ndarray(inputs);


  double best_cost = min_time_reachability.get_best_cost();

  // DFS on tree to get states and parents:
  MatrixXd all_states(4, planner.get_num_vertices() * 20); all_states.setZero();
  MatrixXd all_parents(4, planner.get_num_vertices() * 20); all_parents.setZero();
  int k = 0;
  std::stack<vertex<typeparams>*> fringe;
  fringe.push(planner.root_vertex);
  while(fringe.size() > 0) {
    vertex<typeparams>* candidate = fringe.top();
    fringe.pop();
    if (k == 0) {
      for(int j = 0; j < 4; ++j) {
        all_states(j,k) = candidate->state->state_vars[j];
        all_parents(j,k) = candidate->state->state_vars[j];
      }
      k++;
    } else {
      vertex<typeparams>* parent = (*(candidate->incoming_edges.begin()))->vertex_src;
      trajectory_t* traj = (*(candidate->incoming_edges.begin()))->trajectory_edge;

      for (typename list<state_t*>::iterator it = traj->list_states.begin(); it != traj->list_states.end(); it++) {
        state_t* st = *it;
        for (int j = 0; j < 4; j++) {
          all_states(j,k) = st->state_vars[j];
        }
        if (it == traj->list_states.begin()) {
          for (int j = 0; j < 4; j++) {
            all_parents(j,k) = parent->state->state_vars[j];
          }
        } else {
          for (int j = 0; j < 4; j++) {
            all_parents(j,k) = all_states(j,k-1);
          }
        }
        k++;
      }

      for(int j = 0; j < 4; ++j) {
        all_states(j,k) = candidate->state->state_vars[j];
        all_parents(j,k) = all_states(j,k-1);
      }
      k++;
    }
    for(std::list<edge<typeparams>*>::iterator it = candidate->outgoing_edges.begin();
        it != candidate->outgoing_edges.end(); it++) {
      fringe.push((*it)->vertex_dst);
    }
  }

  MatrixXd final_states = all_states.leftCols(k);
  MatrixXd final_parents = all_parents.leftCols(k);

  np::ndarray all_states_np = eigen_to_ndarray(final_states);
  np::ndarray all_parents_np = eigen_to_ndarray(final_parents);

  plot(plotter, all_states_np, all_parents_np, states_np, inputs_np, obstacles_np, scene_file_name, goal_region_np, MAX_SECONDS, best_cost, scene_num);

  // Successful execution, return 1
  return 1;
  
}
