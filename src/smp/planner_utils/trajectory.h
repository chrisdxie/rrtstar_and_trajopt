/*! \file trajectory.h
  \brief Definition of the trajectory class

*/

#ifndef _SMP_TRAJECTORY_H_
#define _SMP_TRAJECTORY_H_

#include<list>

using namespace std;


namespace smp {

    //! Trajectory definition as a states with interleaving inputs. 
    /*!
      The trajectory class, composed of a list of states and a list of inputs,
      is an implementation of the notion of a trajectory that connects two given
      states in the graph. 
      
      \ingroup graphs
    */
    template< class typeparams >
    class trajectory {

        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;

        typedef trajectory<typeparams> trajectory_t;

    public:
    
        //! A list of the states in the trajectory.
        list< state_t* > list_states;

        //! A list of the inputs in the trajectory.
        list< input_t* > list_inputs;

        trajectory ();
        ~trajectory ();

        //! Clears the trajectory.
        /*!
          This function clears both the state list and the input list in the trajectory.
          But, it does NOT attempt to free the memory occupied by the said states and 
          inputs. 
        */
        int clear ();

        //! Clears the trajectory and frees the memory.
        /*!
          This function clears both the state list and the input list in the trajectory.
          It also frees the memory occupied by the said states and the inputs, by calling
          the delete operator with each state and input present in the lists. 
        */
        int clear_delete ();


        //! Checks whether the trajectory contains any states or inputs.
        /*!
          This function returns 1 whenever the trajectory includes no states and no inputs.
        */
        bool empty () {if (list_states.empty() && list_inputs.empty()) return true; else return false;}


        /**
         * \brief Adds a given trajectory to the beginning of this trajectory 
         *
         * @params trajectory_in The trajectory that will be added in 
         *                       the beginning of the this trajectory. 
         *
         * @returns Returns 1 for success, and a negative value to indicate failure.
         */
        int push_front (trajectory_t *trajectory_in); 


        /**
         * \brief Adds a given trajectory to the end of this trajectory 
         *
         * @params trajectory_in The trajectory that will be added at
         *                       the end of the this trajectory. 
         *
         * @returns Returns 1 for success, and a negative value to indicate failure.
         */    
        int push_back (trajectory_t *trajectory_in); 


        /**
         * \brief Adds a given state to the beginning of this trajectory 
         *
         * @params state_in The state that will be added in 
         *                  the beginning of the this trajectory. 
         *
         * @returns Returns 1 for success, and a negative value to indicate failure.
         */
        int push_front (state_t *state_in); 


        /**
         * \brief Adds a given state to the end of this trajectory 
         *
         * @params state_in The state that will be added in 
         *                  the end of the this trajectory. 
         *
         * @returns Returns 1 for success, and a negative value to indicate failure.
         */
        int push_back (state_t *state_in); 

        
        /**
         * \brief Adds a given input to the beginning of this trajectory 
         *
         * @params input_in The input that will be added in 
         *                  the beginning of the this trajectory. 
         *
         * @returns Returns 1 for success, and a negative value to indicate failure.
         */
        int push_front (input_t *input_in); 


        /**
         * \brief Adds a given input to the beginning of this trajectory 
         *
         * @params input_in The input that will be added in 
         *                  the beginning of the this trajectory. 
         *
         * @returns Returns 1 for success, and a negative value to indicate failure.
         */
        int push_back (input_t *input_in); 

        
    };


}

#endif
