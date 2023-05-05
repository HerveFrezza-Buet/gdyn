/*

Copyright 2023 Herve FREZZA-BUET

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/


#pragma once


#include <iterator>
#include <tuple>


namespace gdyn {
  namespace specs {


    /**
     * @short This specifies what a dynamical system is in gdyn.
     */
    template<typename DYNAMICAL_SYSTEM>
    concept system =
      requires {
      // Let us require nested types definitions.
      typename DYNAMICAL_SYSTEM::state_type;
      typename DYNAMICAL_SYSTEM::observation_type;
      typename DYNAMICAL_SYSTEM::command_type;
    } &&
    requires (DYNAMICAL_SYSTEM system, DYNAMICAL_SYSTEM const constant_system,
	      typename DYNAMICAL_SYSTEM::state_type const constant_state,
	      typename DYNAMICAL_SYSTEM::observation_type observation,
	      typename DYNAMICAL_SYSTEM::command_type const constant_command) {
      // Let us require syntactical properties.
      system      = constant_state;                   // Set the state of the system.
      observation = *constant_system;                 // Get the current observation.
      system(constant_command);                       // Performs the transition
      {constant_system} -> std::convertible_to<bool>; // false means in terminal state.
    };

    /**
     * @short This specifies what a controller is.
     * 
     * Indeed, it is only a function that provides the command to be
     * done from the current observation. 
     */
    template<typename CONTROLLER, typename OBSERVATION, typename COMMAND>
    concept controller =
      requires(CONTROLLER const constant_controller,
	       OBSERVATION const constant_observation,
	       COMMAND command) {command = constant_controller(constant_observation);};

    /**
     * @short This specifies an iterator providing commands to a system.
     */
    template<typename COMMAND_ITERATOR, typename COMMAND>
    concept command_iterator =
      std::input_iterator<COMMAND_ITERATOR>
      && std::convertible_to<std::iter_value_t<COMMAND_ITERATOR>, COMMAND>;

    /**
     * @short An orbit is made of points, specified here.
     */
    template<typename ORBIT_VALUE>
    concept orbit_point =
      requires {
      typename ORBIT_VALUE::observation_type;
      typename ORBIT_VALUE::command_type;
    } &&
    requires (ORBIT_VALUE value, ORBIT_VALUE::observation_type obs, ORBIT_VALUE::command_type cmd, bool test) {
      obs  = value.observation;
      cmd  = *(value.command);
      test = value.command.has_value();
    };
    
    /**
     * @short This specifies an iterator on an orbit.
     */
    template<typename ORBIT_ITERATOR>
    concept orbit_iterator =
      std::input_iterator<ORBIT_ITERATOR>
      && orbit_point<typename std::iter_value_t<ORBIT_ITERATOR>>;
  }
}
