/*

Copyright 2023 Herve FREZZA-BUET, Alain DUTECH

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


#include <optional>
#include <iostream>
#include <tuple>

#include <gdynConcepts.hpp>

namespace gdyn {

  /**
   * This is an orbit "segment", i.e. two consecutive orbit points.
   */
  template<typename OBSERVATION, typename COMMAND, typename REPORT>
  struct transition {
    OBSERVATION            observation;      //!< the current observation
    COMMAND                command;          //!< the current command
    REPORT                 report;           //!< the resulting report
    OBSERVATION            next_observation; //!< the next observation
    std::optional<COMMAND> next_command;     //!< eventually, the next command (if the systeem has not reached a terminal state).

    /**
     * We shift the current transition so thet it hosts the next one.
     */
    template<concepts::orbit_point ORBIT_POINT>
    void operator+=(const ORBIT_POINT& next) {
      observation = next_observation;
      command = *next_command;
      report = *(next.previous_report); // There must be a report in the option.
      next_observation = next.current_observation;
      next_command = next.next_command;
    }

    transition()                             = default;
    transition(const transition&)            = default;
    transition(transition&&)                 = default;
    transition& operator=(const transition&) = default;
    transition& operator=(transition&&)      = default;

    transition(const OBSERVATION&            observation,
	       const COMMAND&                command,
	       const REPORT&                 report,
	       const OBSERVATION&            next_observation,
	       const std::optional<COMMAND>& next_command)
      : observation(observation), command(command), report(report),
	next_observation(next_observation), next_command(next_command) {}
    
    transition(const OBSERVATION&            observation,
	       const COMMAND&                command,
	       const REPORT&  report,
	       const OBSERVATION&            next_observation)
      : transition(observation, command, report, next_observation, std::nullopt) {}
    
    transition(const OBSERVATION&            observation,
	       const COMMAND&                command,
	       const REPORT&                 report,
	       const OBSERVATION&            next_observation,
	       const COMMAND&                next_command)
      : transition(observation, command, report, next_observation, next_command) {}

    bool is_terminal() const {return !(next_command.has_value());}
  };


  /**
   * This builds a transition.
   *
   * @param current An orbit point, that must not be a terminal one.
   * @param next  An orbit point.
   */
  template<concepts::orbit_point ORBIT_POINT>
  auto make_transition(const ORBIT_POINT& current, const ORBIT_POINT& next) {
    return transition<typename ORBIT_POINT::observation_type, typename ORBIT_POINT::command_type, typename ORBIT_POINT::report_type>(current.current_observation, *(current.next_command), *(next.previous_report), next.current_observation, next.next_command);
  }

  
		   
}
