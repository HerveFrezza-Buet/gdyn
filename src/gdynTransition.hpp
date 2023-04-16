#pragma once


#include <optional>
#include <iostream>
#include <tuple>

#include <ds-specs.hpp>

namespace gdyn {

  /**
   * This is an orbit "segment", i.e. two consecutive orbit points.
   */
  template<typename OBSERVATION, typename COMMAND>
  struct transition {
    OBSERVATION            observation;      //!< the current observation
    COMMAND                command;          //!< the current command
    OBSERVATION            next_observation; //!< the next observation
    std::optional<COMMAND> next_command;     //!< eventually, the next command (if the systeem has not reached a terminal state).

    /**
     * We shift the current transition so thet it hosts the next one.
     */
    template<specs::orbit_point ORBIT_POINT>
    void operator+=(const ORBIT_POINT& next) {
      observation = next_observation;
      command = *next_command;
      next_observation = next.observation;
      next_command = next.command;
    }

    transition()                             = default;
    transition(const transition&)            = default;
    transition(transition&&)                 = default;
    transition& operator=(const transition&) = default;
    transition& operator=(transition&&)      = default;

    transition(const OBSERVATION&            observation,
	       const COMMAND&                command,
	       const OBSERVATION&            next_observation,
	       const std::optional<COMMAND>& next_command)
      : observation(observation), command(command),
	next_observation(next_observation), next_command(next_command) {}
    
    transition(const OBSERVATION&            observation,
	       const COMMAND&                command,
	       const OBSERVATION&            next_observation)
      : transition(observation, command, next_observation, std::nullopt) {}
    
    transition(const OBSERVATION&            observation,
	       const COMMAND&                command,
	       const OBSERVATION&            next_observation,
	       const COMMAND&                next_command)
      : transition(observation, command, next_observation, next_command) {}

    bool is_terminal() const {return !(next_command.has_value());}
  };


  /**
   * This builds a transition.
   *
   * @param current An orbit point, that must not be a terminal one.
   * @param next  An orbit point.
   */
  template<specs::orbit_point ORBIT_POINT>
  auto make_transition(const ORBIT_POINT& current, const ORBIT_POINT& next) {
    return transition<typename ORBIT_POINT::observation_type, typename ORBIT_POINT::command_type>(current.observation, *current.command, next.observation, next.command);
  }

  
		   
}
