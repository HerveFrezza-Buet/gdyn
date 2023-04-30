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

#include <cstddef>
#include <concepts>
#include <iterator>
#include <optional>
#include <functional>

#include <gdynSpecs.hpp>
#include <gdynTransition.hpp>

namespace gdyn {
  
  namespace iterators {
    
    /**
     * This is the sentinel for iterations on a dynamical system
     * orbit, when the system enters in a terminal state.
     */
    struct terminal_t {};
    inline constexpr terminal_t terminal {};

    // iterator for the tick view
    template<std::regular_invocable F>
    struct tick {
      using stored_function_type = std::function<decltype(std::declval<F>()()) ()>;
      using value_type = decltype(std::declval<F>()());
	
      using difference_type = std::ptrdiff_t;
      
    private:
      stored_function_type f;
      value_type value;
      
    public:
      
      tick()                       = default;
      tick(const tick&)            = default;
      tick& operator=(const tick&) = default;
      tick(tick&&)                 = default;
      tick& operator=(tick&&)      = default;
	
      tick(const F& f) : f(f), value(f()) {}
	
      auto& operator++()   {value = f(); return *this;}
      auto operator++(int) {auto res = *this; ++(*this); return res;}

      value_type operator*() const {return value;}

      bool operator==(terminal_t) const {return false;}
    };
      
    

    // This is for iterating on system orbits.
    template<specs::system SYSTEM,
	     specs::command_iterator<typename SYSTEM::command_type> COMMAND_ITERATOR,
	     typename COMMAND_SENTINEL>
    struct orbit {
      
    private:
      SYSTEM* system = nullptr;
      COMMAND_ITERATOR it;
      COMMAND_SENTINEL end;

    public:

      using difference_type = std::ptrdiff_t;

      struct value_type {
	using observation_type = typename SYSTEM::observation_type;
	using command_type     = typename SYSTEM::command_type;
	observation_type            observation;
	std::optional<command_type> command;
	
	value_type()                             = default;
	value_type(const value_type&)            = default;
	value_type& operator=(const value_type&) = default;
	value_type(value_type&&)                 = default;
	value_type& operator=(value_type&&)      = default;
	value_type(const observation_type& observation,
		   const command_type& command)
	  : observation(observation), command(command) {}
	  
      };
      
    private:
      
      value_type value;
      bool terminated = false;
      
    public:
      
      
      orbit()                              = default;
      orbit(const orbit&)                  = default;
      orbit(orbit&&)                       = default;
      orbit& operator=(const orbit& other) = default;
      orbit& operator=(orbit&&      other) = default;

      orbit(SYSTEM& system, COMMAND_ITERATOR it, COMMAND_SENTINEL end)
	: system(&system), it(it), end(end),
	  value(), terminated() {
	  if(it == end)
	    terminated = true;
	  else if(system) {
	    value.observation = *system;
	    value.command = *it;
	  }
	  else { // We are in a terminal state.
	    value.observation = *system;
	    value.command = std::nullopt;
	  }
      }
      
      bool operator==(terminal_t) const {return terminated;}
      auto& operator*() const {return value;}
      auto& operator++() {
	if(value.command) { // we are not in a terminal state (the has been checked at previous iteration).
	  (*system)(*(value.command)); 
	  value.observation = *(*system);
	  ++it;
	  if(it == end || !(*system))
	    value.command = std::nullopt;
	  else
	    value.command = *it;
	}
	else // we are in a terminal state
	  terminated = true;
	return *this;
      }
      auto  operator++(int) {auto res = *this; ++(*this); return res;}   
    };

    template<specs::orbit_iterator ORBIT_ITERATOR>
    using observation_t = typename ORBIT_ITERATOR::value_type::observation_type;
				   
    template<specs::orbit_iterator ORBIT_ITERATOR>
    using command_t = typename ORBIT_ITERATOR::value_type::command_type;


    // This is for iterating on system transitions.
    template<specs::orbit_iterator ORBIT_ITERATOR,
	     typename ORBIT_SENTINEL>
    struct transition {
      
    private:
      ORBIT_ITERATOR it;
      ORBIT_SENTINEL end;

    public:

      using value_type = gdyn::transition<observation_t<ORBIT_ITERATOR>, command_t<ORBIT_ITERATOR>>;
      
    private:
      
      std::optional<value_type> value;
      
      
    public:
      
      using difference_type = std::ptrdiff_t;
      
      transition()                                   = default;
      transition(const transition&)                  = default;
      transition(transition&&)                       = default;
      transition& operator=(const transition& other) = default;
      transition& operator=(transition&&      other) = default;

      transition(ORBIT_ITERATOR begin, ORBIT_SENTINEL end) : it(begin), end(end), value() {
	if(it != end) {
	  auto start = *(it++);
	  if(it != end) 
	    value = make_transition(start, *it);
	}
      }
      
      bool operator==(terminal_t) const {return it == end || !value;}
      
      auto& operator++() {
	++it;
	if(it == end)
	  value = std::nullopt;
	else
	  *value += *it; // We skip to the next transition.
	return *this;
      }
      const auto& operator*() const {return *value;} 
      auto  operator++(int)         {auto res = *this; ++(*this); return res;}   
    };

    
  }
}
