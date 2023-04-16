#pragma once


// This file is useless for the users of our dynamical-system
// library. It is only some static checkings that everything is
// defined right, according to concepts.
#include <iterator>
#include <ranges>
#include <vector>

#include <gdynSpecs.hpp>
#include <gdynIterators.hpp>
#include <gdynRanges.hpp>

namespace gdyn {
  namespace checkings {
    
    // System
    // ------
    
    struct system_type {
      using observation_type = double;
      using command_type     = int;
      using state_type       = char;
      
      system_type& operator=(const state_type& init_state) {return *this;}
      observation_type operator*() const {return 0;}
      void operator()(command_type command) {}
      operator bool() const {return true;} 
    };
    static_assert(specs::system<system_type>);

    
    // Controller
    // ----------
    
    inline double controller(int) {return 0;}
    using controller_type = decltype(controller);
    static_assert(specs::controller<controller_type, system_type::observation_type, system_type::command_type>);

    
    // Tick
    // ----
    
    using tick_type = ranges::tick<std::function<int ()>>;
    using tick_iterator_type = std::ranges::iterator_t<tick_type>;
    static_assert(std::ranges::input_range<tick_type>);
    static_assert(std::input_iterator<tick_iterator_type>);

    
    // Orbit
    // -----

    using orbit_type = decltype(std::declval<tick_type>() | views::orbit(std::declval<system_type&>()));
    using orbit_iterator_type = std::ranges::iterator_t<orbit_type>;
    static_assert(std::ranges::input_range<orbit_type>);
    static_assert(specs::orbit_iterator<orbit_iterator_type>);

  }
}
