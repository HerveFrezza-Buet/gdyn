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
    static_assert(std::ranges::range<tick_type>);
    static_assert(std::ranges::input_range<tick_type>);

    using T = tick_type;
    static_assert(std::ranges::enable_view<tick_type>);
    static_assert(std::ranges::view<tick_type>);
    static_assert(std::ranges::viewable_range<tick_type>);

    
    static_assert(std::input_iterator<tick_iterator_type>);

    
    // Orbit
    // -----

    using orbit_type = decltype(std::declval<tick_type>() | views::orbit(std::declval<system_type&>()));
    using orbit_iterator_type = std::ranges::iterator_t<orbit_type>;
    static_assert(std::ranges::input_range<orbit_type>);
    static_assert(specs::orbit_iterator<orbit_iterator_type>);

  }
}
