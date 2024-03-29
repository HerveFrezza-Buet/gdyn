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


// This file is useless for the users of our dynamical-system
// library. It is only some static checkings that everything is
// defined right, according to concepts.
#include <iterator>
#include <ranges>
#include <vector>

#include <gdynConcepts.hpp>
#include <gdynIterators.hpp>
#include <gdynRanges.hpp>
#include <gdynSystem.hpp>

namespace gdyn {
  namespace checkings {
    
    // System
    // ------
    
    struct system_type {
      using observation_type = double;
      using command_type     = int;
      using state_type       = char;
      using report_type      = float;
      
      system_type& operator=(const state_type& init_state) {return *this;}
      observation_type operator*() const {return 0;}
      report_type operator()(command_type command) {return 0.;}
      operator bool() const {return true;} 
    };
    static_assert(concepts::system<system_type>);

    
    // Transparent System
    // ------------------

    struct transparent_system_type {
      using observation_type = double;
      using command_type     = int;
      using state_type       = char;
      using report_type      = float;
      
      transparent_system_type& operator=(const state_type& init_state) {return *this;}
      observation_type operator*() const {return 0;}
      state_type state() const {return 'a';}
      report_type operator()(command_type command) {return 0.;}
      operator bool() const {return true;} 
    };
    static_assert(concepts::transparent_system<transparent_system_type>);
    static_assert(concepts::system<transparent_system_type>);
    static_assert(concepts::system<system::exposed<transparent_system_type>>);

    

    
    // Controller
    // ----------
    
    inline double controller(int) {return 0;}
    using controller_type = decltype(controller);
    static_assert(concepts::controller<controller_type, system_type::observation_type, system_type::command_type>);

    
    // Pulse
    // -----
    
    using pulse_type = views::pulse<std::function<int ()>>;
    using pulse_iterator_type = std::ranges::iterator_t<pulse_type>;
    static_assert(std::ranges::range<pulse_type>);
    static_assert(std::ranges::input_range<pulse_type>);

    using T = pulse_type;
    static_assert(std::ranges::enable_view<pulse_type>);
    static_assert(std::ranges::view<pulse_type>);
    static_assert(std::ranges::viewable_range<pulse_type>);

    
    static_assert(std::input_iterator<pulse_iterator_type>);

    
    // Orbit
    // -----

    using orbit_type = decltype(std::declval<pulse_type>() | views::orbit(std::declval<system_type&>()));
    using orbit_iterator_type = std::ranges::iterator_t<orbit_type>;
    static_assert(std::ranges::input_range<orbit_type>);
    static_assert(concepts::orbit_iterator<orbit_iterator_type>);

  }
}
