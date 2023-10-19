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
#include <ranges>
#include <tuple>

#include <gdynConcepts.hpp>
#include <gdynIterators.hpp>

// Inspired from
// See https://mariusbancila.ro/blog/2020/06/06/a-custom-cpp20-range-view/


namespace gdyn {
  namespace ranges  {

    // ########
    // #      #
    // # Tick #
    // #      #
    // ########

    /**
     * This range calls a function f() forever, the iterator provides
     * successive results of these calls.
     */
    template<std::regular_invocable F>
    class tick : public std::ranges::view_interface<tick<F>> {
    private:
      using stored_function_type = std::function<decltype(std::declval<F>()()) ()>;
      stored_function_type f;
    public:

      tick()                       = delete;
      tick(const tick&)            = default;
      tick& operator=(const tick&) = default;
      tick(tick&&)                 = default;
      tick& operator=(tick&&)      = default;
	
      tick(const F& f) : f(f) {}

      constexpr auto begin() const {return iterators::tick<stored_function_type>(f);}
      constexpr auto end()   const {return iterators::terminal;} // unreachable.
      
      template<typename CLOSURE>
      constexpr auto operator|(const CLOSURE& closure) const {return closure(*this);}
    };

    
    // ##############
    // #            #
    // # Controller #
    // #            #
    // ##############


    /**
     * This provides a command from a controller, i.e. a function that
     * computes a command from the current system observation.
     */
    template<concepts::system SYSTEM,
	     concepts::controller<typename SYSTEM::observation_type, typename SYSTEM::command_type> CONTROLLER>
    auto controller(SYSTEM& system, const CONTROLLER& controller) {return tick([&system, controller](){return controller(*system);});}
    
    
    // #########
    // #       #
    // # Orbit #
    // #       #
    // #########

    // Range adaptors take a range type as template argument and
    // builds up another range type. Here, the adaptor is orbit... it
    // somehow "orbitifies" a range.
    //
    // ranges::orbit_view(R, system) is equivalent to
    // ranges::views::orbit(R, system), which is also views::orbit(R,
    // system).

    // Let us first define the ranges::orbit_view template. The R
    // (base view) parameter is checked to be an input range and a
    // view. We use inheritence from view_interface to implement
    // things that can be deduced automatically from the definitions
    // we make in orbit_view (e.g size, data, ...).
    // We also use our concepts to test that R provides actual commands.

    /**
     * This range builds up the orbit of a dynamical system from its
     * current state.
     */
    template<std::ranges::input_range R,
	     concepts::system SYSTEM>
    requires std::ranges::view<R> &&
    concepts::command_iterator<std::ranges::iterator_t<R>, typename SYSTEM::command_type>
    class orbit_view : public std::ranges::view_interface<orbit_view<R, SYSTEM>> {
    private:
      // from is the range we adapt to become an orbit.
      R from {};

      // This is the iterator enableing an iteration on base.
      std::ranges::iterator_t<R> it {std::begin(from)};

      // This is the data spécific to our orbit range, a System
      // here. As we need our range to be default constructible, this
      // cannot be a reference, so we use a pointer.
      SYSTEM* system = nullptr;

    public:

      // The default constructor applies our default attribute
      // initialization.
      orbit_view() = default;

      // This is the constructor we actually need.
      orbit_view(R from, SYSTEM& system) : from(from), it(std::begin(from)), system(&system) {}

      // Two (required ?) getters for getting the adapted range.
      constexpr R base() const & {return from;}
      constexpr R base() &&      {return std::move(from);}

      // Here are the bounds of the range. Here, we use our terminal
      // iterator type iterators::terminal_t as a sentinel.
      constexpr auto begin() const {
	return iterators::orbit<SYSTEM,
				std::ranges::iterator_t<R>,
				std::ranges::sentinel_t<R>>(*system, from.begin(), from.end());
      }

      constexpr auto end()   const {return iterators::terminal;}
    };
    
    // This enables template argument guessing. It only consists of a
    // headear declaration ? Typing calls the right constructor ?
    // It also apply all_t for compatibility with things like vectors, I guess.
    template<typename R, typename SYSTEM> orbit_view(R&&, SYSTEM&) -> orbit_view<std::ranges::views::all_t<R>, SYSTEM>;

    // Here are tricks for implementing the | operator robustly.
    namespace details {

      // This enables to store the system at construction (i.e. to
      // keep it in the closure) and the adaptation to an orbit is
      // made when this closure is called, as a function, on the range
      // to adapt.
      template<typename SYSTEM>
      struct orbit_range_adaptor_closure {
	SYSTEM& system;
	constexpr orbit_range_adaptor_closure(SYSTEM& system) : system(system) {}
	template <std::ranges::viewable_range R> constexpr auto operator()(R&& from) const {return orbit_view(std::forward<R>(from), system);}
      };

      // This is the adaptor class, that offers to function calls, one
      // for the direct setting of an orbit view, one for taking only
      // the argument in a closure.
      template<typename SYSTEM>
      struct orbit_range_adaptor {
	template<std::ranges::viewable_range R> constexpr auto operator()(R&& from, SYSTEM& system) {return orbit_view(std::forward<R>(from), system);}
	constexpr auto operator()(SYSTEM& system) {return orbit_range_adaptor_closure<SYSTEM>(system);}
      };

      // The point here, is that expression A|B must buid B(A). Our
      // orbit adaptor plays the role of B. So what should be passed
      // as B in A|B is something that must only take the system, and
      // be ready for receiving only A for builo,g a full orbit. This
      // why un A|B, B is a closure.
      template <std::ranges::viewable_range R, typename SYSTEM>
      constexpr auto operator | (R&& from, orbit_range_adaptor_closure<SYSTEM> const& closure) {return closure(std::forward<R>(from));}
    }
    
    namespace views {
      template<typename SYSTEM> auto orbit(SYSTEM& system) {return details::orbit_range_adaptor<SYSTEM>()(system);}
    }


    
    // ##############
    // #            #
    // # transition #
    // #            #
    // ##############


    /**
     * This gathers orbit points into transitions.
     */
    template<std::ranges::input_range R>
    requires std::ranges::view<R> &&
    concepts::orbit_iterator<std::ranges::iterator_t<R>>
    class transition_view : public std::ranges::view_interface<transition_view<R>> {
    private:
      R from {};
      std::ranges::iterator_t<R> it {std::begin(from)};

    public:

      transition_view() = default;
      transition_view(R from) : from(from), it(std::begin(from)) {}
      constexpr R base() const & {return from;}
      constexpr R base() &&      {return std::move(from);}
      constexpr auto begin() const {
	return iterators::transition<std::ranges::iterator_t<R>,
				     std::ranges::sentinel_t<R>>(from.begin(), from.end());
      }
      constexpr auto end()   const {return iterators::terminal;}
    };
    
    template<typename R> transition_view(R&&) -> transition_view<std::ranges::views::all_t<R>>;

    namespace details {
      struct transition_range_adaptor_closure {
	constexpr transition_range_adaptor_closure() {}
	template <std::ranges::viewable_range R> constexpr auto operator()(R&& from) const {return transition_view(std::forward<R>(from));}
      };
      
      struct transition_range_adaptor {
	template<std::ranges::viewable_range R> constexpr auto operator()(R&& from) {return transition_view(std::forward<R>(from));}
	constexpr auto operator()() {return transition_range_adaptor_closure();}
      };
      template <std::ranges::viewable_range R>
      constexpr auto operator | (R&& from, transition_range_adaptor_closure const& closure) {return closure(std::forward<R>(from));}
    }
    
    namespace views {
      constexpr auto transition = details::transition_range_adaptor()();
    }
    
  }

  

  
  namespace views = ranges::views;
}
