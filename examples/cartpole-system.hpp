#pragma once

#include <string>
#include <array>
#include <tuple>
#include <random>
#include <stdexcept>
#include <iostream>
#include <iomanip>

#include <numbers> // pi_v
#include <gdyn.hpp>

// Simulator of the cartpole.
// I'd like to add a `delta_t` parameter to the `step` function.

// This class fits the ds::specs::Simulator concept, and adds some
// usefull features, specific to the BONOBO simulation.


class Cartpole {

public:
  // [Q] constants ?
  // Physics
  const double gravity {9.81};
  const double mass_cart {1.0};
  const double mass_pole {0.1};
  const double mass_total {mass_cart + mass_pole};
  const double length_halfpole {0.5};
  const double lm_pole {mass_pole * length_halfpole};
  const double force_mag {10.0};
  static constexpr double delta_default {0.02};     // delta_time for update, in seconds

  // Termination
  static constexpr double theta_threshold_rad {12.0 * 2.0 * std::numbers::pi / 360.0};
  static constexpr double x_threshold {2.4};


  struct state_s {
    double x;
    double x_dot;
    double theta;
    double theta_dot;
  };
  static std::string to_string(const state_s& s) {
    std::stringstream sbuf;
    sbuf << "x=" << s.x << " x_dot=" << s.x_dot;
    sbuf << " theta=" << s.theta << " theta_dot=" << s.theta_dot;
    return sbuf.str();
  }

  enum class direction : int {L = 0, R = 1};
  static std::string to_string(const direction& d) {
    if (d == direction::L) {
      return "L";
    }
    else {
      return "R";
    }
  }

  // This is required by the gdyn::specs::system concept.
  using observation_type = std::tuple<state_s, double>;
  using command_type     = direction;
  using state_type       = state_s;

  template<typename RANDOM_GENERATOR>
  static command_type random_command(RANDOM_GENERATOR& gen) {
    switch(std::uniform_int_distribution<int>(0, 1)(gen)) {
    case  0: return direction::L; break;
    default: return direction::R; break;
    }
  }

  template<typename RANDOM_GENERATOR>
  static state_type random_state(RANDOM_GENERATOR& gen) {
    state_type res;
    res.x = (std::uniform_real_distribution<double>(0,1)(gen) * 2.0 - 1.0) * Cartpole::x_threshold;
    res.x_dot = 0.0;
    res.theta = (std::uniform_real_distribution<double>(0,1)(gen) * 2.0 - 1.0) * Cartpole::theta_threshold_rad;
    res.theta_dot = 0.0;
    return res;
  }
  
private:
  
  state_type  state  {0.0, 0.0, 0.0, 0.0};
  double      reward {0};
  bool        terminated {false};
  bool        just_terminated {false};

  // Reward of 1 until 1 update after going out of bounds
  void compute_reward() {
    if (not terminated) {
      reward = 1.0;
    }
    else if (just_terminated) {
      just_terminated = true;
      reward = 1.0;
    }
    else {
      // should not be called -> reset ?
      reward = 0.0;
    }
  }

public:
  

  // This is required by the gdyn::specs::system concept.
  // This is for initializing the state of the system.
  Cartpole& operator=(const state_type& init_state) {
    state = init_state;
    // TODO reward = compute_reward();
    compute_reward();
    return *this;
  }
  
  // This is required by the gdyn::spec::system concept.
  // This returns the obsrvation corresponding to the system's state.
  observation_type operator*() const {
    return {state, reward};
  }

  // This is required by the gdyn::specs::system concept.
  // This it true if the system is not in a terminal state.
  operator bool() const {
    return terminated;
  }

  
  // This is required by the gdyn::specs::system concept.
  // This performs a state transition.
  void operator()(command_type command, double delta_time=Cartpole::delta_default) {
  // TODO void operator()(command_type command) {
    // TODO double delta_time = delta_default;
    double force {force_mag};
    if (command == direction::L) {
      force *= -1.0;
    }

    auto theta = state.theta;
    double costheta = std::cos(theta);
    double sintheta = std::sin(theta);
    double temp = (force + lm_pole * std::pow(state.theta_dot, 2) * sintheta) / mass_total;
    double theta_acc = (gravity * sintheta - costheta * temp) /
      (length_halfpole * (4.0 / 3.0 - mass_pole * std::pow(costheta, 2)) / mass_total);
    double x_acc = temp - lm_pole * theta_acc * costheta / mass_total;

    state.x += delta_time * state.x_dot;
    state.x_dot += delta_time * x_acc;
    state.theta += delta_time * state.theta_dot;
    state.theta_dot += delta_time * theta_acc;

    if (state.x < - x_threshold or state.x > x_threshold
        or state.theta < - theta_threshold_rad or state.theta > theta_threshold_rad) {
      terminated = true;
    }
    compute_reward();
  }

};

// Let us check the ds concept.
static_assert(gdyn::specs::system<Cartpole>);

// Here are some usefull prints

inline std::ostream& operator<<(std::ostream& os, Cartpole::command_type action) {
  return os << "action='" << Cartpole::to_string(action) << "'";
}

void print_context(const std::string& msg,
                   Cartpole::state_type& state,
                   double reward ) {
  std::cout << msg << ": "<< Cartpole::to_string(state) << ", " << std::setw(3) << reward << std::endl;
}

// void print_orbit_point(const Cartpole::observation_type& observation, const std::optional<Cartpole::command_type>& action, unsigned int& step) {
//   std::cout << std::setw(8) << (step++) << " : "
// 	    << std::get<0>(observation) << ", "
// 	    << std::setw(3) << std::get<1>(observation);
//   if(action)
//     std::cout << " -> " << *action;
//   std::cout << std::endl;
// }

// void print_transition(const gdyn::transition<Cartpole::observation_type, Cartpole::command_type> t, unsigned int& step) {
//   std::cout << std::setw(8) << (step++) << " : ["
// 	    << std::get<0>(t.observation) << ", "
// 	    << std::setw(3) << std::get<1>(t.observation)
// 	    << "] --> " << t.command << " --> ["
// 	    << std::get<0>(t.next_observation) << ", "
// 	    << std::setw(3) << std::get<1>(t.next_observation)
// 	    << ']';
//   if(t.next_command)
//     std::cout << " ( -> " << *(t.next_command) << ')';
//   std::cout << std::endl;
// }
