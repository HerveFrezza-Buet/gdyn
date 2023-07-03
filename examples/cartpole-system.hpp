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
// A companion struct Parameters hold the various parameters/const of the Simulator.

// This class fits the gdyn::specs::Simulator concept.

// **************************************************************** Parameters
struct Parameters {

  // Physics
  double gravity {9.81};
  double mass_cart {1.0};
  double mass_pole {0.1};
  double mass_total {mass_cart + mass_pole};
  double length_halfpole {0.5};
  double lm_pole {mass_pole * length_halfpole};
  double force_mag {10.0};

  // Simulation engine
  double delta_time {0.02};     // delta_time for update, in seconds

  // Termination
  double theta_threshold_rad {12.0 * 2.0 * std::numbers::pi / 360.0};
  double x_threshold {2.4};

  // Random State generation
  double range_x {0.5};
  double range_theta_rad {theta_threshold_rad / 2.0};

}; // struct Parameters

// ***************************************************************************
// ****************************************************************** CartPole
// ***************************************************************************
class Cartpole {

public:

  Parameters param;

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

  // for command
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
  static state_type random_state(RANDOM_GENERATOR& gen, const Parameters& param) {
    state_type res;
    res.x = (std::uniform_real_distribution<double>(0,1)(gen) * 2.0 - 1.0) * param.range_x;
    res.x_dot = 0.0;
    res.theta = (std::uniform_real_distribution<double>(0,1)(gen) * 2.0 - 1.0) * param.range_theta_rad;
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
  // Constructor initialize Parameters
  Cartpole()  : param(Parameters{}) {}

  // This is required by the gdyn::specs::system concept.
  // This is for initializing the state of the system.
  Cartpole& operator=(const state_type& init_state) {
    state = init_state;
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
  void operator()(command_type command) {

    auto p = param;
    double force {p.force_mag};
    if (command == direction::L) {
      force *= -1.0;
    }

    auto theta = state.theta;
    double costheta = std::cos(theta);
    double sintheta = std::sin(theta);
    double temp = (force + p.lm_pole * std::pow(state.theta_dot, 2) * sintheta) / p.mass_total;
    double theta_acc = (p.gravity * sintheta - costheta * temp) /
      (p.length_halfpole * (4.0 / 3.0 - p.mass_pole * std::pow(costheta, 2)) / p.mass_total);
    double x_acc = temp - p.lm_pole * theta_acc * costheta / p.mass_total;

    state.x += p.delta_time * state.x_dot;
    state.x_dot += p.delta_time * x_acc;
    state.theta += p.delta_time * state.theta_dot;
    state.theta_dot += p.delta_time * theta_acc;

    if (state.x < - p.x_threshold or state.x > p.x_threshold
        or state.theta < - p.theta_threshold_rad or state.theta > p.theta_threshold_rad) {
      terminated = true;
    }
    compute_reward();
  }

}; // class CartPole

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
