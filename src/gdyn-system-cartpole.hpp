#pragma once

#include <string>
#include <array>
#include <tuple>
#include <random>
#include <stdexcept>
#include <iostream>
#include <iomanip>

#include <numbers> // pi_v

namespace gdyn {
  namespace problem {
    /**
       Simulator of the cartpole.
       This class fits the gdyn::specs::Simulator concept.
       State = {x, x_dot, theta, theta_dot}
       Command = Dir (enum L, R)
       Observation = State
       Report = double

       A companion struct Parameters holds the various parameters/const of the Simulator.
    */

    namespace cartpole {
      // **************************************************************** Parameters
      struct parameters {

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

      }; // struct parameters

      // ******************************************************************* State
      struct state {
	double x;
	double x_dot;
	double theta;
	double theta_dot;
      };
    
      inline std::string to_string(const state& s) {
	std::stringstream sbuf;
	sbuf << "x=" << s.x << " x_dot=" << s.x_dot;
	sbuf << " theta=" << s.theta << " theta_dot=" << s.theta_dot;
	return sbuf.str();
      }
      
      inline std::ostream& operator<<(std::ostream& os, state s) {
	return os << "state='" << to_string(s) << "'";
      }
      
      template<typename RANDOM_GENERATOR>
      state random_state(RANDOM_GENERATOR& gen, const parameters& param) {
	state res;
	res.x = (std::uniform_real_distribution<double>(0,1)(gen) * 2.0 - 1.0) * param.range_x;
	res.x_dot = 0.0;
	res.theta = (std::uniform_real_distribution<double>(0,1)(gen) * 2.0 - 1.0) * param.range_theta_rad;
	res.theta_dot = 0.0;
	return res;
      }

      // ********************************************************************* direction
      enum class direction : std::size_t {Left = 0, Right = 1};
      static std::string to_string(const direction& d) {
	if (d == direction::Left) {
	  return "Left";
	}
	else {
	  return "Right";
	}
      }

      inline std::ostream& operator<<(std::ostream& os, direction action) {
	return os << "action='" << to_string(action) << "'";
      }
      
      template<typename RANDOM_GENERATOR>
      direction random_command(RANDOM_GENERATOR& gen) {
	switch(std::uniform_int_distribution<int>(0, 1)(gen)) {
	case  0: return direction::Left; break;
	default: return direction::Right; break;
	}
      }

      // ***************************************************************************
      // ****************************************************************** CartPole
      // ***************************************************************************
      class system {

      public:

	parameters param;

	// This is required by the gdyn::specs::system concept.
	using observation_type = state;
	using command_type     = direction;
	using state_type       = state;
	using report_type      = double;

      private:
  
	state_type  _state  {0.0, 0.0, 0.0, 0.0}; // state is already the type name of states.
	double      reward {0};
	bool        terminated {false};
	bool        just_terminated {false};

	// Reward of 1 until 1 update after going out of bounds
	void compute_reward() {
	  terminated = false;
	  auto p = param;
	  if (_state.x < - p.x_threshold or _state.x > p.x_threshold
	      or _state.theta < - p.theta_threshold_rad or _state.theta > p.theta_threshold_rad) {
	    terminated = true;
	  }

	  if (not terminated) {
	    reward = 1.0;
	  }
	  // in the step just falling
	  else if (not just_terminated) {
	    just_terminated = true;
	    reward = 1.0;
	  }
	  // has already fallen
	  else {
	    // should not be called -> reset ?
	    reward = 0.0;
	  }
	}

      public:
	// Constructor initialize parameters
	system(const parameters& params)  : param(params) {}

	// This is required by the gdyn::specs::system concept.
	// This is for initializing the state of the system.
	system& operator=(const state_type& init_state) {
	  _state = init_state;
	  compute_reward();
	  return *this;
	}
  
	// This is required by the gdyn::spec::system concept.
	// This returns the obsrvation corresponding to the system's state.
	observation_type operator*() const {
	  return _state;
	}

	// This is required by the gdyn::specs::system concept.
	// This it true if the system is not in a terminal state.
	operator bool() const {
	  return !terminated;
	}

  
	// This is required by the gdyn::specs::system concept.
	// This performs a state transition.
	report_type operator()(command_type command) {

	  auto p = param;
	  double force {p.force_mag};
	  if (command == direction::Left) {
	    force *= -1.0;
	  }

	  auto theta = _state.theta;
	  double costheta = std::cos(theta);
	  double sintheta = std::sin(theta);
	  double temp = (force + p.lm_pole * (_state.theta_dot * _state.theta_dot) * sintheta) / p.mass_total;
	  double theta_acc = (p.gravity * sintheta - costheta * temp) /
	    (p.length_halfpole * (4.0 / 3.0 - p.mass_pole * (costheta * costheta)) / p.mass_total);
	  double x_acc = temp - p.lm_pole * theta_acc * costheta / p.mass_total;

	  _state.x += p.delta_time * _state.x_dot;
	  _state.x_dot += p.delta_time * x_acc;
	  _state.theta += p.delta_time * _state.theta_dot;
	  _state.theta_dot += p.delta_time * theta_acc;

	  compute_reward(); // and 'terminated'
	  return reward;
	}

      }; // class gdyn::problem::cartpole::system

      auto make(const parameters& params = parameters{}) {
	return system(params);
      }

    } // namespace cartpole

  } // namespace problem
} // namespace gdyn
