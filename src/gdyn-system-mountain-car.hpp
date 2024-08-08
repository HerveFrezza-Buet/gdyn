#pragma once

#include <algorithm>
#include <random>
#include <string>
#include <sstream>

namespace gdyn {
  namespace problem {
    namespace mountain_car {

struct parameters {
  double min_position = -1.2;
  double max_position = 0.6;
  double max_speed = 0.07;
  double goal_position = 0.5;
  double goal_velocity = 0;

  double start_position_min = -0.6;
  double start_position_max = -0.4;
  double start_velocity_min =  0.0;
  double start_velocity_max =  0.0;

  double force = 0.001;
  double gravity = 0.0025;

}; // struct parameters

struct state {
  double position;
  double velocity;

  operator std::string() const { // Cast to string
    return std::string("{pos=")+std::to_string(position)+", vel="+std::to_string(velocity)+"}";
  }
}; // struct state


inline std::ostream& operator<<(std::ostream& os, state s) {
  return os << static_cast<std::string>(s);
}

template<typename RANDOM_GENERATOR>
state random_state(RANDOM_GENERATOR& gen, const parameters& param) {
  state res;
  double pos_range = param.start_position_max - param.start_position_min;
  res.position = (std::uniform_real_distribution<double>(0,1)(gen) * 2.0 - 1.0) * pos_range + param.start_position_min;

  double vel_range = param.start_velocity_max - param.start_velocity_min;
  res.velocity = (std::uniform_real_distribution<double>(0,1)(gen) * 2.0 - 1.0) * vel_range + param.start_velocity_min;

  return res;
}

// ********************************************************************* direction
enum class acceleration : std::size_t {Left = 0, None = 1, Right = 2};
static std::string to_string(const acceleration& d) {
  if (d == acceleration::Left) {
    return "Left";
  }
  else if (d == acceleration::None) {
    return "None";
  }
  else {
    return "Right";
  }
}

inline std::ostream& operator<<(std::ostream& os, acceleration action) {
  return os << "action='" << to_string(action) << "'";
}

template<typename RANDOM_GENERATOR>
acceleration random_command(RANDOM_GENERATOR& gen) {
  switch(std::uniform_int_distribution<int>(0, 2)(gen)) {
  case  0: return acceleration::Left; break;
  case  1: return acceleration::None; break;
  default: return acceleration::Right; break;
  }
}

class system {

public:

  parameters param;

  // This is required by the gdyn::specs::system concept.
  using observation_type = state;
  using command_type     = acceleration;
  using state_type       = state;
  using report_type      = double;

private:

  state_type  _state  {0.0, 0.0}; // state is already the type name of states.
  double      reward {0};
  bool        terminated {false};

  // Reward of -1 until goal reached
  void compute_reward() {
    terminated = false;
    auto p = param;

    reward = -1.0;
    if (_state.position >= p.goal_position and _state.velocity >= p.goal_velocity)
      terminated = true;
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

    auto vel = _state.velocity;
    auto pos = _state.position;

    auto p = param;
    double force = p.force;

    if (command == acceleration::Left) {
      force *= -1.0;
    }
    else if (command == acceleration::None) {
      force = 0.0;
    }
    vel += force + std::cos(3.0 * _state.position) * (- p.gravity);
    vel = std::clamp( vel, -p.max_speed, p.max_speed );

    pos += vel;
    pos = std::clamp( pos, p.min_position, p.max_position );

    // stop at min_pos
    if (pos <= p.min_position and vel < 0.0)
      vel = 0.0;

    _state.position = pos;
    _state.velocity = vel;

    compute_reward(); // and 'terminated'
    return reward;
  }
}; // class gdyn::problem::mountain_car::system

  auto make(const parameters& params = parameters{}) {
	return system(params);
  }
    } // namespace mountain_car
  } // namespace problem
} // namespace gdyn
