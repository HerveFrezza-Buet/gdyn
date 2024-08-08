#pragma once

// A very simple WxH grid world.
// The state is the number of the cell (0 to WxH-1).
// The actions are North, South, East, West
// The reward is 1 in cell GOAL_STATE, -1 if bumping into wall

// This class fits the gdyn::concepts::system

#include <iostream>
#include <random>
#include <tuple>

namespace gdyn {
namespace problem {
namespace grid_world {

enum class dir : char { North = 'N', South = 'S', West = 'W', East = 'E' };

template <unsigned int W, unsigned int H, unsigned int GOAL_STATE>
struct system {
  constexpr static unsigned int NB_STATES{W * H};

  // required by gdyn::concept::system
  using observation_type = unsigned int;
  using command_type = dir;
  using state_type = unsigned int;
  using report_type = double;

  template <typename RANDOM_GENERATOR>
  static state_type random_state(RANDOM_GENERATOR &gen) {
    return std::uniform_int_distribution<state_type>(0, (NB_STATES - 1))(gen);
  }

  static std::tuple<unsigned int, unsigned int> position(state_type s) {
    return {s % W, s / W };
  }

  state_type state{0};
  double reward{0.0};

  void compute_reward() {
    reward = 0.0;
    if (state == GOAL_STATE)
      reward = 1;
  }

  // This is required by the gdyn::specs::system concept.
  // This is for initializing the state of the system.
  system &operator=(const state_type &init_state) {
    state = init_state;
    reward = 0.0;
    if (state == GOAL_STATE)
      reward = 1.0;
    return *this;
  }

  // This is required by the gdyn::spec::system concept.
  // This returns the obsrvation corresponding to the system's state.
  observation_type operator*() const { return state; }

  // This is required by the gdyn::specs::system concept.
  // This it true if the system is not in a terminal state.
  operator bool() const { return state != GOAL_STATE; }

  // This is required by the gdyn::specs::system concept.
  // This performs a state transition.
  report_type operator()(command_type command) {
    reward = 0.0;
    switch (command) {
    case dir::North:
      if (state < W)
        reward = -1.0;
      else
        state -= W;
      break;
    case dir::South:
      if (state >= NB_STATES - W)
        reward = -1.0;
      else
        state += W;
      break;
    case dir::West:
      if (state % W == 0)
        reward = -1.0;
      else
        state -= 1;
      break;
    case dir::East:
      if (state % W == W - 1)
        reward = -1.0;
      else
        state += 1;
      break;
    }

    if (state == GOAL_STATE)
      reward += 1.0;

    return reward;
  }
}; // struct system

  template <typename RANDOM_GENERATOR>
  static dir random_command(RANDOM_GENERATOR &gen) {
    switch (std::uniform_int_distribution<int>(0, 3)(gen)) {
    case 0:
      return dir::North;
      break;
    case 1:
      return dir::South;
      break;
    case 2:
      return dir::West;
      break;
    default:
      return dir::East;
      break;
    }
  }

template <unsigned int W, unsigned int H, unsigned int GOAL_STATE>
system<W, H, GOAL_STATE> make() {return {};}

inline std::ostream &operator<<(std::ostream &os, dir action) {
  switch (action) {
  case dir::North:
    os << "North";
    break;
  case dir::South:
    os << "South";
    break;
  case dir::West:
    os << "West";
    break;
  case dir::East:
    os << "East";
    break;
  }
  return os;
}
} // namespace grid_world
} // namespace problem
} // namespace gdyn
