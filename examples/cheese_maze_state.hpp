#ifndef CHEESE_MAZE_STATE_H_
#define CHEESE_MAZE_STATE_H_

#include <tuple>
#include <random>

#include <gdyn.hpp>

// Simulator of the CheeseMaze, where the State can be observed.
// observation = state + reward
// command = Left/Right/Up/Down

struct Parameters {
  // amount of stochasticity in moving
  double mishap_proba {0.1};
}; // struct Parameters

class CheeseMazeState {

public:

  Parameters param;

  enum class Cell : int {C1 = 0, C2, C3, C4, C5, C6, C7, C8, C9, C10 , C11};
  constexpr static int nbCell {static_cast<int>(Cell::C11)+1};
  static std::string to_string( const Cell& c ) {
    std::stringstream sbuf;
    sbuf << "C" << static_cast<int>(c)+1;
    return sbuf.str();
  }
  enum class Dir : int {Left = 0, Right, Up, Down};
  constexpr static int nbDir {static_cast<int>(Dir::Down)+1};

  // Transition as a vector of vector of neighbors
  // BEWARE as Cell and Dir will be cast as int to get to results
  std::vector<std::vector<Cell>> neighbors = {
    {Cell::C1, Cell::C2, Cell::C1, Cell::C6},
    {Cell::C1, Cell::C3, Cell::C2, Cell::C2},
    {Cell::C2, Cell::C4, Cell::C3, Cell::C7},
    {Cell::C3, Cell::C5, Cell::C4, Cell::C4},
    {Cell::C4, Cell::C5, Cell::C5, Cell::C8},

    {Cell::C6, Cell::C6, Cell::C1, Cell::C9},
    {Cell::C7, Cell::C7, Cell::C3, Cell::C10},
    {Cell::C8, Cell::C8, Cell::C5, Cell::C11},

    {Cell::C9, Cell::C9, Cell::C6, Cell::C9},
    {Cell::C10, Cell::C10, Cell::C7, Cell::C10},
    {Cell::C11, Cell::C11, Cell::C8, Cell::C11},
  };

  // various spaces, as required by gdyn::specs::system.
  using observation_type = std::tuple<Cell, double>;
  using command_type     = Dir;
  using state_type       = Cell;

  // Random state
  template<typename RANDOM_GENERATOR>
  static command_type random_command(RANDOM_GENERATOR& gen) {
    switch(std::uniform_int_distribution<int>(0, nbDir-1)(gen)) {
    case  0: return Dir::Left; break;
    case  1: return Dir::Right; break;
    case  2: return Dir::Up; break;
    default: return Dir::Down; break;
    }
  }

  template<typename RANDOM_GENERATOR>
  static state_type random_state(RANDOM_GENERATOR& gen) {
    auto rnd_int = std::uniform_int_distribution<int>(0,CheeseMazeState::nbCell-1)(gen);
    state_type res = static_cast<state_type>(rnd_int);
    return res;
  }

private:

  state_type state {Cell::C1};
  double     reward {0};

  // Does NOT take into account Bumping into walls
  void compute_reward() {
    if (state == Cell::C10) {
      reward = 5;
    }
    else {
      reward = 0;
    }
  }

public:
  // Constructor initialize Parameters
  CheeseMazeState() : param(Parameters{}) {}

  // This is required by the gdyn::specs::system concept.
  // This is for initializing the state of the system.
  CheeseMazeState& operator=(const state_type& init_state) {
    state = init_state;
    compute_reward();
    return *this;
  }

  // This is required by the gdyn::specs::system concept.
  // This returns the obsrvation corresponding to the system's state.
  observation_type operator*() const {
    return {state, reward};
  }

  // This is required by the gdyn::specs::system concept.
  // This it true if the system is not in a terminal state.
  operator bool() const {
    return state != Cell::C10;
  }

  // This is required by the gdyn::specs::system concept.
  // This performs a state transition.
  template<typename RANDOM_GENERATOR>
  void operator()(command_type command, RANDOM_GENERATOR& gen) {
    if(*this) { // If we are not in a terminal state

      auto prev_state = state;
      // deterministic transition
      state = neighbors.at(static_cast<int>(prev_state))
        .at(static_cast<int>(command));

      // mishap ?
      auto proba = std::uniform_real_distribution<double>(0,1)(gen);
      if (proba < param.mishap_proba) {
        state = neighbors.at(static_cast<int>(prev_state))
          .at(std::uniform_int_distribution(0, nbDir-1)(gen));
      }

      // rewards for wall and cheese
      if (prev_state == state) {
        reward = reward - 1;
      }
    }
  }
}; // class CheeseMazeState

std::ostream& operator<<(std::ostream& os, const CheeseMazeState::command_type& c)
{
  os << "act=";
  switch (c) {
  case CheeseMazeState::Dir::Left:
    os << "L";
    break;
  case CheeseMazeState::Dir::Right:
    os << "R";
    break;
  case CheeseMazeState::Dir::Up:
    os << "U";
    break;
  case CheeseMazeState::Dir::Down:
    os << "D";
    break;
  default:
    os << "???";
  }

  return os;
}
std::ostream& operator<<(std::ostream& os, const CheeseMazeState::state_type& s)
{
  os << "state=" << CheeseMazeState::to_string(s);
  return os;
}

void print_context(const std::string& msg,
                   CheeseMazeState::state_type& state,
                   double reward ) {
  std::cout << msg << ": "<< state << ", " << std::setw(3) << reward << std::endl;
}
#endif // CHEESE_MAZE_STATE_H_
