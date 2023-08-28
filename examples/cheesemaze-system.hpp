#pragma once // HFB: c'est pas standard stricto sensu... mais c'est mieux non ?


#include <tuple>
#include <random>

#include <gdyn.hpp>

// Simulator of the CheeseMaze, where the State can be observed.
// observation = state + reward
// command = Left/Right/Up/Down
namespace cheese_maze {

  struct Parameters {
    // amount of stochasticity in moving
    double mishap_proba {0.1};
  }; // struct Parameters


  enum class Cell : int {C1 = 0, C2, C3, C4, C5, C6, C7, C8, C9, C10 , C11};
  constexpr static int nbCell {static_cast<int>(Cell::C11)+1};
  
  static std::string to_string(const Cell& c ) {
    std::stringstream sbuf;
    sbuf << "C" << static_cast<int>(c)+1;
    return sbuf.str();
  }
  
  template<typename RANDOM_GENERATOR>
  Cell random_state(RANDOM_GENERATOR& gen) {
    auto rnd_int = std::uniform_int_distribution<int>(0, nbCell-1)(gen);
    return static_cast<Cell>(rnd_int);
  }
  
  enum class Dir : int {Left = 0, Right, Up, Down};
  constexpr static int nbDir {static_cast<int>(Dir::Down)+1};
  
  static std::string to_string (const Dir& d ) {
    switch (d) {
    case Dir::Left:
      return "L";
    case Dir::Right:
      return "R";
    case Dir::Up:
      return "U";
    default:
      return "D"; 
    }
  }

  // Random command
  template<typename RANDOM_GENERATOR>
  Dir random_command(RANDOM_GENERATOR& gen) {
    switch(std::uniform_int_distribution<int>(0, nbDir-1)(gen)) {
    case  0: return Dir::Left; break;
    case  1: return Dir::Right; break;
    case  2: return Dir::Up; break;
    default: return Dir::Down; break;
    }
  }

  template<typename RG>
  class Environment { 
  public:

    Parameters param; 
    RG& gen; 

    // Transition as a vector of vector of neighbors
    // BEWARE as Cell and Dir will be cast as int to get to results
    std::array<std::array<Cell, 4>, 11> neighbors {{
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
      }};

    // various spaces, as required by gdyn::specs::system.
    using observation_type = std::tuple<Cell, double>;
    using command_type     = Dir;
    using state_type       = Cell;


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

    Environment(const Parameters& params,
                RG& generator) : param(params), gen(generator) {}

    // This is required by the gdyn::specs::system concept.
    // This is for initializing the state of the system.
    Environment& operator=(const state_type& init_state) {
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
    // template<typename RANDOM_GENERATOR>
    // void operator()(command_type command, RANDOM_GENERATOR& gen) {
    void operator()(command_type command) {
      if(*this) { // If we are not in a terminal state

        auto prev_state = state;
        // deterministic transition
        state = neighbors[(static_cast<int>(prev_state))]
	  [(static_cast<int>(command))];

        // mishap ?
        auto proba = std::uniform_real_distribution<double>(0,1)(gen);
        if (proba < param.mishap_proba) {
          state = neighbors[(static_cast<int>(prev_state))]
            [(std::uniform_int_distribution(0, nbDir-1)(gen))];
        }

        // rewards for wall and cheese
        if (prev_state == state) {
          reward = reward - 1;
        }
      }
    }
  }; // class Environment

  template<typename RG>
  auto make_environment(const Parameters& params, RG& generator) {
    return Environment<RG>(params, generator);
  }

  inline std::ostream& operator<<(std::ostream& os, Dir c)
  {
    os << "act=" << to_string(c);
    return os;
  }
  
  inline std::ostream& operator<<(std::ostream& os, Cell s)
  {
    os << "state=" << to_string(s);
    return os;
  }

  inline void print_context(const std::string& msg,
			    Cell state,
			    double reward ) {
    std::cout << msg << ": "<< to_string(state) << ", " << std::setw(3) << reward << std::endl;
  }
} // namespace cheese_maze
