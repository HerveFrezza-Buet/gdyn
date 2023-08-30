#pragma once // HFB: c'est pas standard stricto sensu... mais c'est mieux non ?


#include <tuple>
#include <random>

#include <gdyn.hpp>

/**
   Simulator of the CheeseMaze problem, implement gdyn::system and
   gdyn::transparent_system.
   State       = Cell : pos of the agent
   Command     = Dir : direction the agent wants to move
   Observation = Walls : local view of the walls around the agent
   Report      = double : the reward
   */
namespace cheese_maze {

  struct Parameters {
    // amount of stochasticity in moving
    double mishap_proba {0.1};
  }; // struct Parameters


  // ******************************************************************** Cell
  enum class Cell : int {C1 = 0, C2, C3, C4, C5, C6, C7, C8, C9, C10 , C11};
  constexpr static int nbCell {static_cast<int>(Cell::C11)+1};
  
  std::string to_string(const Cell& c ) {
    std::stringstream sbuf;
    sbuf << "C" << static_cast<int>(c)+1;
    return sbuf.str();
  }
  
  template<typename RANDOM_GENERATOR>
  Cell random_state(RANDOM_GENERATOR& gen) {
    auto rnd_int = std::uniform_int_distribution<int>(0, nbCell-1)(gen);
    return static_cast<Cell>(rnd_int);
  }

  // ********************************************************************* Dir
  enum class Dir : int {Left = 0, Right, Up, Down};
  constexpr static int nbDir {static_cast<int>(Dir::Down)+1};
  
  std::string to_string (const Dir& d ) {
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

  template<typename RANDOM_GENERATOR>
  Dir random_command(RANDOM_GENERATOR& gen) {
    switch(std::uniform_int_distribution<int>(0, nbDir-1)(gen)) {
    case  0: return Dir::Left; break;
    case  1: return Dir::Right; break;
    case  2: return Dir::Up; break;
    default: return Dir::Down; break;
    }
  }

  // ******************************************************************* Walls
  enum class Walls : int {bLUr=0, BlUr, blUr, blUR, bLuR, BLuR};
  constexpr static int nbWalls {static_cast<int>(Walls::BLuR)+1};

  static std::string to_string (const Walls& w) {
    switch (w) {
    case Walls::bLUr: return ".LU.";
    case Walls::BlUr: return "B.U.";
    case Walls::blUr: return "..U.";
    case Walls::blUR: return "..UR";
    case Walls::bLuR: return ".L.R";
    case Walls::BLuR:
    default:          return "BL.R";
    }
  }

  // *************************************************************************
  // ************************************************************* Environment
  // *************************************************************************
  template<typename RG>
  class Environment { 
  public:

    Parameters param; 
    RG& gen; 

    // Transition as a vector of vector of neighbors
    // BEWARE as Cell and Dir will be cast as int to get to results
    std::array<std::array<Cell, nbDir>, nbCell> neighbors {{
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

    // For each of the (b)ottom, (l)eft, (u)p and (r)righ wall,
    // - uppercase => Wall
    // - lowercase => no wall
    std::array<Walls, nbCell> local_view {
      Walls::bLUr,
      Walls::BlUr,
      Walls::blUr,
      Walls::BlUr,
      Walls::blUR,
      Walls::bLuR,
      Walls::bLuR,
      Walls::bLuR,
      Walls::BLuR,
      Walls::BLuR,
      Walls::BLuR,
    };
    
    // various spaces, as required by gdyn::specs::system.
    using observation_type = Walls;
    using command_type     = Dir;
    using state_type       = Cell;
    using report_type      = double;


  private:

    state_type internal_state {Cell::C1};
    report_type reward {0};

    // Does NOT take into account Bumping into walls
    void compute_reward() {
      if (internal_state == Cell::C10) {
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
      internal_state = init_state;
      compute_reward();
      return *this;
    }
    
    // This is required by the gdyn::specs::system concept.
    // This returns the obsrvation corresponding to the system's state.
    observation_type operator*() const {
      return local_view[static_cast<int>(internal_state)];
    }

    // This is required by the gdyn::specs::transparent_system concept.
    // This returs the 'true' state of the system
    state_type state() const {return internal_state;}

    // This is required by the gdyn::specs::system concept.
    // This it true if the system is not in a terminal state.
    operator bool() const {
      return internal_state != Cell::C10;
    }

    // This is required by the gdyn::specs::system concept.
    // This performs a state transition.
    // template<typename RANDOM_GENERATOR>
    // void operator()(command_type command, RANDOM_GENERATOR& gen) {
    report_type operator()(command_type command) {
      if(*this) { // If we are not in a terminal state

        auto prev_state = internal_state;
        // deterministic transition
        internal_state = neighbors[(static_cast<int>(prev_state))]
          [(static_cast<int>(command))];

        // mishap ?
        auto proba = std::uniform_real_distribution<double>(0,1)(gen);
        if (proba < param.mishap_proba) {
          internal_state = neighbors[(static_cast<int>(prev_state))]
            [(std::uniform_int_distribution(0, nbDir-1)(gen))];
        }

        compute_reward();
        // If we have bumped into a wall...
        if (prev_state == internal_state) {
          reward = reward - 1;
        }
      }
      return reward;
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
  
  inline std::ostream& operator<<(std::ostream& os, Walls w)
  {
    os << "local_view=" << to_string(w);
    return os;
  }

  template<typename THING>
  void print_context(const std::string& msg,
                     THING thing,
                     double reward ) {
    std::cout << msg << ": "<< thing << ", " << std::setw(3) << reward << std::endl;
  }

  template<typename THING>
  void print_orbit_point(THING obs_or_state,
                         const std::optional<Dir>& action,
                         const std::optional<double>& reward,
                         unsigned int& step) {
    std::cout << std::setw(8) << (step++) << " : "
              << "at " << obs_or_state;
    if(reward)
      std::cout << ", " << std::setw(3) << *reward << " received";
    if(action)
      std::cout << " -> " << *action;
    std::cout << std::endl;
}


} // namespace cheese_maze
