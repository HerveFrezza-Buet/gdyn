#pragma once

#include <string>
#include <array>
#include <tuple>
#include <random>
#include <stdexcept>
#include <iostream>
#include <iomanip>

#include <gdyn.hpp>

// Let us define a simulator. Ths state is a 6 letter word made of the
// letters 'B', 'O', 'N' only.  A transition consists in putting one
// of these letter in the front of the word (thus popping the last
// letter). Observation is the word itslef, and an associated
// reward. This reward is -10 if the word is a palindrom, 100 if the
// word is 'BONOBO', 0 otherwise.
// A  palindrom is a terminal state of the simulation.

// This class fits the ds::specs::Simulator concept, and adds some
// usefull features, specific to the BONOBO simulation.

#define bonoboPALINDROM_REWARD   -10
#define bonoboBONOBO_REWARD      100
class Bonobo {
public:
  
  enum class letter : char {B = 'B', O = 'O', N = 'N'};

  // This is required by the gdyn::specs::system concept.
  using observation_type = std::tuple<std::string, double>;
  using command_type     = letter;
  using state_type       = std::array<command_type, 6>;

  static state_type to_state(const std::string& s) {
    state_type res;
    if(s.size() != 6) throw std::logic_error(s + " invalid argument for 'to_state'");
    auto out = res.begin();
    for(auto c : s) {
      switch(c) {
      case 'B': *out++ = letter::B; break;
      case 'O': *out++ = letter::O; break;
      case 'N': *out++ = letter::N; break;
      default:
	throw std::logic_error(s + " invalid argument for 'to_state'");
      }
    }
    return res;
  }

  static std::string to_string(const state_type& p) {
    std::string res(6, ' ');
    auto src = p.begin();
    for(auto dst = res.begin(); dst != res.end();) *dst++ = static_cast<char>(*src++);
    return res;
  }

  template<typename RANDOM_GENERATOR>
  static command_type random_command(RANDOM_GENERATOR& gen) {
    switch(std::uniform_int_distribution<int>(0, 2)(gen)) {
    case  0: return letter::B; break;
    case  1: return letter::O; break;
    default: return letter::N; break;
    }
  }

  template<typename RANDOM_GENERATOR>
  static state_type random_state(RANDOM_GENERATOR& gen) {
    state_type res;
    for(auto& cmd : res) cmd = random_command(gen);
    return res;
  }
  
private:
  
  state_type  state  {letter::B, letter::O, letter::N, letter::B, letter::O, letter::N};
  double      reward {0};

  void compute_reward() {
    if(state[0] == state[5] &&  state[1] == state[4] && state[2] == state[3])
      reward = bonoboPALINDROM_REWARD;
    else if(state[0]    == letter::B
	    && state[1] == letter::O
	    && state[2] == letter::N
	    && state[3] == letter::O
	    && state[4] == letter::B
	    && state[5] == letter::O)
      reward = bonoboBONOBO_REWARD;
    else
      reward = 0;
  }

public:
  

  // This is required by the gdyn::specs::system concept.
  // This is for initializing the state of the system.
  Bonobo& operator=(const state_type& init_state) {
    state = init_state;
    compute_reward();
    return *this;
  }
  
  // This is required by the gdyn::specs::system concept.
  // This returns the obsrvation corresponding to the system's state.
  observation_type operator*() const {
    return {to_string(state), reward};
  }

  // This is required by the gdyn::specs::system concept.
  // This it true if the system is not in a terminal state.
  operator bool() const {
    return reward != bonoboPALINDROM_REWARD;
  }

  
  // This is required by the gdyn::specs::system concept.
  // This performs a state transition.
  void operator()(command_type command) {
    if(*this) { // If we are not in a terminal state
      auto dst = state.rbegin();
      auto src = dst + 1;
      while(src != state.rend()) *(dst++) = *(src++);
      *dst = command;
      compute_reward();
    }
  }
  
};

// Let us check the ds concept.
static_assert(gdyn::specs::system<Bonobo>);

// Here are some usefull prints

inline std::ostream& operator<<(std::ostream& os, Bonobo::command_type action) {
  return os << "action='" << static_cast<char>(action) << "'";
}


void print_start(const std::string& state, double reward) {
  std::cout << "Starting : " << state << ", " << std::setw(3) << reward << std::endl;
}

void print_orbit_point(const Bonobo::observation_type& observation, const std::optional<Bonobo::command_type>& action, unsigned int& step) {
  std::cout << std::setw(8) << (step++) << " : "
	    << std::get<0>(observation) << ", "
	    << std::setw(3) << std::get<1>(observation);
  if(action)
    std::cout << " -> " << *action;
  std::cout << std::endl;
}

void print_transition(const gdyn::transition<Bonobo::observation_type, Bonobo::command_type> t, unsigned int& step) {
  std::cout << std::setw(8) << (step++) << " : ["
	    << std::get<0>(t.observation) << ", "
	    << std::setw(3) << std::get<1>(t.observation)
	    << "] --> " << t.command << " --> ["
	    << std::get<0>(t.next_observation) << ", "
	    << std::setw(3) << std::get<1>(t.next_observation)
	    << ']';
  if(t.next_command)
    std::cout << " ( -> " << *(t.next_command) << ')';
  std::cout << std::endl;
}


void print_terminal(const std::string& state, double reward) {
  std::cout << "Terminal : " << state << ", " << std::setw(3) << reward << std::endl;
}
