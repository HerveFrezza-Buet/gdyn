#include <iostream>
#include <random>
#include <tuple>
#include <iomanip>

#include <gdyn.hpp>
#include "bonobo-system.hpp"

// This is a controller. It takes the observation of the current
// system and build a command from it.
auto control_policy(const Bonobo::observation_type& observation) {
  switch(observation[5]) { // we take the last letter of the word.
  case 'B': return Bonobo::letter::B;
  case 'O': return Bonobo::letter::O;
  default : return Bonobo::letter::N;
  }
}

// Let us check that it fits the appropriate concept.
static_assert(gdyn::concepts::controller<decltype(control_policy), Bonobo::observation_type, Bonobo::command_type>);

int main(int argc, char* argv[]) {
  std::random_device rd;
  std::mt19937 gen(rd());

  Bonobo simulator;
  unsigned int step;
    
  simulator = Bonobo::random_state(gen); // We set the state
  auto state = *simulator;     // We get the observation.
  print_start(state);
  
  step = 1;
  for(auto [observation, action, report] 
	: gdyn::ranges::controller(simulator, control_policy)  // We generate commands from a controller on the current state.
	| gdyn::views::orbit(simulator)                        // It feeds an orbit.
	| std::views::take(20))                                // We take at most 20 orbit points.
    print_orbit_point(observation, action, report, step);
  
  state = *simulator; // We get the observation.
  print_final(state);
  
  return 0;
}
