#include <iostream>
#include <random>
#include <tuple>
#include <iomanip>

#include <gdyn.hpp>
#include "bonobo-system.hpp"


int main(int argc, char* argv[]) {
  std::random_device rd;
  std::mt19937 gen(rd());

  Bonobo simulator;
  unsigned int step;
  
  // This illustrates the use of orbit view.

  // The tick(f) range
  // provides values iteratively, each one being obtained from a call
  // f(). This is very usefull for feeding the orbit with events.
  
  simulator = Bonobo::random_state(gen); // We set the state.
  auto state = *simulator;     // We get the observation.
  print_start(state);
  
  step = 1;
  for(auto [observation, action, report] // report is the reward here.
	: gdyn::ranges::tick([&gen](){return Bonobo::random_command(gen);}) // This is a source of random commands...
	| gdyn::views::orbit(simulator)                                     // ... that feeds an orbit of the stystem...
	| std::views::take(20))                                             // ... which will be interrupted after 20 steps at most.
    print_orbit_point(observation, action, report, step);
  
  state = *simulator; // We get the observation.
  print_terminal(state);
  
  return 0;
}
