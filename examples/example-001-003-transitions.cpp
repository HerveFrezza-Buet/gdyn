#include <iostream>
#include <random>
#include <tuple>
#include <iomanip>
#include <iterator>
#include <vector>
#include <algorithm>

#include <gdyn.hpp>
#include "bonobo-system.hpp"


#define NB_ORBITS 50

int main(int argc, char* argv[]) {
  std::random_device rd;
  std::mt19937 gen(rd());

  Bonobo simulator;
  unsigned int step;
  
  simulator = Bonobo::random_state(gen); // We set the state
  auto state = *simulator; // We get the observation.
  print_start(state);

  step = 1;
  for(const auto& t 
	: gdyn::ranges::tick([&gen](){return Bonobo::random_command(gen);}) // We generate random command.
	| gdyn::views::orbit(simulator)                                     // They feed an orbit.
	| std::views::take(20)                                              // We take ato modt 20 orbit points.
	| gdyn::views::transition)                                          // We gather them into transitions.
    print_transition(t, step);

  state = *simulator; // We get the observation.
  print_terminal(state);
  
  // With ranges, we can easily set up a dataset from a set of successive orbits.
  std::cout << std::endl
	    << "Let us collect a dataset from " << NB_ORBITS << " orbits." << std::endl;
  
  std::vector<gdyn::transition<Bonobo::observation_type, Bonobo::command_type, Bonobo::report_type>> transitions_dataset;
  for(unsigned int orbit = 0; orbit < NB_ORBITS; ++orbit) {
    simulator = Bonobo::random_state(gen);
    std::ranges::copy(gdyn::ranges::tick([&gen](){return Bonobo::random_command(gen);}) // We generate random command.
		      | gdyn::views::orbit(simulator)                                   // They feed an orbit until system termination.
		      | gdyn::views::transition,                                        // We gather orbit points into transitions.
		      std::back_inserter(transitions_dataset));                         // We store the result in the dataset.
  }

  std::cout << "  we have got " << transitions_dataset.size() << " transitions." << std::endl;
  std::cout << std::endl
	    << std::endl
	    << "Let us show the terminal ones" << std::endl
	    << std::endl;

  // Let us use the filter view to display terminal transitions in our dataset.
  step = 1;
  for(auto& t
	: transitions_dataset
	| std::views::filter([](auto t){return t.is_terminal();}))
    print_transition(t, step);
  
  return 0;
}
