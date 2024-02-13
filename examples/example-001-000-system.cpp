#include <iostream>
#include <random>
#include <string>
#include <iomanip>

#include <gdyn.hpp>
#include "bonobo-system.hpp"

int main(int argc, char* argv[]) {
  std::random_device rd;
  std::mt19937 gen(rd());

  Bonobo simulator;
  
  simulator = Bonobo::random_state(gen); // We set the state.
  auto state = *simulator;     // We get the observation.
  print_start(state);

  // Let us apply a command to the system. Here we well apply a random
  // one. We get the reward (i.e. the transition report).
  auto reward = simulator(Bonobo::random_command(gen));
  state = *simulator; // We get the new observation.
  print_current(state, reward);
  std::cout << std::endl;

  // We can use command sources in order to feed the system. The pulse
  // range enables this. It repeats calls of a function f() that produces
  // a command. Here, f is a lambda calling random_command.
  std::cout << "Random command source" << std::endl;
  for(auto command
	: gdyn::views::pulse([&gen](){return Bonobo::random_command(gen);})
	| std::views::take(20))
    std::cout << command << std::endl;

  // The command source can be obtained by a policy, i.e. a function
  // that chooses the command according to the system state.
  std::cout << std::endl;
  std::cout << "Policy command source" << std::endl;
  auto policy = [](const std::string state) {
    // We return the command correponding to the last letter
    return static_cast<Bonobo::letter>(state[5]);
  };
  state = *simulator;
  print_start(state);
  for(auto command
	: gdyn::views::pulse([&simulator, &policy](){return policy(*simulator);})
	| std::views::take(20)) {
    reward = simulator(command); // We apply the command to the system to trigger a state transition.
    state = *simulator;
    std::cout << command << " => " << state << " (" << reward << ")." << std::endl;
  }
  print_final(state);

  // What we have just done is running an orbit/trajectory of the dynamical system. See next example.
  
  return 0;
  
}
