#include <iostream>
#include <random>
#include <string>
#include <iomanip>

#include <gdyn.hpp>
#include "cartpole-system.hpp"

int main(int argc, char* argv[]) {
  std::random_device rd;
  std::mt19937 gen(rd());

  Cartpole simulator;
  
  simulator = Cartpole::random_state(gen); // We set the state.
  auto [state, reward] = *simulator;     // We get the observation.
  print_context("start", state, reward);

  // Let us apply a command to the system. Here we well apply a random
  // one.
  simulator(Cartpole::random_command(gen));
  std::tie(state, reward) = *simulator; // We get the new observation.
  print_context("current", state, reward);
  std::cout << std::endl;

  // We can use command sources in order to feed the system. The tick
  // range enables this. It repeats calls of a function f() that produces
  // a command. Here, f is a lambda calling random_command.
  std::cout << "Random command source" << std::endl;
  for(auto command
	: gdyn::ranges::tick([&gen](){return Cartpole::random_command(gen);})
	| std::views::take(20))
    std::cout << command << std::endl;

  // The command source can be obtained by a policy, i.e. a function
  // that chooses the command according to the system state.
  std::cout << std::endl;
  std::cout << "Policy command source" << std::endl;
  auto policy = [](const Cartpole::state_type state) {
    // L if positive theta, R otherwise
    if (state.theta > 0) {
      return Cartpole::command_type::L;
    }
    else {
      return Cartpole::command_type::R;
    }
  };
  std::tie(state, reward) = *simulator;
  print_context("start", state, reward);
  // TODO or until termination
  for(auto command
	: gdyn::ranges::tick([&simulator, &policy](){return policy(std::get<0>(*simulator));})
	| std::views::take(20)) {
    simulator(command, 5.0*Cartpole::delta_default); // We apply the command to the system to trigger a state transition.
    std::tie(state, reward) = *simulator;
    std::cout << command << " => " << Cartpole::to_string(state) << " (" << reward << ")." << std::endl;
  }
  std::tie(state, reward) = *simulator;
  print_context( "terminal", state, reward);

  // What we have just done is running an orbit/trajectory of the dynamical system. See next example.
  
  return 0;
  
}
