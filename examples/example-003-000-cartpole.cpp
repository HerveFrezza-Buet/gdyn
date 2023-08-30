#include <iostream>
#include <random>
#include <string>
#include <iomanip>

#include <gdyn.hpp>
#include "cartpole-system.hpp"

int main(int argc, char* argv[]) {
  std::random_device rd;
  std::mt19937 gen(rd());

  // make Environment with default parameters
  auto simulator = cartpole::make_environment();
  // Let us check the type requirements for our simulator.
  static_assert(gdyn::specs::system<decltype(simulator)>);
  
  simulator = cartpole::random_state(gen, simulator.param); // We set the state.
  auto obs = *simulator;          // We get the observation.
  print_context("start", obs, 0); // no reward at start

  // Let us apply a command to the system. Here we well apply a random
  // one.
  auto reward = simulator(cartpole::random_command(gen));
  obs = *simulator; // We get the new observation.
  print_context("current", obs, reward);
  std::cout << std::endl;

  // We can use command sources in order to feed the system. The tick
  // range enables this. It repeats calls of a function f() that produces
  // a command. Here, f is a lambda calling random_command.
  std::cout << "Random command source" << std::endl;
  for(auto command
	: gdyn::ranges::tick([&gen](){return cartpole::random_command(gen);})
	| std::views::take(20))
    std::cout << command << std::endl;

  // The command source can be obtained by a policy, i.e. a function
  // that chooses the command according to the system state.
  std::cout << std::endl;
  std::cout << "Policy command source" << std::endl;
  auto policy = [](const cartpole::Environment::observation_type obs) {
    // L if positive theta, R otherwise
    if (obs.theta > 0) {
      return cartpole::Environment::command_type::L;
    }
    else {
      return cartpole::Environment::command_type::R;
    }
  };

  // change Simulator parameters
  simulator.param.delta_time *= 5.0;

  obs = *simulator;
  print_context("start", obs, 0);

  for(auto command
        : gdyn::ranges::tick([&simulator, &policy](){return policy(*simulator);})
        | std::views::take(20)) {
    reward = simulator(command); // We apply the command to the system to trigger a state transition.
    obs = *simulator;
    print_context(cartpole::to_string(command)+" => ", obs, reward );

  }
  obs = *simulator;
  print_context( "terminal", obs, reward);

  // What we have just done is running an orbit/trajectory of the dynamical system.
  // See example-001-001-orbit or example-004-000-cheesemaze
  
  return 0;
  
}
