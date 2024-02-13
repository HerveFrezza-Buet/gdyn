#include <iostream>
#include <random>
#include <string>
#include <iomanip>

#include <gdyn.hpp>

void print_context(const std::string& msg,
		   gdyn::problem::cartpole::system::observation_type& obs,
		   double reward ) {
  
  std::cout << msg << ": "<< to_string(obs) << ", " << std::setw(3) << reward << std::endl;
}

int main(int argc, char* argv[]) {
  std::random_device rd;
  std::mt19937 gen(rd());

  // make system with default parameters
  auto simulator = gdyn::problem::cartpole::make();
  // Let us check the type requirements for our simulator.
  static_assert(gdyn::concepts::system<decltype(simulator)>);
  
  simulator = gdyn::problem::cartpole::random_state(gen, simulator.param); // We set the state.
  auto obs = *simulator;          // We get the observation.
  print_context("start", obs, 0); // no reward at start

  // Let us apply a command to the system. Here we well apply a random
  // one.
  auto reward = simulator(gdyn::problem::cartpole::random_command(gen));
  obs = *simulator; // We get the new observation.
  print_context("current", obs, reward);
  std::cout << std::endl;

  // We can use command sources in order to feed the system. The tick
  // range enables this. It repeats calls of a function f() that produces
  // a command. Here, f is a lambda calling random_command.
  std::cout << "Random command source" << std::endl;
  for(auto command
	: gdyn::views::pulse([&gen](){return gdyn::problem::cartpole::random_command(gen);})
	| std::views::take(20))
    std::cout << command << std::endl;

  // The command source can be obtained by a policy, i.e. a function
  // that chooses the command according to the system state.
  std::cout << std::endl;
  std::cout << "Policy command source" << std::endl;
  auto policy = [](const gdyn::problem::cartpole::system::observation_type obs) {
    // Left if positive theta, Right otherwise
    if (obs.theta > 0) return gdyn::problem::cartpole::system::command_type::Left; // We can use system::command_type...
    else               return gdyn::problem::cartpole::direction::Right;           // ... or directly gdyn::problem::cartpole::direction
    
  };

  // change Simulator parameters
  simulator.param.delta_time *= 5.0;

  obs = *simulator;
  print_context("start", obs, 0);

  for(auto command
        : gdyn::views::controller(simulator, policy)
        | std::views::take(20)) {
    reward = simulator(command); // We apply the command to the system to trigger a state transition.
    obs = *simulator;
    print_context(gdyn::problem::cartpole::to_string(command)+" => ", obs, reward );

  }
  obs = *simulator;
  print_context( "final", obs, reward);

  // What we have just done is running an orbit/trajectory of the dynamical system.
  // See example-001-001-orbit or example-004-000-cheesemaze
  
  return 0;
  
}
