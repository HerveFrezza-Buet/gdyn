/*
 * Test du system CheeseMaze qui a des transitions probabilistes.
 */

#include <iostream>
#include <iomanip>
#include <random>
#include <string>

#include "cheesemaze-system.hpp"


template<typename SIMULATOR, typename GEN>
void orbit(SIMULATOR& sim, GEN& gen) {
  unsigned int step = 1;
  for(auto [obs_or_state, command, report]
        : gdyn::ranges::tick([&gen](){return cheese_maze::random_command(gen);})
        | gdyn::views::orbit(sim)
        | std::views::take(10)) {
    // print_orbit_point increments 'step'
    cheese_maze::print_orbit_point(obs_or_state, command, report, step);
  }
}


int main(int argc, char *argv[]) {

  std::random_device rd;
  std::mt19937 gen(rd());

  cheese_maze::Cell state;


  cheese_maze::Parameters param;
  param.mishap_proba = 0.0;
  auto simulator = cheese_maze::make_environment(param, gen);

  // Let us check the type requirements for our simulator.
  static_assert(gdyn::specs::system<decltype(simulator)>);
  static_assert(gdyn::specs::transparent_system<decltype(simulator)>);

  state = cheese_maze::random_state(gen); 
  simulator = state; // We set the state, by an extra 'out of concept' function.
  // simulator = {state, 0}; // This is the initialization fitting the system concept.
  std::cout << std::endl << std::endl
            << "Orbit of observations (i.e. local view of walls)"
            << std::endl;
  cheese_maze::print_context("start",
                             simulator.state(), // can get state of transparent_system
                             0);                // no reward as no transition has been applied.

  auto reward = simulator(cheese_maze::random_command(gen));
  cheese_maze::print_context("step ", *simulator, reward);
  cheese_maze::print_context("     ", simulator.state(), reward);

  orbit(simulator, gen);

  // simulator separates the internal state from the observation. We
  // can ignore observation and work only with states, by "exposing"
  // the simulator (i.e. the observations become the state itself).

  auto exposed_simulator = gdyn::system::make_exposed(simulator);
  
  static_assert(gdyn::specs::system<decltype(exposed_simulator)>);
  // but exposed_simulator is no more a transparent_system.
  // static_assert(gdyn::specs::transparent_system<decltype(exposed_simulator)>);
  
  state = cheese_maze::random_state(gen); 
  simulator = state; // We set the state (to exposed simulator as well, since it owns a reference on simulator).
  std::cout << std::endl << std::endl
            << "Orbit of states (i.e. cells)"
            << std::endl;
  cheese_maze::print_context("start",
                             state, // .state() is not available for non-transparent systems.
                             0);    // no transition => no report => no reward
  reward = exposed_simulator(cheese_maze::random_command(gen));
  cheese_maze::print_context("step ", *simulator, reward);
  // cannot use .state()
  // cheese_maze::print_context("     ", simulator.state(), reward);

  orbit(exposed_simulator, gen);

  return 0;
}
