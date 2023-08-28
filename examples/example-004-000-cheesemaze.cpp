/*
 * Test du system CheeseMaze qui a des transitions probabilistes.
 */

#include <iostream>
#include <iomanip>
#include <random>
#include <string>

#include "cheesemaze-system.hpp"

int main(int argc, char *argv[]) {

  std::random_device rd;
  std::mt19937 gen(rd());


  cheese_maze::Parameters param;
  param.mishap_proba = 0.0;
  auto simulator = cheeze_maze::make_environment(param, gen);

  // Let us check the type requirements for our simulator.
  static_assert(gdyn::specs::system<decltype(simulator)>);

  std::cout << "__Welcome to the CheeseMaze **********************" << std::endl;
  simulator = cheese_maze::random_state(gen); // We set the state.
  auto [state, reward] = *simulator;     // We get the observation.
  cheese_maze::print_context("start", state, reward);

  // Let us apply a command to the system. Here we well apply a random
  // one.
  CheeseMaze::command_type cmd = cheese_maze::random_command(gen);
  std::cout << "  apply " << cmd  << std::endl;
  simulator(cmd);
  std::tie(state, reward) = *simulator; // We get the new observation.
  cheese_maze::print_context("current", state, reward);
  std::cout << std::endl;

  return 0;
}
