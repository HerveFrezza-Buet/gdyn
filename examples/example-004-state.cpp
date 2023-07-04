#include <iostream>
#include <iomanip>
#include <random>
#include <string>

#include "cheese_maze_state.hpp"

int main(int argc, char *argv[]) {
  std::random_device rd;
  std::mt19937 gen(rd());

  CheeseMazeState simulator;
  simulator.param.mishap_proba = 0.0;

  std::cout << "__Welcome to the CheeseMaze **********************" << std::endl;
  simulator = CheeseMazeState::random_state(gen); // We set the state.
  auto [state, reward] = *simulator;     // We get the observation.
  print_context("start", state, reward);

  // Let us apply a command to the system. Here we well apply a random
  // one.
  auto cmd = CheeseMazeState::random_command(gen);
  std::cout << "  apply " << cmd << std::endl;
  simulator(cmd, gen);
  std::tie(state, reward) = *simulator; // We get the new observation.
  print_context("current", state, reward);
  std::cout << std::endl;

  return 0;
}
