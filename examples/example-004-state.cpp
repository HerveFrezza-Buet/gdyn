/*
 * Test du system CheeseMaze qui a des transitions probabilistes.
 */

#include <iostream>
#include <iomanip>
#include <random>
#include <string>

//#include "cheese_maze_state.hpp"
#include "cheese_maze_state_hfb.hpp"

int main(int argc, char *argv[]) {

  std::random_device rd;
  std::mt19937 gen(rd());

  // using CheeseMaze = CheeseMazeState<std::mt19937>;
  using CheeseMaze = cheese_maze::Environment<std::mt19937>;
  // Checking
  std::cout << "__CHECK it is a gdyn" << std::endl;
  static_assert(gdyn::specs::system<CheeseMaze>);


  // Parameters<std::mt19937> param(gen);
  // CheeseMazeState<std::mt19937> simulator(param);
  // simulator.param.mishap_proba = 0.0;
  cheese_maze::Parameters param;
  param.mishap_proba = 0.0;
  CheeseMaze simulator(param, gen);

  std::cout << "__Welcome to the CheeseMaze **********************" << std::endl;
  simulator = cheese_maze::random_cell(gen); // We set the state.
  auto [state, reward] = *simulator;     // We get the observation.
  cheese_maze::print_context<std::mt19937>("start", state, reward);

  // Let us apply a command to the system. Here we well apply a random
  // one.
  CheeseMaze::command_type cmd = cheese_maze::random_dir(gen);
  std::cout << "  apply " << cheese_maze::to_string(cmd) << std::endl;

  // TODO FIXME mais std::cout << "  apply " << cmd  << std::endl;
  // ne compile pas.
  std::cout << "  apply " << cmd  << std::endl;
  simulator(cmd);
  std::tie(state, reward) = *simulator; // We get the new observation.
  cheese_maze::print_context<std::mt19937>("current", state, reward);
  std::cout << std::endl;

  return 0;
}
