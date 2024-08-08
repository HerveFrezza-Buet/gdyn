#include <iostream>
#include <random>

#include <gdyn.hpp>

#define GRID_WIDTH  5
#define GRID_HEIGHT 4
#define GRID_GOAL   8

int main(int argc, char* argv[]) {
  std::random_device rd;
  std::mt19937 gen(rd());

  // make system
  auto simulator = gdyn::problem::grid_world::make<GRID_WIDTH, GRID_HEIGHT, GRID_GOAL>();
  simulator = decltype(simulator)::random_state(gen);
  auto [w, h] = decltype(simulator)::position(GRID_GOAL);
  std::cout << "goal pos = " << w << ", " << h << std::endl;

  double reward_sum {0.0};
  for(auto [observation, action, report] // report is the reward here.
          : gdyn::views::pulse([&gen](){return gdyn::problem::grid_world::random_command(gen);}) // This is a source of random commands...
          | gdyn::views::orbit(simulator)) {                                    // ... that feeds an orbit of the stystem...
      if (report) {
          reward_sum += *report;

          if (*report == -1.0) std::cout << "Ouch !!!  ";
          else                 std::cout << "          ";

          auto [w, h] = decltype(simulator)::position(observation);
          std::cout << "pos = " << observation << " : " << w << ", " << h ;
          if (action) std::cout << " -> " << *action;
          else        std::cout << " Goal reached !!";
          std::cout << std::endl;
      }
  }
  std::cout << "reward sum before reaching goal with random policy is " << reward_sum << std::endl;

  return 0;
}
