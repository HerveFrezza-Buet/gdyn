#include <iostream>
#include <random>

#include <gdyn.hpp>


int main(int argc, char* argv[]) {
  std::random_device rd;
  std::mt19937 gen(rd());

  // make system
  // we define some parameters (here, the same as the defaults ones)
  gdyn::problem::mountain_car::parameters params;
  params.gravity = 0.0025; // default value
  auto simulator = gdyn::problem::mountain_car::make(params);
  simulator = gdyn::problem::mountain_car::random_state(gen, params);

  for(auto [observation, action, report] // report is the reward here.
        : gdyn::views::pulse([&gen](){return gdyn::problem::mountain_car::random_command(gen);}) // This is a source of random commands...
        | gdyn::views::orbit(simulator)
        | std::views::take(20)) {                                    // ... that feeds an orbit of the stystem...
          std::cout << "state = " << observation;
          if (action) std::cout << " -> " << *action;
          else        std::cout << " Goal reached !!";
          if (report) std::cout << " (reward=" << *report << ")";
          std::cout << std::endl;
  }

  return 0;
}
