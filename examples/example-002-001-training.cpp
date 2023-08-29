#include <tuple>
#include <map>
#include <array>
#include <algorithm>
#include <cstddef>
#include <iostream>
#include <iomanip>

#include <gdyn.hpp>
#include "bonobo-system.hpp"

// Let us illustrate here how to implement an adaptive controler.

// First, we need to parametrize the controller. It consists of
// building a functional object, that computes the command from the
// parameter. This class also offers a way to learn from the
// transition.

struct adaptive_controller {
private:

  // reward_average['BONOOB'] = {average reward when action 'B' is applied from state 'BONOON',
  //                             ... action 'O'... ,
  //                             ... action 'N' ...};
  // We store values in a tabular way.
  std::map<std::string, std::array<double, 3>> reward_average;

  std::size_t index_of(Bonobo::letter l) {
    switch(l) {
    case Bonobo::letter::B : return 0;
    case Bonobo::letter::O : return 1;
    default                : return 2;
    }
  }

public:

  adaptive_controller() = default;

  // This controller is geedy. If the average reward for a state is
  // known, the command is the one with the highest reward. Otherwise,
  // we return 'B'. 
  auto operator()(const Bonobo::observation_type& observation) const {
    if(auto it = reward_average.find(observation); it != reward_average.end()) {
      auto values = it->second; // This is an array of 3 double
      auto best_position = std::distance(values.begin(),
					 std::max_element(values.begin(), values.end()));
      switch(best_position) {
      case 0 : return Bonobo::letter::B;
      case 1 : return Bonobo::letter::O;
      default: return Bonobo::letter::N;
      }
    }
    else
      return Bonobo::letter::B;
  }

#define ALPHA .1
  // Here, we train the controller from a transition. It consists in
  // updating the knowledge about average rewards.
  void learn(const gdyn::transition<Bonobo::observation_type, Bonobo::command_type, Bonobo::report_type>& sample) {
    auto obs    = sample.observation;
    auto cmd    = sample.command;
    auto reward = sample.report; 

    if(reward == 0) {
      // If we already know something about the reward, we update the
      // average, informing that 0 reward has been seen in the
      // transition.
      if(auto it = reward_average.find(obs); it != reward_average.end()) {
	auto& w = (it->second)[index_of(cmd)];
	w += ALPHA * (reward - w);
      }
    }
    else {
      // If a non null reward is observed, we create the entry in the
      // map if needed, and then we update the average.
      auto it = reward_average.find(obs); 
      if(it == reward_average.end()) 
	it = reward_average.insert({obs, {}}).first;
      auto& w = (it->second)[index_of(cmd)];
      w += ALPHA * (reward - w);
    }
  }  
};

// Let us check that our controller satisfies the appropriate concept.
static_assert(gdyn::specs::controller<adaptive_controller,
	      Bonobo::observation_type, Bonobo::command_type>);


// Our adaptive controller is greedy, it takes the best action it
// knows for a given observation. Let us set up an epsilon-greedy
// adaptor.
template<gdyn::specs::controller<Bonobo::observation_type, Bonobo::command_type> CONTROLLER,
	 typename RANDOM_GENERATOR>
auto epsilon_greedy(RANDOM_GENERATOR& gen, double epsilon, const CONTROLLER& controller) {
  return [&gen, &controller, epsilon](const auto& observation) {
    if(std::bernoulli_distribution(epsilon)(gen)) return Bonobo::random_command(gen);
    return controller(observation);
  };
}

int main(int argc, char* argv[]) {
  std::random_device rd;
  std::mt19937 gen(rd());

  Bonobo simulator;
  adaptive_controller greedy_controller;
  
  double avg_orbit_length = 0;
  while(true) {
    unsigned int step = 0;
    simulator = Bonobo::random_state(gen);
    for([[maybe_unused]] auto& unused
	  : gdyn::ranges::controller(simulator, epsilon_greedy(gen, .1, greedy_controller))                              // We use the controller to feed an orbit.
	  | gdyn::views::orbit(simulator)                                                                                // This is the orbit.
	  | gdyn::views::transition                                                                                      // We collect transitions.
	  | std::views::filter([&greedy_controller](const auto& sample){greedy_controller.learn(sample); return true;})) // We use the transitions to update the controller.
      ++step;
    avg_orbit_length += .05 * (step - avg_orbit_length);
    std::cout << "Average duration : " << std::setw(4) << (int)avg_orbit_length << " steps \r" << std::flush;
    // This is not working well, this example is just given for illustration.
  }
  
  
  return 0;
}
