#include <tuple>
#include <cmath>
#include <iostream>
#include <gdyn.hpp>

struct circle_system {
  enum class orientation : int {Up, Down, Left, Right};
  using observation_type = orientation;
  using command_type     = double; // dtheta
  using state_type       = double; // theta
  using report_type      = gdyn::no_report;

  double theta = 0;

  state_type state() const                               {return theta;}
  circle_system& operator=(const state_type& init_state) {theta = init_state; return *this;}
  report_type operator()(command_type dtheta)            {theta += dtheta; return {};}
  operator bool() const                                  {return true;}
  
  observation_type operator*() const {
    if(auto ct = std::cos(theta); ct > 0.7071) return orientation::Right;
    else if(ct < -0.7071)                      return orientation::Left;
    if(std::sin(theta) > 0.7071) return orientation::Up;
    return orientation::Down;
  }
};

std::ostream& operator<<(std::ostream& os, circle_system::orientation o) {
  switch(o) {
  case circle_system::orientation::Up:   return os << "Up";
  case circle_system::orientation::Left: return os << "Left";
  case circle_system::orientation::Right: return os << "Right";
  default: return os << "Down";
  }
}

#define dTHETA .5

int main(int argc, char *argv[]) {
  circle_system simulator;

  std::cout << "The native system" << std::endl
	    << "-----------------" << std::endl
	    << std::endl;
  for(auto [observation, action, report] 
	: gdyn::views::pulse([](){return dTHETA;}) 
	| gdyn::views::orbit(simulator)        
	| std::views::take(20))
    std::cout << observation << std::endl;
    
  std::cout << std::endl
	    << "The exposed system" << std::endl
	    << "------------------" << std::endl
	    << std::endl;
  auto exposed_simulator = gdyn::system::make_exposed(simulator);
  for(auto [state, action, report] 
	: gdyn::views::pulse([](){return dTHETA;}) 
	| gdyn::views::orbit(exposed_simulator)        
	| std::views::take(20))
    std::cout << state << std::endl;
  
  std::cout << std::endl
	    << "The detailed system" << std::endl
	    << "------------------" << std::endl
	    << std::endl;
  auto detailed_simulator = gdyn::system::make_detailed(simulator);
  for(auto [state_observation, action, report] 
	: gdyn::views::pulse([](){return dTHETA;}) 
	| gdyn::views::orbit(detailed_simulator)        
	| std::views::take(20)) {
    auto [state, observation] = state_observation;
    std::cout << state << ", " << observation << std::endl;
  }


  return 0;
  
}
