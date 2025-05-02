#include <iostream>
#include <ranges>
#include <sstream>
#include <fstream>
#include <gdyn.hpp>


#define DT .1

int main(int argc, char* argv[]) {
  std::random_device rd;
  std::mt19937 gen(rd());
  
  gdyn::problem::rocket::parameters params;
  gdyn::problem::rocket::thrust up   {.value = 20, .duration = DT};
  gdyn::problem::rocket::thrust none {.value =  0, .duration = DT};

  auto rocket = gdyn::problem::rocket::system(params); // This knows its height and speed.
  double target = 50; // This is the height we would like the rocket to stay at.
  double t;

  auto relative_rocket = gdyn::problem::rocket::relative::system(rocket, [&target](){return target;});

  // Let us control the rocket roughly: if the error is negative, we apply up. We apply none otherwise.

  std::cout << std::endl;
  
  {
    std::string filename {"relative-rocket.dat"};
    std::ofstream datafile {filename};
    std::string targetname {"target.dat"};
    std::ofstream datatarget {targetname};
    t = 0;
    relative_rocket = {.error = 10, .speed = 0};
    for(auto [observation, action, report]
	  : gdyn::views::controller(relative_rocket, [up, none](double error){if(error < 0) return up; return none;})
	  | gdyn::views::orbit(rocket)
	  | std::views::take(1000)) {
      // We get orbits of rocket, while controlling relative_rocket. So
      // we have access to height and speed, while relative rocket
      // observations are only the current error.
      datafile << t << ' ' << observation.height << ' ' << observation.speed << std::endl;
      datatarget << t << ' ' << target << std::endl;
      t += DT;
      if(t > 35) target = 30; // We change the target.
      if(t > 60) target = 50; // We change the target.
    }
    std::cout << "Generating " << filename << " and " << targetname << std::endl;
  }

  
  std::string filename {"relative-rocket.plot"};
  std::ofstream plotfile {filename};

  plotfile << "set yrange [0: 100]" << std::endl
	   << "set trange [0:" << t << ']' << std::endl
	   << "set parametric" << std::endl
	   << "plot 'relative-rocket.dat' using 1:2 with lines lc rgb \"black\" title \"rocket height\", \\" << std::endl
	   << "'target.dat' using 1:2 with lines lc rgb \"green\" title \"target\"" << std::endl;
  std::cout << std::endl
	    << "Run : gnuplot -p " << filename << std::endl
	    << std::endl
	    << std::endl;


  return 0;
}
