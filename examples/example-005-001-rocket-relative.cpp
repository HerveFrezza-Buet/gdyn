#include <iostream>
#include <ranges>
#include <iostream>
#include <sstream>
#include <gdyn.hpp>
#include <fstream>
#include <iterator>
#include <array>


#define END_OF_THRUST 9

int main(int argc, char* argv[]) {
  std::random_device rd;
  std::mt19937 gen(rd());
  
  gdyn::problem::rocket::parameters params;
  auto rocket = gdyn::problem::rocket::system(params);
  gdyn::problem::rocket::thrust up   {.value = 12};
  gdyn::problem::rocket::thrust none {};
  gdyn::problem::rocket::phase init_state {.height = 30., .speed = -5.};

  
  std::string mode = "nodrag";
  for(int i = 0; i < 2; ++i) {
    std::cout << std::endl;

    // Using different dt shows that the dynamics is not implemented by
    // an Euler integration when there is no drags. See the plottings.
    for(auto dt : {.1, 1., 3.}) {
      std::ostringstream filename;
      filename << "rocket-" << mode << "-" << int(1000*dt) << "ms.dat";
      std::ofstream datafile {filename.str()};
      up.duration = dt;
      none.duration = dt;
      double t = 0;
      rocket = init_state;
      for(auto [observation, action, report]
	    : gdyn::views::pulse([&up, &none, &t](){if(t < END_OF_THRUST - .01) return up; else return none;})
	    | gdyn::views::orbit(rocket)) {
	datafile << t << ' ' << observation.height << ' ' << observation.speed << std::endl;
	t += dt;
      }
      std::cout << "Generating " << filename.str() << std::endl;
    }
    
    std::ostringstream filename;
    filename << "rocket-" << mode << ".plot";
    std::ofstream datafile {filename.str()};

    datafile << "set yrange [-10: 100]" << std::endl
	     << "set trange [-10: 100]" << std::endl
	     << "set parametric" << std::endl
	     << "plot " << END_OF_THRUST << ",t lc rgb \"green\" notitle, \\" << std::endl
	     << "'rocket-" << mode << "-100ms.dat' using 1:2 with lines lc rgb \"black\" title \"0.1s\", \\" << std::endl
	     << "'rocket-" << mode << "-1000ms.dat' using 1:2 with points pt 1 ps 2 lc rgb \"blue\" title \"1s\", \\" << std::endl
	     << "'rocket-" << mode << "-3000ms.dat' using 1:2 with points pt 2 ps 2 lc rgb \"red\" title \"2.5s\"" << std::endl;

    std::cout << std::endl
	      << "Run : gnuplot -p " << filename.str() << std::endl
	      << std::endl
	      << std::endl;

    // For the second run
    mode = "drag";
    params.drag_coef = .2;
    rocket = params;
  }

  return 0;
}
