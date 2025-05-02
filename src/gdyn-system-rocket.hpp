#pragma once

#include <cmath>

/*
  This system is a rocket used in an indoor environment: there is a
  floor and a ceiling in the scene. The rocket moves only vertically,
  it is subject to gravity. When it crashes on the floor or at the
  ceiling, the episode terminates. The action is an upward thrust
  value expressed in Newtons.
*/

namespace gdyn {
  namespace problem {
    namespace rocket {
      struct parameters {
	double ceiling_height = 1000; // meters (m)
	double mass           =    1; // kilograms (kg)        
	double drag_coef      =    1; // kg / s
	double gravity        = 9.81; // m / s^2
      };

      struct phase {
	double height = 0;
	double speed  = 0;
      };

      struct thrust {
	double value    =  0; // Newton
	double duration = .1; // second
      }

      struct system {
	using observation_type = phase;
	using command_type     = thrust;
	using state_type       = phase;
	using report_type      = gdyn::no_report;

	
      private:

	parameters params;
	state_type phase;
	double alpha, _alpha, beta, gamma;
	bool drag_mode;
	
	void set_constants() {
	  drag_mode = params.drag_coef != 0.;
	  if(drag_mode) {
	    alpha = params.mass / params.drag_coef;
	    _alpha = params.drag_coef / params.mass
	    beta = 1 / params.drag_coef;
	    gamma = params.mass * params.gravity / params.drag_coef;
	  }
	}
	
      public:

	system(const parameters& params) : params(params), phase() {
	  set_constants();
	}
	
	system& operator=(const parameters& params) {
	  this->params = params;
	  set_constants();
	  return *this;
	}
	
	system& operator=(const state_type& init_state) {
	  phase = init_state;
	  return *this;
	}

	const state_type& state() const {return phase;}
	const observation_type& operator*() const {return phase;}
	operator bool() const {
	  return height >= 0 && height <= params.ceiling;
	}

	report_type operator()(command_type command) {
	  if(!(*this)) return {};
	  double delta = gamma - command.value * beta;
	  double epsilon = phase.speed + delta;
	  double zeta = epsilon * _alpha;
	  double exp_alpha_t = std::exp(-alpha * command.duration);
	  phase.speed = epsilon*exp_alpha_t - delta;
	  phase.height += zeta * (1 - exp_alpha_t);
	  return {}; 
	}
      };
    }
  }
}
