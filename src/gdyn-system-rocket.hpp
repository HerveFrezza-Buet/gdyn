#pragma once

#include <cmath>
#include <functional>

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
	double ceiling_height    = 1000; // meters (m)
	double mass              =    1; // kilograms (kg)        
	double drag_coef         =  .00; // kg / s
	double gravity           = 9.81; // m / s^2
	double internal_euler_dt = .01; // Used when there is drags, for euler integration.
      };

      struct phase {
	double height = 0;
	double speed  = 0;
      };

      struct thrust {
	double value    =  0; // Newton
	double duration = .1; // second
      };

      struct system {
	using observation_type = phase;
	using command_type     = thrust;
	using state_type       = phase;
	using report_type      = gdyn::no_report;

	
      private:

	parameters params;
	state_type internal_state;
	double alpha, _alpha, _m, _dc, mg;
	bool drag_mode;
	
	void set_constants() {
	  drag_mode = params.drag_coef != 0.;
	  if(drag_mode) {
	    // I encounter numerical issues... I use Euler when there is a drag.
	    
	    // alpha  = params.mass / params.drag_coef;
	    // _alpha = params.drag_coef / params.mass;
	    // mg     = params.mass * params.gravity;
	    // _dc    = 1 / params.drag_coef;
	    
	    mg = params.mass * params.gravity;
	    _m = 1 / params.mass;
	  }
	  else {
	    _m = 1 / params.mass;
	  }
	}
	
      public:

	system(const parameters& params) : params(params), internal_state() {
	  set_constants();
	}
	
	system& operator=(const parameters& params) {
	  this->params = params;
	  set_constants();
	  return *this;
	}
	
	system& operator=(const state_type& init_state) {
	  internal_state = init_state;
	  return *this;
	}

	const state_type& state() const {return internal_state;}
	const observation_type& operator*() const {return internal_state;}
	operator bool() const {
	  return internal_state.height >= 0 && internal_state.height <= params.ceiling_height;
	}

	report_type operator()(command_type command) {
	  if(!(*this)) return {};
	  if(drag_mode) {
	    // I encounter numerical issues... I use Euler when there is a drag.
	    
	    // double delta = (command.value - mg) * _dc;
	    // double A = internal_state.speed - delta;
	    // double exp_alpha_t = std::exp(-alpha * command.duration);
	    // internal_state.speed = A*exp_alpha_t + delta;
	    // internal_state.height += A * (1 - exp_alpha_t) * _alpha * (1 - exp_alpha_t) + delta * command.duration;
	    
	    double a = command.value * _m  - params.gravity;
	    for(double t = params.internal_euler_dt; t <= command.duration; t += params.internal_euler_dt) {
	      double aa = a - params.drag_coef * internal_state.speed;
	      internal_state.speed  += aa * params.internal_euler_dt;
	      internal_state.height += internal_state.speed * params.internal_euler_dt;
	    }
	  }
	  else {
	    // We do not use Euler here
	    double v0 = internal_state.speed;
	    double coef = -params.gravity + command.value * _m;
	    internal_state.speed = v0 + coef * command.duration;
	    internal_state.height += command.duration * (v0 + .5 * coef * command.duration);
	  }
	  return {}; 
	}
      };

      namespace relative {
	struct phase {
	  double error = 0;
	  double speed = 0;
	};
	
	struct system {
	  using observation_type = double;
	  using command_type     = thrust;
	  using state_type       = phase;
	  using report_type      = gdyn::no_report;
	
	private:

	  gdyn::problem::rocket::system& borrowed_system;
	  std::function<double ()> get_target;
	  mutable state_type internal_state;
	  
	  void synchronize_state() const {
	    auto& borrowed_state = borrowed_system.state();
	    internal_state.speed = borrowed_state.speed;
	    internal_state.error = borrowed_state.height - get_target();
	  }

	public:

	  template<typename GET_TARGET>
	  system(gdyn::problem::rocket::system& borrowed_system, const GET_TARGET& get_target)
	    : borrowed_system(borrowed_system), get_target(get_target) {
	    synchronize_state();
	  }
	  
	  system& operator=(const state_type& init_state) {
	    gdyn::problem::rocket::phase init {.height = get_target() + init_state.error, .speed = init_state.speed};
	    borrowed_system = init;
	    synchronize_state();
	    return *this;
	  }

	  const state_type& state() const {synchronize_state(); return internal_state;}
	  const observation_type& operator*() const {synchronize_state(); return internal_state.error;}
	  operator bool() const {return borrowed_system;}

	  report_type operator()(command_type command) {return borrowed_system(command);} // no need to synchronize here, this will be done on demand.
	  
	};
      }
    }
  }
}
