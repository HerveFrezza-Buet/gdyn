/*

Copyright 2023 Herve FREZZA-BUET, Alain DUTECH

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/


#pragma once


#include <gdynConcepts.hpp>


namespace gdyn {
  /**
   * This is en empty structure that can be used as a reporting type
   * for system transitions when nothing is to be reported.
   */
  struct no_report {};
  
  namespace system {
    template<concepts::transparent_system BASE_SYSTEM>
    class exposed {
      BASE_SYSTEM& base_system;
    public:
      
      using observation_type = typename BASE_SYSTEM::state_type;
      using command_type     = typename BASE_SYSTEM::command_type;
      using state_type       = typename BASE_SYSTEM::state_type;
      using report_type       = typename BASE_SYSTEM::report_type;

      exposed(BASE_SYSTEM& base_system) : base_system(base_system) {}
      
      exposed& operator=(const state_type& init_state) {base_system = init_state; return *this;}
      observation_type operator*() const               {return base_system.state();}
      report_type operator()(command_type command)     {return base_system(command);}
      operator bool() const                            {return base_system;}  
    };

    template<concepts::transparent_system BASE_SYSTEM>
    auto make_exposed(BASE_SYSTEM& base_system) {return exposed<BASE_SYSTEM>(base_system);}
    
  }
}
