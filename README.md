# gdym

This is C++20 library for designing dynamical systems. This is based on generic programming (templates and concepts), so that many issues are solved at compiling time.

# Installing


Go into the `gdyn` directory you have git-cloned. Then type

```
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make -j
sudo make install
```

# Documentation

## Learn from examples

read and execute the examples in the order suggested by their names. Examples are installed in bin, so they can be invoked from command lines.

```
gdyn-example-....
```

## The concepts of gdyn

### Systems

A dynamical `system` is something that hosts an internal `state`. It may not be accessible directly, only observations are provided from it. It is "dynamic" since that state evolves. Evolution is triggerd by applying a so-called `command`. So from the current state, applying a command leads to the next state. This triplet is called a `transition`. When a transition is performed, the gdyn system generates a `report` for that transition.

Some specific states are defined as terminal states. When the system reaches such states, the experiment is considered usually as ended. The C++ syntax for gdyn systems is the following:

```cpp
auto sys = some_function_making_a_dynamical_system();

auto current_observation = *sys; // get current observation.

if (sys) { /* code when we are not in a terminal state. */ }
else     { /* code when we are in a terminal state.     */ }

sys = some_state; // sets the current state.

auto report = sys(some_command); // triggers a transition.
```

Having systems for which we cannot access the current state, but only the current observation, sounds right in physics, but this may be restrictive for simulators. The is why gdyn add the concept of `transparent_system`.

```cpp
auto current_state = sys.state();
```

### Orbits

A system evolves by performing successive transitions. This realizes a "path" in the state space, that are often reffered to as "trajectories" or "orbits". More precisely, orbits are made of successive transition, i.e. successive

- init_state/init_observation
- command_1
- state_1/observation_1, report_1. 
- command_2
- state_2/observation_2, report_2. 
- ...
- command_n
- state_n/observation_n, report_n
- command_n+1
- ...

It is convenient to represent a single `orbit_point` as $`(\mathrm{observation}_n, \mathrm{report}_n, \mathrm{command}_{n+1})`$.


command2triplets observation-command-report. These triplets are called `orbit_point`s in gdyn.
So an orbit is:
- observation1-command1-report1  (we have reached state 2 at this point)
- observation2-command2-report2  (we have reached state 3 at this point)
- ...




