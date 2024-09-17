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

A dynamical system is something that hosts an internal state. It may not be accessible directly, only observations are provided from it. It is "dynamic" since that state evolves. Evolution is triggerd by applying a so-called command. So from the current state, applying a command leads to the next state. This triplet is called a transition. When a transition is performed, the gdyn system generates a report for that transition.

Some specific states are defined as terminal states. When the system reaches such states, the experiment is considered usually as ended. The C++ syntax for gdyn systems is the following:

```cpp
auto sys = some_function_making_a_dynamical_system();

auto current_observation = *sys; // get current observation.

if (sys) { /* code when we are not in a terminal state. */ }
else     { /* code when we are in a terminal state.     */ }

sys = some_state; // sets the current state.

auto report = sys(some_command); // triggers a transition.
```





