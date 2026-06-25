IO
==

Input/Output (IO) classes and interfaces define how the subsystems interact with the environment.
There are 3 types of IO classes:

- **Real** - These classes interact with the real hardware.
- **Sim** - These classes simulate the hardware in a simulation environment.
- **Stub** - These classes are used for not yet completed subsystems or to temporarily disable a subsystem and do not interact with any hardware or simulation.

-----------------------

.. toctree::
    :maxdepth: 1

    io/climber-io
    io/coral-intake-io
    io/elevator-arm-io
    io/elevator-head-io
    io/elevator-io
    io/nav-sensor-io
    io/swerve-module-io
