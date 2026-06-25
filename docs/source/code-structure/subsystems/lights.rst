Lights
======

This class should be inherited by every subsystem. Its responsible for emergency robot stops,
making sure that every subsystem fully stops.
This abstract class holds the abstract ``kill()``, every subsystem that
inherits ``KillableSubsystem`` implments its own version that shuts down the subsystem

----------------

.. toctree::
    :maxdepth: 1

    lights/algae-grabber-lights
