PowerDistributionPanel
=============================
``public class Intake extends KillableSubsystem``
Section of the Robot resposible for collecting notes from the field

Spins 1 DC motor

Aquisition can be spinning into the note channel (IN), out of the note channel (REVERSE), or not moving(OFF).
On Start up defaults to off
The state of the Intake is changed through calling ``set(IntakeStates state)``.

**public void set(IntakeStates state, double speed))**
	Changes the speed of the Intake in the inputed direction
**public void set(IntakeStates state)**
	Wrapper around general toggle this function turns the Intake in the inputed direction and sets it to the default speed

