#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftMotor1 = motor(PORT11, ratio6_1, false);
motor leftMotor2 = motor(PORT12, ratio6_1, false);
motor leftMotor3 = motor(PORT13, ratio6_1, true);
motor rightMotor1 = motor(PORT1, ratio6_1, true);
motor rightMotor2 = motor(PORT2, ratio6_1, true);
motor rightMotor3 = motor(PORT3, ratio6_1, false);
inertial Gyro = inertial(PORT5);
motor intake=motor(PORT7,ratio18_1,false);
motor Throw=motor(PORT18,ratio18_1,true);
limit throw_limit=limit(Brain.ThreeWirePort.A);
distance throw_starter=distance(PORT19);
distance back_left=distance(PORT5);
distance back_right=distance(PORT4);
digital_out arm=digital_out(Brain.ThreeWirePort.A);
digital_out door=digital_out(Brain.ThreeWirePort.B);
digital_out wing=digital_out(Brain.ThreeWirePort.H);
digital_out Fuck=digital_out(Brain.ThreeWirePort.F);
inertial inertial_right=inertial(PORT15);
rotation throw_rotation = rotation(PORT14);
motor Arm = motor(PORT6,ratio36_1,true);
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}