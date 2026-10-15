#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftMotor1 = motor(PORT4, ratio6_1, false);
motor leftMotor2 = motor(PORT7, ratio6_1, false);
motor leftMotor3 = motor(PORT5, ratio18_1, false);

motor leftMotor4 = motor(PORT12, ratio6_1, false);
motor leftMotor5 = motor(PORT13, ratio6_1, false);

motor rightMotor1 = motor(PORT1, ratio6_1, true);
motor rightMotor2 = motor(PORT2, ratio18_1, true);
motor rightMotor3 = motor(PORT3, ratio6_1, true);

motor rightMotor4 = motor(PORT14, ratio18_1, true);
motor rightMotor5 = motor(PORT15, ratio18_1, false);
inertial Gyro = inertial(PORT11);
motor intake=motor(PORT11,ratio18_1,true);
motor tai1 = motor(PORT19, ratio18_1, true);
motor tai2 = motor(PORT20, ratio18_1, false);
motor intake2=motor(PORT15,ratio6_1,false);
motor Throw=motor(PORT17,ratio6_1,true);//up intake / score
//limit throw_limit=limit(Brain.ThreeWirePort.A);
distance throw_starter=distance(PORT19);
distance back_left=distance(PORT21);//back distance
//distance back_right=distance(PORT4);
digital_out arm=digital_out(Brain.ThreeWirePort.A);
digital_out door=digital_out(Brain.ThreeWirePort.B);
digital_out wing=digital_out(Brain.ThreeWirePort.H);
digital_out Fuck=digital_out(Brain.ThreeWirePort.C);//hook
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