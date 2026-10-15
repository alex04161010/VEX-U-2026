#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftMotor1 = motor(PORT15, ratio6_1, false);
motor leftMotor2 = motor(PORT2, ratio6_1, false);
motor leftMotor3 = motor(PORT3, ratio6_1, false);
motor leftMotor4 = motor(PORT4, ratio6_1, true);
motor rightMotor1 = motor(PORT5, ratio6_1, true);
motor rightMotor2 = motor(PORT6, ratio6_1, true);
motor rightMotor3 = motor(PORT10, ratio6_1, true);
motor rightMotor4 = motor(PORT8, ratio6_1, false);
inertial Gyro = inertial(PORT21);//陀螺仪

motor intake_upfr=motor(PORT7,ratio18_1,false);//吸球电机1号，方向待定
motor intake_downfr=motor(PORT11,ratio18_1,false);//吸球电机2号，方向待定
motor intake_upbk=motor(PORT12,ratio18_1,false);//吸球电机3号，方向待定
motor intake_downbk=motor(PORT13,ratio18_1,false);//吸球电机4号，方向待定
motor intake_power=motor(PORT14,ratio18_1,false);//吸球电机5号，方向待定
motor intake_secondtopfr=motor(PORT1,ratio18_1,false);//吸球电机5号，方向待定
motor intake_topfr=motor(PORT16,ratio18_1,false);//吸球电机5号，方向待定


optical ColorSensor = optical(PORT20); // 颜色传感器
// distance back_left=distance(PORT5);
// distance back_right=distance(PORT4);距离传感器

digital_out arm=digital_out(Brain.ThreeWirePort.H);//下气动，铲子
digital_out top_arm_right=digital_out(Brain.ThreeWirePort.C);//上右气动

bool RemoteControlCodeEnabled = true;
void vexcodeInit( void ) {
  // nothing to initialize
}
//R1 吸
//R2 吐下层
//L1 吐上层
//L2 吐中层