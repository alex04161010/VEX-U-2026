using namespace vex;
extern brain Brain;
// VEXcode devices
extern digital_out arm;//下气动，铲子
extern digital_out top_arm_left;//上左气动
extern digital_out top_arm_right;//上右气动

extern controller Controller1;

extern motor leftMotor1;
extern motor leftMotor2;
extern motor leftMotor3;
extern motor leftMotor4;
extern motor rightMotor1;
extern motor rightMotor2;
extern motor rightMotor3;
extern motor rightMotor4;//底盘马达，左四右四

extern inertial Gyro;//陀螺仪

extern optical ColorSensor;//分色传感器

extern motor intake_upfr;
extern motor intake_downfr;
extern motor intake_upbk;
extern motor intake_downbk;
extern motor intake_power;//吸吐球要用到的五个电机变量
extern motor intake_secondtopfr;
extern motor intake_topfr;//新增两个电机变量

void  vexcodeInit( void );