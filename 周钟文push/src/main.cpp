// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Optical21            optical       21              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#define Controller Controller1
#define base_stop(x) base_drive(0, 0, 0, x)
#define open_door door.set(1)
#define close_door door.set(0)
#define lift_arm arm.set(1)
#define drop_arm arm.set(0)
#define open_wing wing.set(1)
#define close_wing wing.set(0)
#define FUCK Fuck.set(1);
#define start_intake intake.spin(fwd,100,pct)
#define stop_intake intake.stop(hold);
#define re_intake intake.spin(fwd,-100,pct)
extern motor intake;
extern motor intake2;
extern motor tai1;
extern motor tai2;
extern motor Throw;
extern motor Arm;
extern limit throw_limit;
extern distance throw_starter;
extern distance back_left;
extern distance back_right;
extern digital_out arm;
extern digital_out door;
extern digital_out wing;
extern digital_out Fuck;
extern inertial inertial_right;
extern rotation throw_rotation;
using namespace vex;
bool arm_down = true;
bool Bpressed = false;
bool door_open = true;
bool Ypressed = false;
bool wing_open = false;
bool Leftpressed = false;
bool Fucked=false;
int Reverse = -1;
bool Xpressed=false;
int max_throw_angle=310;
double t_fl, t_fr,t_ml,t_mr, t_bl, t_br, t_intake, t_throw;
int fucknumber=0;
bool fucklast=false;
//////////////////////////////////////////////////////////////////////////
///////////////////////////////Basic Functoin/////////////////////////////
//////////////////////////////////////////////////////////////////////////

competition Competition;

timer T1,T2,TAcc;

double max_l = 0, max_r = 0;
void base_drive(double left_speed, double right_speed, double max_rpm = 570,
                brakeType type = brake) {
  double speed_l = left_speed, speed_r = right_speed;
  speed_l = speed_l / 100.0 * max_rpm;
  speed_r = speed_r / 100.0 * max_rpm;
  if (speed_l != 0) {
    leftMotor1.spin(reverse, speed_l, rpm);
    leftMotor2.spin(reverse, speed_l, rpm);
    leftMotor3.spin(reverse, speed_l, rpm);
    leftMotor4.spin(reverse, speed_l, rpm);
    leftMotor5.spin(reverse, speed_l, rpm);
  }
  if (speed_r != 0) {
    rightMotor1.spin(reverse, speed_r, rpm);
    rightMotor2.spin(reverse, speed_r, rpm);
    rightMotor3.spin(reverse, speed_r, rpm);
    rightMotor4.spin(reverse, speed_r, rpm);
    rightMotor5.spin(reverse, speed_r, rpm);
  }
  if (speed_l == 0) {
    leftMotor1.stop(type);
    leftMotor2.stop(type);
    leftMotor3.stop(type);
    leftMotor4.stop(type);
    leftMotor5.stop(type);
  }
  if (speed_r == 0) {
    rightMotor1.stop(type);
    rightMotor2.stop(type);
    rightMotor3.stop(type);
    rightMotor4.stop(type);
    rightMotor5.stop(type);
  }
  double rpm_la = leftMotor1.velocity(rpm), rpm_lb = leftMotor2.velocity(rpm),
         rpm_lc = leftMotor3.velocity(rpm), rpm_ra = rightMotor1.velocity(rpm),
         rpm_rb = rightMotor2.velocity(rpm), rpm_rc = rightMotor3.velocity(rpm);
  /*
  Brain.Screen.printAt(100, 100, "%f", rpm_la);
  Brain.Screen.printAt(100, 120, "%f", rpm_lb);
  Brain.Screen.printAt(100, 140, "%f", rpm_lc);
  Brain.Screen.printAt(300, 100, "%f", rpm_ra);
  Brain.Screen.printAt(300, 120, "%f", rpm_rb);
  Brain.Screen.printAt(300, 140, "%f", rpm_rc);
  */
  if (((rpm_la + rpm_lb + rpm_lc) / 3.0) > max_l) {
    max_l = (rpm_la + rpm_lb + rpm_lc) / 3.0;
  }
  if (((rpm_ra + rpm_rb + rpm_rc) / 3.0) > max_r) {
    max_r = (rpm_ra + rpm_rb + rpm_rc) / 3.0;
  }
  Brain.Screen.printAt(100, 180, "%f", max_l);
  Brain.Screen.printAt(300, 180, "%f", max_r);
}



void user_move() {
  if (!Controller.ButtonX.pressing()) {
    Xpressed = false;
  }
  if (Controller.ButtonX.pressing() && !Xpressed)
    Reverse *= -1;
    Xpressed = true;
  double turn_ = 0;
  if (abs(Controller.Axis1.value()) >= 3) {
    turn_ = -Controller.Axis1.value()*0.75;
  }
  double fwd_ = 0;
  if (abs(Controller.Axis3.value()) >= 3) {
    fwd_ = -Controller.Axis3.value() * int(Reverse);
  }
 // if (turn_ > 10 || turn_ < -10) {
  //  fwd_ = fwd_ * 0.8571;
 // }
  base_drive(-fwd_ - turn_, -fwd_ +turn_, 600);
}


int sign(int a)
{
    if(a>0) return 1;
    else if(a<0) return -1;
    else return 0;
}

int speedlimit(int speed,int max=100)
{
    return abs(speed)>max ? sign(speed)*max : speed;
}

void LeftWheel(int speed)
{
  leftMotor1.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  leftMotor2.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  leftMotor3.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  leftMotor4.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  leftMotor5.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
}

void RightWheel(int speed)
{
  rightMotor1.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  rightMotor2.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  rightMotor3.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  rightMotor4.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  rightMotor5.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
}

void Forward(double dis, double speed) {
  double left, right, start_dis, target_dis,
      start_slow_k = 0.3, start_mis = 5, fix_k = 0.2, basic_k = 0.8, corr = 5;
  left = back_left.objectDistance(mm);
  right = back_right.objectDistance(mm);
  start_dis = (left + right) / 2;
  target_dis = start_dis + dis;
  double now_dis = 0;
  double despeed = 0;
  double basic_speed;
  while (1) {
    left = back_left.objectDistance(mm);
    right = back_right.objectDistance(mm);
    now_dis = (left + right) / 2;
    if (fabs(left - right) > start_mis) {
      despeed = -(left - right - start_mis) * fix_k;
    }
    if (now_dis <= target_dis) {
      if (now_dis >= start_slow_k * dis + start_dis) {
        basic_speed = (target_dis - now_dis) / dis * basic_k * speed + corr;
      } else {
        basic_speed = speed;
      }
    }
    double speed_l, speed_r;
    if (basic_speed + despeed <= 3) {
      speed_l = 0;
    } else {
      speed_l = basic_speed + despeed;
    }
    if (basic_speed - despeed <= 3) {
      speed_r = 0;
    } else {
      speed_r = basic_speed - despeed;
    }
    if ((speed_l <= 0 && speed_r <= 0) || now_dis - start_dis > dis - 10) {
      base_stop(hold);
      break;
    } else {
      base_drive(speed_l, speed_r);
    }
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 20, "left :%f", left);
    Brain.Screen.printAt(10, 40, "right:%f", right);
    Brain.Screen.printAt(10, 60, "%f", now_dis - start_dis);
    Brain.Screen.printAt(10, 80, "%f", basic_speed);
    Brain.Screen.printAt(10, 100, "%f", despeed);
    wait(20, msec);
  }
}
void move_with_time(double time_ms ,double speed)
{
  base_drive(speed, speed);
  wait(time_ms, msec);
  base_stop(coast);
}
void Backward(double dis, double speed) {
  double left, right, start_dis, target_dis,
      start_slow_k = 0.3, start_mis = 5, fix_k = 0.2, basic_k = 0.8, corr = 5;
  left = back_left.objectDistance(mm);
  right = back_right.objectDistance(mm);
  start_dis = (left + right) / 2;
  target_dis = start_dis - dis;
  double now_dis = 0;
  double despeed = 0;
  double basic_speed;
  double speed_l, speed_r;
  while (1) {
    left = back_left.objectDistance(mm);
    right = back_right.objectDistance(mm);
    now_dis = (left + right) / 2;
    if (fabs(left - right) > start_mis) {
      despeed = -(left - right - start_mis) * fix_k;
    }
    if (now_dis >= target_dis) {
      if (now_dis <=start_dis- start_slow_k * dis  ) {
        basic_speed = (target_dis - now_dis) / dis * basic_k * speed - corr;
      } else {
        basic_speed = -speed;
      }
    }
    if (basic_speed + despeed <= 3) {
      speed_l = 0;
    } else {
      speed_l = basic_speed + despeed;
    }
    if (basic_speed - despeed <= 3) {
      speed_r = 0;
    } else {
      speed_r = basic_speed - despeed;
    }
    if ((speed_l >= 5 && speed_r >= 5) || fabs(now_dis - start_dis) > dis - 5) {
      base_stop(hold);
      break;
    } else {
      base_drive(speed_l, speed_r);
    }
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 20, "left :%f", speed_l);
    Brain.Screen.printAt(10, 40, "right:%f", speed_r);
    Brain.Screen.printAt(10, 60, "%f", now_dis - start_dis);
    Brain.Screen.printAt(10, 80, "%f", basic_speed);
    Brain.Screen.printAt(10, 100, "%f", despeed);
    wait(20, msec);
  }
  Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 20, "left :%f", speed_l);
    Brain.Screen.printAt(10, 40, "right:%f", speed_r);
    Brain.Screen.printAt(10, 60, "%f", now_dis - start_dis);
    Brain.Screen.printAt(10, 80, "%f", now_dis);
    Brain.Screen.printAt(10, 100, "%f", start_dis);
    Brain.Screen.printAt(10, 120, "finish");
    wait(20, msec);
}

void InitialGyro()
{
    Gyro.calibrate();   
    vex::task::sleep(2000);
}

void Move(int speed)
{
    LeftWheel(speed);
    RightWheel(speed);
}

void Rotate(int speed)
{
    LeftWheel(speed);
    RightWheel(-1*speed);
}

//////////////////////////////////////////////////////////////////////////
///////////////////////////////PID Controller/////////////////////////////
//////////////////////////////////////////////////////////////////////////

int OutPut_limit;
typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	int Error;
	int Integral;
	int Derivative;
	int LastError;
	int Output;
	bool insidetarget;
}TPID;

TPID PIDEncoder;
TPID PIDGyro;

void initialize_pid()
{
	PIDEncoder.Kp=0.155;
	PIDEncoder.Ki=0.01;
	PIDEncoder.Kd=0.01;
    
  PIDGyro.Kp=0.75;
  PIDGyro.Ki=0.03;
  PIDGyro.Kd=0.0;
}

void UpdatePIDController(TPID *PIDController, int Error,int tolerance)
{
    
	PIDController->Error = Error;

	if (abs(Error)<=tolerance) PIDController->insidetarget=true;
	else {PIDController->insidetarget=false; T2.clear();}

	if (abs(Error)<30 && Error!=0) PIDController->Integral += PIDController->Error;
	else PIDController->Integral=0;

	PIDController->Derivative = PIDController->Error - PIDController->LastError;
	PIDController->LastError  = PIDController->Error;

	PIDController->Output  =  (int)((float) PIDController->Error * PIDController->Kp)
	+   ((float)PIDController->Integral   * PIDController->Ki)
	+   ((float)PIDController->Derivative * PIDController->Kd);
}


void PIDForward(int MoveTarget,int MaxSpeed = 60,int tolerance=50)
{ 
  MoveTarget = -MoveTarget;
    int wait=500;
    TPID *PID = NULL;
    PID=&PIDEncoder;
    PID->insidetarget=false;
    T1.clear();
    T2.clear();
    TAcc.clear();
    rightMotor1.resetPosition();
    leftMotor1.resetPosition();
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    while(PID->insidetarget==false||T2.time()<100)
    {
        UpdatePIDController( PID,  MoveTarget - (rightMotor1.position(vex::rotationUnits::deg)+leftMotor1.position(vex::rotationUnits::deg))/2,tolerance);
        OutPut_limit=speedlimit(PID->Output,MaxSpeed);
        
        if(TAcc.time()<wait && MoveTarget>0)  //limit the acceleration
        {
            OutPut_limit=OutPut_limit*TAcc.time()/wait;
        }
        Move(OutPut_limit);
        vex::task::sleep(6);
        if (T1.time()>5000) break;
    }
    Controller1.Screen.print("goal=%d",MoveTarget);
    Controller1.Screen.newLine();
    Controller1.Screen.print("A=%2f",(rightMotor1.position(vex::rotationUnits::deg)+leftMotor1.position(vex::rotationUnits::deg))/2);
    Move(0);

}


void  PIDTurnTo(int RotateTarget,int MaxSpeed =100,float tolerance=1)
{     
  RotateTarget = -RotateTarget;
    TPID *PID=NULL;
    PID=&PIDGyro;
    PID->insidetarget=false;
    T1.clear();
    T2.clear();
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("PIDturnTo start......");
    while(PID->insidetarget==false||T2.time()<200)
    {
        UpdatePIDController( PID,  RotateTarget + Gyro.orientation(yaw,degrees),tolerance);
        OutPut_limit=speedlimit(PID->Output,MaxSpeed);
        Rotate(OutPut_limit);
        vex::task::sleep(10);//10
        if (T1.time()>5000) break;
    }
    Controller1.Screen.newLine();
    Controller1.Screen.print("goal=%d",RotateTarget);
    Controller1.Screen.newLine();  
    Controller1.Screen.print("angle=%.2f",Gyro.orientation(yaw,degrees));
    Move(0);
}
//////////////////////////////////////////////////////////////////////////
////////////////////////other/////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void store(){intake.spin(fwd, 100, pct);intake2.spin(fwd, 100, pct);Throw.spin(fwd, 20, pct);}
void down(){intake.spin(reverse, 100, pct);intake2.spin(reverse, 100, pct);Throw.spin(fwd, 91, pct);}
void upscore(){intake.spin(fwd, 100, pct);intake2.spin(fwd, 91, pct);Throw.spin(reverse, 91, pct);}
void midscore(){intake.spin(fwd, 100, pct);intake2.spin(fwd, 91, pct);Throw.spin(reverse, 30, pct);}

void user_intake() {
  if (Controller.ButtonR1.pressing()) {
    store();
  } else if (Controller.ButtonR2.pressing()) {
    down();
  } else if (Controller.ButtonL1.pressing()) {
    upscore();
  } else if (Controller.ButtonUp.pressing()) {
    midscore();
  } else{
    intake.stop(coast);
    intake2.stop(coast);
    Throw.stop(coast);}
}

void user_throw() {
  if (Controller.ButtonX.pressing()) {
    tai2.spin(fwd, 100, pct);
    tai1.spin(fwd, 100, pct);
  } else {
    if (Controller.ButtonA.pressing()) {
      tai2.spin(reverse, 100, pct);
    tai1.spin(reverse, 100, pct);
    } else {
      if (throw_rotation.angle(deg)<=max_throw_angle && throw_rotation.angle(deg)>=50) {
        tai1.stop(hold);
        tai2.stop(hold);
      } else {
        tai2.spin(fwd, 100, pct);
    tai1.spin(fwd, 100, pct);
      }
      
    }
  }
}

void user_import() {
  double dis = throw_starter.objectDistance(mm);
  if (Controller.ButtonA.pressing()) {
    if (throw_rotation.angle(deg)<=max_throw_angle && throw_rotation.angle(deg)>=50) {
      Throw.stop(hold);
    } else {
      Throw.spin(fwd, 100, pct);
    }
    if (dis <= 100) {
      wait(50, msec);
      Throw.spin(fwd, 100, pct);
      wait(250, msec);
    }
  }
}
void arm_control() {
  if (!Controller.ButtonB.pressing()) {
    Bpressed = false;
  }
  if (Controller.ButtonB.pressing() && !Bpressed) {
    if (arm_down) {
      lift_arm;
      arm_down = false;
      Bpressed = true;
    } else {
      drop_arm;
      arm_down = true;
      Bpressed = true;
    }
  }
}
void door_control() {
  if (!Controller.ButtonY.pressing()) {
    Ypressed = false;
  }
  if (Controller.ButtonY.pressing() && !Ypressed) {
    if (door_open) {
      close_door;
      door_open = false;
      Ypressed = true;
    } else {
      open_door;
      door_open = true;
      Ypressed = true;
    }
  }
}

void mod1()
{
    Arm.spinTo(0,deg);
}

int spinNumber = -33;

void mod2()
{
  if(Controller1.ButtonL1.pressing())
    {Arm.spin(reverse,100,pct);}
  else 
  {
    Arm.spinTo(spinNumber, deg);
  }
}

bool wing_control() 
{
  if (!Controller.ButtonLeft.pressing()) 
  {
    Leftpressed = false;
  }
  if (Controller.ButtonLeft.pressing() && !Leftpressed) 
  {
    if (wing_open) 
    {
      wing_open = false;
      Leftpressed=true;
      return wing_open;
    } 
    else 
    {
      wing_open = true;
      Leftpressed=true;
      return wing_open;
    }
  }
  return wing_open;
}

void auto_throw_prepare()
{
  Throw.spin(fwd,100,pct);
  waitUntil(throw_rotation.angle(deg)<=max_throw_angle &&throw_rotation.angle()<=350);
  Throw.stop(hold);
}

void user_auto_throw() {
  if (Controller.ButtonL1.pressing()) {
    base_stop(hold);
    intake.spin(fwd, 100, pct);
    wait(100, msec);
    open_door;
    wait(500, msec);
    lift_arm;
    wait(600, msec);
    stop_intake;
    wait(200, msec);
    drop_arm;
    wait(800, msec);
    Throw.spin(fwd, 100, pct);
    close_door;
    wait(200, msec);
    Throw.spin(fwd, 100, pct);
  }
}

void auto_throw() {
    base_stop(hold);
    intake.spin(fwd, 100, pct);
    wait(100, msec);
    open_door;
    wait(500, msec);
    lift_arm;
    wait(600, msec);
    stop_intake;
    wait(200, msec);
    drop_arm;
    wait(800, msec);
    Throw.spin(fwd, 100, pct);
    close_door;
    wait(200, msec);
    Throw.spin(fwd, 100, pct);
    Throw.spin(fwd,100,pct);
  waitUntil(throw_limit.pressing());
  Throw.stop(hold);
}
void rightAutonomous()
{
  intake.spin(fwd,60,pct);
  auto_throw_prepare();
  PIDForward(3390,90,5);
  start_intake;
  PIDTurnTo(90,100,3);
  Move(-100);
  wait(0.3,sec);
  re_intake;
  PIDTurnTo(-45,90,3);
  start_intake;
  PIDForward(1000,100);
  PIDTurnTo(90,45,3);
  Move(-100);
  wait(0.8, sec);
  re_intake;
  Move(100);
  wait(0.15, sec);
  PIDTurnTo(-90,100,3);
  start_intake;
  PIDForward(1300,100);
  PIDTurnTo(90,40,3);
  Move(-100);
  intake.spin(reverse,30,pct);
  wait(1.7, sec);
  Move(30);
  wait(0.4, sec);
}


void leftAutonomous_basic()
{
  intake.spin(fwd,10,pct);
  auto_throw_prepare();
  PIDForward(3150,90,5);
  PIDTurnTo(-90,100,3);
  open_wing;
  Move(100);
  wait(0.1,sec);
  Move(-100);
  wait(0.6,sec);
  PIDForward(-250,90,5);
  close_wing;
  PIDTurnTo(0,100,3);
  PIDForward(-3000,90,5);
  PIDTurnTo(100,100,3);
  re_intake;
  PIDForward(1600,90,1);
  PIDForward(-150,90,1);
}

void  leftPIDTurnTo(int RotateTarget,int MaxSpeed =100,int tolerance=1)
{     
  RotateTarget = -RotateTarget;
    TPID *PID=NULL;
    PID=&PIDGyro;
    PID->insidetarget=false;
    T1.clear();
    T2.clear();
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("PIDturnTo start......");
    while(PID->insidetarget==false||T2.time()<200)
    {
        UpdatePIDController( PID,  RotateTarget + Gyro.orientation(yaw,degrees),tolerance);
        OutPut_limit=speedlimit(PID->Output,MaxSpeed);
        LeftWheel(OutPut_limit);
        vex::task::sleep(10);//10
        if (T1.time()>3000) break;
    }
    Controller1.Screen.newLine();
    Controller1.Screen.print("goal=%d",RotateTarget);
    Controller1.Screen.newLine();  
    Controller1.Screen.print("angle=%.2f",Gyro.orientation(yaw,degrees));
    Move(0);
}


void left_new()
{
  open_wing;
  leftPIDTurnTo(-70);
  close_wing;
  PIDTurnTo(-150);
  start_intake;
  RightWheel(-100);
  LeftWheel(-100);
  wait(1,sec);
  Move(-50);
  wait(0.3,sec);
  PIDTurnTo(0);
  re_intake;
  PIDForward(200);
  PIDTurnTo(45);
  PIDForward(200);
}

void leftAutonomous_defence()
{
  start_intake;
  PIDForward(3400,70,1);
  PIDForward(-3200,60,1);
  PIDTurnTo(-10,100,3);
  Move(20);
  wait(0.2, sec);
  InitialGyro();
  PIDTurnTo(90,60,1);
  re_intake;
  PIDForward(2000,90,1);
  PIDForward(-150,90,1);
}
void leftAutonomous_defence_with_throw()
{ auto_throw_prepare();
  start_intake;
  PIDForward(3400,70,1);
  PIDForward(-600,60,1);
  PIDTurnTo(-135,100,3);
  auto_throw();
}

void tem()
{
  t_fl = leftMotor1.temperature(celsius);
  t_ml=leftMotor2.temperature(celsius);
  t_bl=leftMotor3.temperature(celsius);
  t_fr=rightMotor1.temperature(celsius);
  t_mr=rightMotor2.temperature(celsius);
  t_br=rightMotor3.temperature(celsius);
  t_intake=intake.temperature(celsius);
  t_throw=Throw.temperature(celsius);
  Brain.Screen.printAt(0, 20,  "fl");
  Brain.Screen.printAt(100, 20, "%3.1f", t_fl);

  Brain.Screen.printAt(0, 40,  "fr");
  Brain.Screen.printAt(100, 40, "%3.1f",t_fr);

  Brain.Screen.printAt(0, 60,  "ml");
  Brain.Screen.printAt(100, 60, "%3.1f", t_ml);

  Brain.Screen.printAt(0, 80,  "br");
  Brain.Screen.printAt(100, 80, "%3.1f", t_mr);

  Brain.Screen.printAt(0, 60,  "bl");
  Brain.Screen.printAt(100, 60, "%3.1f", t_bl);

  Brain.Screen.printAt(0, 80,  "br");
  Brain.Screen.printAt(100, 80, "%3.1f", t_br);

  Brain.Screen.printAt(0, 100,  "intake");
  Brain.Screen.printAt(100, 100, "%3.1f", t_intake);

  Brain.Screen.printAt(0, 120,  "throw");
  Brain.Screen.printAt(100, 120., "%3.1f", t_throw);
}



void fuck()
{
  if(Controller1.ButtonL2.pressing())
  {
    
    if(fucklast==false)
    {
      fucknumber++;
      if(fucknumber%2==1)
      {
        Fuck.set(0);
      }
      else 
      {
        Fuck.set(1);
      }
    }
      fucklast=true;
  }
  else 
  {
    fucklast=false;
  }
}
void primaryArm()
{
  if(Controller1.ButtonDown.pressing())
    Arm.spin(fwd,30,pct);
  else if(Controller1.ButtonUp.pressing())
    Arm.spin(reverse,30,pct);
  else if(Controller1.ButtonL2.pressing())
    Arm.spin(fwd,100,pct);
  else if (Controller1.ButtonL1.pressing())
    Arm.spin(reverse,100,pct);
  else
    Arm.stop(hold);
}

void seu_hook(){
  if(Controller.ButtonL2.pressing()){
    Fuck.set(0);
  }
  if(!Controller.ButtonL2.pressing()){
    Fuck.set(1);
  }
}

// ---------- PID参数 ----------
float sKp = 0.7, sKi = 0.04, sKd = 0.2;
float target_angle = 0.0;
float integral_error = 0.0, last_error = 0.0;
unsigned long last_time = 0;

// ---------- 距离控制参数 ----------
float target_distance = 0.0;       // 目标距离（毫米或厘米）
float wheel_circumference = 200.0; // 车轮周长（毫米），需实测
float start_left_angle = 0.0;
float start_right_angle = 0.0;

// 角度归一化
float normalize_angle(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

float constrain(float x, float min, float max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

// PID计算修正量
float pid_correction(float current_angle, float dt) {
    float error = normalize_angle(target_angle - current_angle);
    float P = sKp * error;
    integral_error += error * dt;
    integral_error = constrain(integral_error, -100.0, 100.0);
    float I = sKi * integral_error;
    float D = 0;
    if (dt > 0.001) D = sKd * (error - last_error) / dt;
    float output = constrain(P + I + D, -80.0, 80.0);
    last_error = error;
    return output;
}

// 获取当前已行驶距离（取左右轮平均转动角度换算）
float get_current_distance() {
    float left_angle = leftMotor2.position(deg) - start_left_angle;
    float right_angle = rightMotor2.position(deg) - start_right_angle;
    float avg_angle = (left_angle + right_angle) / 2.0;
    return (avg_angle / 360.0) * wheel_circumference;
}

// 直线运动（带距离自动停止）
// base_power:  正为前进，负为后退
// distance_mm: 目标距离（毫米），正数表示前进方向，负数表示后退方向
// 返回值： true=到达目标距离，false=未到达
bool straight_with_distance(int base_power, float distance_mm, float ang = Gyro.value()) {
  Brain.resetTimer();
  Brain.setTimer(0,msec);
    static bool init_flag = true;
    static float goal_angle_diff;
    unsigned long now = Brain.timer(msec);
    float dt = (now - last_time) / 1000.0;
    
    // 首次调用或目标方向改变时重置
    if (init_flag || target_distance != distance_mm) {
        target_distance = distance_mm;
        start_left_angle = leftMotor2.position(deg);
        start_right_angle = rightMotor2.position(deg);
        target_angle = ang;           // 锁定初始方向
        integral_error = 0.0;
        last_error = 0.0;
        last_time = now;
        init_flag = false;
        
        // 计算目标角度差（用于距离判断，单位度）
        float goal_rotations = distance_mm / wheel_circumference;
        goal_angle_diff = goal_rotations * 360.0;
        return false;
    }
    
    if (dt <= 0.001) dt = 0.02;
    
    // 距离到达检查（方向一致：前进时平均角度差>=目标，后退时<=目标）
    float avg_angle = ( (leftMotor2.position(deg) - start_left_angle) + 
                        (rightMotor2.position(deg) - start_right_angle) ) / 2.0;
    bool reached = false;
    if (distance_mm > 0) {
        reached = (avg_angle >= goal_angle_diff);
    } else {
        reached = (avg_angle <= goal_angle_diff);
    }
    
    if (reached) {
        base_stop(brake);
        init_flag = true;   // 下次调用重新初始化
        return true;
    }
    
    // 未到达：执行PID航向修正
    float current_angle = Gyro.value();
    float correction = pid_correction(current_angle, dt);
    int left_power = base_power - correction;
    int right_power = base_power + correction;
    left_power = constrain(left_power, -100, 100);
    right_power = constrain(right_power, -100, 100);
    LeftWheel(left_power);
    RightWheel(right_power);
    
    last_time = now;
    return false;
}

void loopfwd(int speed, int tar, double ang = Gyro.value()) {
  Brain.setTimer(0, msec);
    while (!straight_with_distance(speed, tar, ang)) {
        // 在等待完成期间，可以执行其他非阻塞任务
        wait(10,msec);  // 控制调用频率，避免占用过多 CPU
    }
    base_stop(brake);
    PIDTurnTo(target_angle,40,1);//转回原本朝向
    //Controller1.Screen.newLine();
    //Controller1.Screen.print("%2f,%2f",(lefm,rigm));
}

// 时间控制的直线运动
// speed: 电机基础功率（-100~100），正前进/负后退
// duration_ms: 运动持续时间（毫秒）
// 返回值：true=时间到达已停止，false=运动中


//////////////////////////////////////////////////////////////////////////
/////////////////////////////////Autonomous///////////////////////////////
//////////////////////////////////////////////////////////////////////////

void pre_auton(void) {
  vexcodeInit();
  initialize_pid();
}

void fuckCSTU(){
  Fuck.set(0);
  wing.set(0);
  loopfwd(-50,-750, 0);
  store();
  PIDTurnTo(91,50,1);
  store();
  //loopfwd(80,-500,90);
  Move(80);
  wait(500,msec);
  Move(0);
  wait(1800, msec);
  //PIDTurnTo(90,50,0.5);
  loopfwd(-80,-500, 90);
  Move(-50);
  wait(500, msec);
  Move(0);
  upscore();
  wait(1000,msec);
  
  loopfwd(50,380);
  Move(80);
  wait(800,msec);
  Move(0);
  store();
  wait(3,sec);
  PIDTurnTo(92,50,1);
  //Fuck.set(1);
  //wait(1,sec);
  //Fuck.set(0);
  loopfwd(-70,-500, 90);
  Move(-50);
  wait(500, msec);
  Move(0);
  upscore();
  wait(800,msec);
  store();

  loopfwd(80,500,90);
  door.set(1);
  PIDTurnTo(135);
  loopfwd(-80,-1800,135);
  loopfwd(40, 300, 135);
  midscore();
  wait(1,sec);
  loopfwd(60, 800, 135);
  PIDTurnTo(90,50,1);
  Fuck.set(0);
  loopfwd(-60,-600,90);
  LeftWheel(-80);
  RightWheel(80);
  wait(500,msec);
  base_stop(brake);
  intake.stop();
  intake2.stop();
  Throw.stop();
}

void autonomous(void) {
 /////////////按照预期放入两个
  fuckCSTU();
  //PIDTurnTo(90);
}

void fix()
{
  if(Controller1.ButtonL2.pressing())
  {
    Arm.spinFor(fwd, 10, deg);
  }
}

//////////////////////////////////////////////////////////////////////////
////////////////////////////////User Control//////////////////////////////
//////////////////////////////////////////////////////////////////////////

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) { 
    user_move();
    user_intake();
    seu_hook();
    //user_throw();
    //user_import();
    arm_control();
    door_control();
    wing_control();
    //fuck();
    //primaryArm();
    //if(!wing_open)
      //mod1();
    //else mod2();
   // user_auto_throw();
    tem();
    wait(20, msec);
  }
}



//////////////////////////////////////////////////////////////////////////
////////////////////////////////////Main//////////////////////////////////
//////////////////////////////////////////////////////////////////////////
int main() {
  pre_auton();
  Gyro.calibrate();
  vex::task::sleep(2000);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  //throw_rotation.setPosition(0, deg);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.

  while (true) {
    wait(50, msec);
  }
}
