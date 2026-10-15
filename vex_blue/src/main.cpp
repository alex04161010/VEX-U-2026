
#include "vex.h"
#define Controller Controller1
#define base_stop(x) base_drive(0, 0, 0, x)
// extern distance back_left;
// extern distance back_right;距离传感器没有加的时候先注释掉
extern inertial inertial_right; 
using namespace vex;

// 上左气动变量
bool top_left_extended = false;
bool Leftpressed = false;

// 上右气动变量
bool top_right_extended = false;
bool Rightpressed = false;

//下气动变量
bool arm_down = false;    // 记录气动状态：false为收回，true为伸出
bool Downpressed = false;  // 记录按键状态，防止按住不放时反复切换

bool Ypressed = false;
int Reverse = -1; 
bool Xpressed=false;
//////////////////////////////////////////////////////////////////////////
///////////////////////////////Basic Functoin/////////////////////////////
/////////////////////////////////////////////////////////////////////////
competition Competition;
timer T1,T2,TAcc;
double max_l = 0, max_r = 0;
void base_drive(double left_speed, double right_speed, double max_rpm = 600,
                brakeType type = coast){
  double speed_l = left_speed, speed_r = right_speed;
  speed_l = speed_l / 100.0 * max_rpm;
  speed_r = speed_r / 100.0 * max_rpm;
  if (speed_l != 0) {
    leftMotor1.spin(reverse, speed_l, rpm);
    leftMotor2.spin(reverse, speed_l, rpm);
    leftMotor3.spin(reverse, speed_l, rpm);
    leftMotor4.spin(reverse, speed_l, rpm);
  }
  if (speed_r != 0) {
    rightMotor1.spin(reverse, speed_r, rpm);
    rightMotor2.spin(reverse, speed_r, rpm);
    rightMotor3.spin(reverse, speed_r, rpm);
    rightMotor4.spin(reverse, speed_r, rpm);
  }
  if (speed_l == 0) {
    leftMotor1.stop(type);
    leftMotor2.stop(type);
    leftMotor3.stop(type);
    leftMotor4.stop(type);
  }
  if (speed_r == 0) {
    rightMotor1.stop(type);
    rightMotor2.stop(type);
    rightMotor3.stop(type);
    rightMotor4.stop(type);
  }
  double rpm_la = leftMotor1.velocity(rpm), rpm_lb = leftMotor2.velocity(rpm),
         rpm_lc = leftMotor3.velocity(rpm), rpm_ld = leftMotor4.velocity(rpm),
         rpm_ra = rightMotor1.velocity(rpm),rpm_rb = rightMotor2.velocity(rpm), 
         rpm_rc = rightMotor3.velocity(rpm), rpm_rd = rightMotor4.velocity(rpm);

  if (((rpm_la + rpm_lb + rpm_lc + rpm_ld) / 4.0) > max_l) {
    max_l = (rpm_la + rpm_lb + rpm_lc + rpm_ld) / 4.0;
  }

  if (((rpm_ra + rpm_rb + rpm_rc + rpm_rd) / 4.0) > max_r) {
    max_r = (rpm_ra + rpm_rb + rpm_rc + rpm_rd) / 4.0;
  }

  Brain.Screen.printAt(100, 180, "%f", max_l);
  Brain.Screen.printAt(300, 180, "%f", max_r);
}
//用户一键切换方向
void user_move() {
  if (!Controller.ButtonX.pressing()) {
    Xpressed = false;
  }
  if (Controller.ButtonX.pressing() && !Xpressed)
   { Reverse *= -1;
    Xpressed = true;}


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
  base_drive(fwd_ - turn_, fwd_ +turn_, 600);
}
//电机速度为负数，则电机反转的定义
int sign(int speed)
{
    if(speed>0) return 1;
    else if(speed<0) return -1;
    else return 0;
}
//限制电机速度最大为100
int speedlimit(int speed,int max=100)
{
    return abs(speed)>max ? sign(speed)*max : speed;
}
//左轮速度
void LeftWheel(int speed)
{
  leftMotor1.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  leftMotor2.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  leftMotor3.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  leftMotor4.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
}
//右轮速度
void RightWheel(int speed)
{
  rightMotor1.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  rightMotor2.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  rightMotor3.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
  rightMotor4.spin(vex::directionType::fwd,speed,vex::velocityUnits::pct);
}
// //距离传感器的前进（目前没有装距离传感器，前进纠偏）
// void Forward(double dis, double speed) {
//   double left, right, start_dis, target_dis,
//       start_slow_k = 0.3, start_mis = 5, fix_k = 0.2, basic_k = 0.8, corr = 5;
//   left = back_left.objectDistance(mm);
//   right = back_right.objectDistance(mm);
//   start_dis = (left + right) / 2;
//   target_dis = start_dis + dis;
//   double now_dis = 0;
//   double despeed = 0;
//   double basic_speed;
//   while (1) {
//     left = back_left.objectDistance(mm);
//     right = back_right.objectDistance(mm);
//     now_dis = (left + right) / 2;
//     if (fabs(left - right) > start_mis) {
//       despeed = -(left - right - start_mis) * fix_k;
//     }
//     if (now_dis <= target_dis) {
//       if (now_dis >= start_slow_k * dis + start_dis) {
//         basic_speed = (target_dis - now_dis) / dis * basic_k * speed + corr;
//       } else {
//         basic_speed = speed;
//       }
//     }
//     double speed_l, speed_r;
//     if (basic_speed + despeed <= 3) {
//       speed_l = 0;
//     } else {
//       speed_l = basic_speed + despeed;
//     }
//     if (basic_speed - despeed <= 3) {
//       speed_r = 0;
//     } else {
//       speed_r = basic_speed - despeed;
//     }
//     if ((speed_l <= 0 && speed_r <= 0) || now_dis - start_dis > dis - 10) {
//       base_stop(hold);
//       break;
//     } else {
//       base_drive(speed_l, speed_r);
//     }
//     Brain.Screen.clearScreen();
//     Brain.Screen.printAt(10, 20, "left :%f", left);
//     Brain.Screen.printAt(10, 40, "right:%f", right);
//     Brain.Screen.printAt(10, 60, "%f", now_dis - start_dis);
//     Brain.Screen.printAt(10, 80, "%f", basic_speed);
//     Brain.Screen.printAt(10, 100, "%f", despeed);
//     wait(20, msec);
//   }
// }
//暴力按照时间跑
void move_with_time(double time_ms ,double speed)
{
  base_drive(speed, speed);
  wait(time_ms, msec);
  base_stop(coast);
}
//基于距离传感器的后退
// void Backward(double dis, double speed) {
//   double left, right, start_dis, target_dis,
//       start_slow_k = 0.3, start_mis = 5, fix_k = 0.2, basic_k = 0.8, corr = 5;
//   left = back_left.objectDistance(mm);
//   right = back_right.objectDistance(mm);
//   start_dis = (left + right) / 2;
//   target_dis = start_dis - dis;
//   double now_dis = 0;
//   double despeed = 0;
//   double basic_speed;
//   double speed_l, speed_r;
//   while (1) {
//     left = back_left.objectDistance(mm);
//     right = back_right.objectDistance(mm);
//     now_dis = (left + right) / 2;
//     if (fabs(left - right) > start_mis) {
//       despeed = -(left - right - start_mis) * fix_k;
//     }
//     if (now_dis >= target_dis) {
//       if (now_dis <=start_dis- start_slow_k * dis  ) {
//         basic_speed = (target_dis - now_dis) / dis * basic_k * speed - corr;
//       } else {
//         basic_speed = -speed;
//       }
//     }
//     if (basic_speed + despeed <= 3) {
//       speed_l = 0;
//     } else {
//       speed_l = basic_speed + despeed;
//     }
//     if (basic_speed - despeed <= 3) {
//       speed_r = 0;
//     } else {
//       speed_r = basic_speed - despeed;
//     }
//     if ((speed_l >= 5 && speed_r >= 5) || fabs(now_dis - start_dis) > dis - 5) {
//       base_stop(hold);
//       break;
//     } else {
//       base_drive(speed_l, speed_r);
//     }
//     Brain.Screen.clearScreen();
//     Brain.Screen.printAt(10, 20, "left :%f", speed_l);
//     Brain.Screen.printAt(10, 40, "right:%f", speed_r);
//     Brain.Screen.printAt(10, 60, "%f", now_dis - start_dis);
//     Brain.Screen.printAt(10, 80, "%f", basic_speed);
//     Brain.Screen.printAt(10, 100, "%f", despeed);
//     wait(20, msec);
//   }
//   Brain.Screen.clearScreen();
//     Brain.Screen.printAt(10, 20, "left :%f", speed_l);
//     Brain.Screen.printAt(10, 40, "right:%f", speed_r);
//     Brain.Screen.printAt(10, 60, "%f", now_dis - start_dis);
//     Brain.Screen.printAt(10, 80, "%f", now_dis);
//     Brain.Screen.printAt(10, 100, "%f", start_dis);
//     Brain.Screen.printAt(10, 120, "finish");
//     wait(20, msec);
// }
//校准陀螺仪
void InitialGyro()
{
    Gyro.calibrate();   
    vex::task::sleep(2000);
}
//基础的移动
void Move(int speed)
{
    LeftWheel(speed);
    RightWheel(speed);
}
//原地（定点）旋转
void Rotate(int speed)
{
    LeftWheel(speed);
    RightWheel(-1*speed);
}
//////////////////////////////////////////////////////////////////////////
////////////////////////PID/////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// 实例化全局变量
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
	PIDEncoder.Kp=0.15;
	PIDEncoder.Ki=0.01;
	PIDEncoder.Kd=0.1;
    
  PIDGyro.Kp=0.50;
  PIDGyro.Ki=0.001;
  PIDGyro.Kd=0.1;
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
void  PIDTurnTo(int RotateTarget,int MaxSpeed =100,int tolerance=1)
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
void arm_control() { //下气动，铲子
  // 1. 如果没有按下 ButtonDown，重置状态标记位
  if (!Controller1.ButtonDown.pressing()) {
    Downpressed = false; 
  }
  // 2. 如果按下 ButtonDown且之前没有被记录为“已按下”
  if (Controller1.ButtonDown.pressing() && !Downpressed) {
    if (arm_down) {
      // 执行动作：收回/抬起 arm
      arm.set(false);      // 使用 robot-config.cpp 中定义的 digital_out arm
      arm_down = false;    // 更新状态变量
      Downpressed = true;    // 标记本次按键已处理
    } else {
      // 执行动作：伸出/放下 arm
      arm.set(true);       // 调用 V5 气动 set 方法
      arm_down = true;     // 更新状态变量
      Downpressed = true;    // 标记本次按键已处理
    }
  }
}
void top_arm_control() {
    // --- 上右气动逻辑 (绑定右键) ---
    if (!Controller1.ButtonRight.pressing()) {
        Rightpressed = false; 
    }
    if (Controller1.ButtonRight.pressing() && !Rightpressed) {
        if (top_right_extended) {
            top_arm_right.set(false); // 收回
            top_right_extended = false;
        } else {
            top_arm_right.set(true);  // 伸出
            top_right_extended = true;
        }
        Rightpressed = true; 
    }
}



// 统一控制四个电机的方向
// 1 代表 forward, -1 代表 reverse, 0 代表停止
void intake_move(double p1, double p2, double p3, double p4, double p5, double p6, double p7) {
    
    // 使用 spin 的电压百分比模式，正数前进，负数后退，0 停止
    intake_upfr.spin(forward, p1, percent);
    intake_downfr.spin(forward, p2, percent);
    intake_upbk.spin(forward, p3, percent);
    intake_downbk.spin(forward, p4, percent);
    intake_power.spin(forward, p5, percent);
    intake_secondtopfr.spin(forward, p6, percent);
    intake_topfr.spin(forward, p7, percent);
}
// 按键逻辑映射，后续还要详细修改正反转组合
void intake_control() {
  ColorSensor.setLightPower(20);
    if (Controller.ButtonR1.pressing()) 
    {
        // --- 核心分色分支 ---
        // 如果感应到物体，且（颜色极小 或 颜色极大），则判定为红色
    if (ColorSensor.isNearObject() && (ColorSensor.hue() <20|| ColorSensor.hue()>340)) {
    // 剔除敌方红色球模式
    intake_move(100, 100, 100, -100, 100, 100, -100); 
    }
    else {
    // 正常吸球模式
    intake_move(100, 100, 100, -100, -100, 100, 100); 
    }

    } 
    else if (Controller.ButtonR2.pressing()) {
        // 模式：吐下层 (此处示例方向，需实测)
        intake_move(100, -100, 100, 100, 100, 100, 100); 
    } 
    else if (Controller.ButtonL1.pressing()) {
        // 模式：吐上层 (此处示例方向，需实测)
        intake_move(100, 100, 100, 100, 100 ,-100 ,100); 
    }
    else if (Controller.ButtonL2.pressing()) {
        // 模式：吐中层 (此处示例方向，需实测)
        intake_move(-100, 100, 100, 100, 100, 100, 100); 
    }
    else {
        // 没有任何按键按下，全部停止
        intake_move(0, 0, 0, 0,0,0,0);
    }
}

//左轮单边转（PID）
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
//测温函数（防止电机瘫痪）
// 在 src/main.cpp 的第 404 行左右进行修改
double t_fl = leftMotor1.temperature(celsius);
double t_ml = leftMotor2.temperature(celsius);
double t_bl = leftMotor3.temperature(celsius);
double t_l4 = leftMotor4.temperature(celsius);
double t_fr = rightMotor1.temperature(celsius);
double t_mr = rightMotor2.temperature(celsius);
double t_br = rightMotor3.temperature(celsius);
double t_r4 = rightMotor4.temperature(celsius);
void tem()
{
  t_fl = leftMotor1.temperature(celsius);
  t_ml=leftMotor2.temperature(celsius);
  t_bl=leftMotor3.temperature(celsius);
  t_l4 = leftMotor4.temperature(celsius);
  t_fr=rightMotor1.temperature(celsius);
  t_mr=rightMotor2.temperature(celsius);
  t_br=rightMotor3.temperature(celsius);
  t_r4 = rightMotor4.temperature(celsius);
  Brain.Screen.printAt(0, 20,  "L1: %3.0f", t_fl);
  Brain.Screen.printAt(0, 40,  "L2: %3.0f", t_ml);
  Brain.Screen.printAt(0, 60,  "L3: %3.0f", t_bl);
  Brain.Screen.printAt(0, 80,  "L4: %3.0f", t_l4);
  // 右侧电机显示在 x=120
  Brain.Screen.printAt(120, 20, "R1: %3.0f", t_fr);
  Brain.Screen.printAt(120, 40, "R2: %3.0f", t_mr);
  Brain.Screen.printAt(120, 60, "R3: %3.0f", t_br);
  Brain.Screen.printAt(120, 80, "R4: %3.0f", t_r4);

  Brain.Screen.printAt(240, 40, "hue: %3.0f", ColorSensor.hue());
}
//////////////////////////////////////////////////////////////////////////
/////////////////////////////////Autonomous///////////////////////////////
//////////////////////////////////////////////////////////////////////////
//检查站函数
void pre_auton(void) {
  vexcodeInit();
  initialize_pid();
  //PIDForward(1000,60,20);
}
//////////////////////////////////////////////////////////////////////////
////////////////////////////////User Control//////////////////////////////
//////////////////////////////////////////////////////////////////////////
void usercontrol(void)
{
  while (1) { 
    intake_control(); // 吸取控制（四个模式）
    top_arm_control();
    arm_control();
    user_move();//移动
    tem();//测温
    wait(20, msec);
  }
}
//////////////////////////////////////////////////////////////////////////
////////////////////////////////////自动//////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void autonomous(void) {
    // 确保 PID 参数已初始化
    initialize_pid();
    // 1. 后退一段距离 (假设 500 度)
    // 逻辑：MoveTarget 变号后，如果传正数则车向后退（取决于你 Move 函数的写法）
    // 建议：如果 PIDForward(500) 是前进，那么后退就是 -500
    ColorSensor.setLightPower(70);
    arm.set(false);   
    PIDForward(-1350, 50, 20); 
    wait(100, msec); // 动作间的缓冲，让车停稳
    // 2. 左转 (假设 90 度)
    // 逻辑：根据你的 PIDTurnTo，RotateTarget 变号后与 Gyro 比较
    // 建议：通常向左转为正（如 90），向右转为负（如 -90）
    PIDTurnTo(-90, 50, 1); 
    wait(100, msec);
    T1.clear(); 
    int auto_time_ms = 2000; // 设置吸球持续时间为 6.5 秒
    while(T1.time() < auto_time_ms) {
        // --- 核心分色逻辑 ---
        // 如果感应到物体且颜色是蓝色（自己是红色）
        if (ColorSensor.isNearObject() && (ColorSensor.hue() <20||ColorSensor.hue() >340)) {
           // 剔除敌方红色球
            intake_move(100, 100, 100, -100, 100, 100, -100); 
        } 
        else {
            // 正常吸球
            intake_move(100, 100, 100, -100, -100, 100, 100); 
        }
        Move(-25);
      }
      while(T1.time() < 2*auto_time_ms) {
        // --- 核心分色逻辑 ---
        // 如果感应到物体且颜色是蓝色（自己是红色）
        if (ColorSensor.isNearObject() && (ColorSensor.hue() >130 && ColorSensor.hue() <250)) {
           // 剔除敌方红色球
            intake_move(100, 100, 100, -100, 100, 100, -100); 
        } 
        else {
            // 正常吸球
            intake_move(100, 100, 100, -100, -100, 100, 100); 
        }
        //Move(-30);
      }



     // intake_move(100, 100, 100, -100, -100, 100, 100); 
      PIDForward(-350, 70, 20);
      arm.set(true);   
      //wait(50, msec);
      PIDTurnTo(90, 50, 1); 

      PIDForward(625, 35, 40);  
      Move(-10);
      wait(200, msec);
      intake_move(100, 100, 100, 100, 100 ,-100 ,100);//导入上层
      wait(500, msec);
      Move(0); 
      wait(5000,msec);
      intake_move(0,0,0,0,0,0,0); 
      PIDForward(-700, 50, 40); 
      PIDTurnTo(45, 50, 1); 

     // PIDForward(2200, 70, 40);
     // intake_move(100, -100, 100, 100, 100, 100, 100); //导入下层
     // wait(1300,msec);
    //  PIDForward(-1230, 35, 40); 
      PIDForward(750,40,10);
      PIDTurnTo(90, 40, 1); 
      top_arm_right.set(true);
      PIDForward(900, 50, 80); 
            
}
//////////////////////////////////////////////////////////////////////////
////////////////////////////////////Main//////////////////////////////////
//////////////////////////////////////////////////////////////////////////
int main() {
  Gyro.calibrate();
  vex::task::sleep(2000);
  pre_auton();
  
 Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(50, msec);
  }
}
