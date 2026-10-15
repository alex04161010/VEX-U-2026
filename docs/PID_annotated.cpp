//////////////////////////////////////////////////////////////////////////
///////////////////////////////PID Controller/////////////////////////////
//////////////////////////////////////////////////////////////////////////
// 两个 PID 环：一个读轮子编码器管直行，一个读陀螺仪管转向。
// 结构都一样，就是误差来源和参数不同。

int OutPut_limit;          // 全局变量，装限速之后的输出，调试的时候方便直接看

typedef struct
{
	float Kp;              // 比例
	float Ki;              // 积分
	float Kd;              // 微分
	int Error;             // 当前误差
	int Integral;          // 误差累加（积分项）
	int Derivative;        // 本次误差 - 上次误差
	int LastError;         // 上一轮的误差
	int Output;            // PID 算出来的输出
	bool insidetarget;     // 是否已经进到容差范围内
}TPID;
// 提一句：误差、积分、输出全用 int 存，能跑，但小数部分基本都丢了。
// 离目标 1 度以内的时候 Kp*1 还是不到 1，截成 int 就是 0，最后那段残差基本靠积分慢慢磨。

TPID PIDEncoder;           // 直行用的那套参数
TPID PIDGyro;              // 转向用的那套参数

void initialize_pid()
{
	// 直行的参数。这几个数是实际试出来的，换车换场地都要重调，别照抄。
	PIDEncoder.Kp=0.155;
	PIDEncoder.Ki=0.01;
	PIDEncoder.Kd=0.01;

  // 转向的参数。转向因为陀螺仪读数比较干净，Kp 可以给大不少，Kd 干脆不要。
  PIDGyro.Kp=0.75;
  PIDGyro.Ki=0.03;
  PIDGyro.Kd=0.0;
}

// 两个环共用的计算函数。传进来当前误差和容差，把结果写进结构体里。
void UpdatePIDController(TPID *PIDController, int Error,int tolerance)
{

	PIDController->Error = Error;

	// 进容差范围就算到位，出界就置回 false，并且顺手把 T2 清零。
	// 所以 T2 量的其实是"从上次跑出界到现在，已经连续稳了多久"。
	if (abs(Error)<=tolerance) PIDController->insidetarget=true;
	else {PIDController->insidetarget=false; T2.clear();}

	// 离目标还远（超过 30）的时候不攒积分，免得冲过头再往回拉太狠，典型的积分分离。
	// 误差正好等于 0 也清零，不然积分会在 0 附近来回蹭。
	if (abs(Error)<30 && Error!=0) PIDController->Integral += PIDController->Error;
	else PIDController->Integral=0;

	// 微分项用的是"误差的变化量"，不是测量值的变化量。
	PIDController->Derivative = PIDController->Error - PIDController->LastError;
	PIDController->LastError  = PIDController->Error;

	// 三个项加起来，最后截断成 int 丢给电机。
	PIDController->Output  =  (int)((float) PIDController->Error * PIDController->Kp)
	+   ((float)PIDController->Integral   * PIDController->Ki)
	+   ((float)PIDController->Derivative * PIDController->Kd);
}


// 直行 MoveTarget 个单位（轮子转过的角度），最多 MaxSpeed，误差 50 以内算到位。
void PIDForward(int MoveTarget,int MaxSpeed = 60,int tolerance=50)
{ 
  // 目标取反。因为电机实际装的方向跟这里用的坐标系是反的，不取反车就往后跑。
  MoveTarget = -MoveTarget;
    int wait=500;              // 起步加速用的斜坡时长（毫秒）
    TPID *PID = NULL;
    PID=&PIDEncoder;           // 直行就用编码器这套
    PID->insidetarget=false;   // 强制先跑一轮，不然一上来就满足条件会直接跳过
    T1.clear();                // T1：总超时计时
    T2.clear();                // T2：到位后稳定计时
    TAcc.clear();              // TAcc：加速斜坡计时
    rightMotor1.resetPosition();   // 编码器归零，误差是相对这次起点算的
    leftMotor1.resetPosition();
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    // 没到位、或者到位了但还没稳够 100ms，就继续跑
    while(PID->insidetarget==false||T2.time()<100)
    {
        // 误差 = 目标 - 两个轮子转角的平均值。取平均是为了避免一边轮子打滑就把车带歪。
        UpdatePIDController( PID,  MoveTarget - (rightMotor1.position(vex::rotationUnits::deg)+leftMotor1.position(vex::rotationUnits::deg))/2,tolerance);
        OutPut_limit=speedlimit(PID->Output,MaxSpeed);   // speedlimit 是外面写的，就是把输出卡在 ±MaxSpeed

        if(TAcc.time()<wait && MoveTarget>0)  //limit the acceleration
        {
            // 起步 500ms 内按时间比例慢慢把速度放出去，相当于软启动，防打滑也防冲。
            // 注意只对取反后为正的那个方向生效，另一边是没有斜坡的。
            OutPut_limit=OutPut_limit*TAcc.time()/wait;
        }
        Move(OutPut_limit);
        vex::task::sleep(6);   // 直行的循环周期 6ms
        if (T1.time()>5000) break;   // 兜底：5 秒还没到就直接放弃，别卡死
    }
    // 下面两行是打到手柄屏幕上看的调试信息，正式跑可以留也可以删
    Controller1.Screen.print("goal=%d",MoveTarget);
    Controller1.Screen.newLine();
    Controller1.Screen.print("A=%2f",(rightMotor1.position(vex::rotationUnits::deg)+leftMotor1.position(vex::rotationUnits::deg))/2);
    Move(0);   // 停下来

}


// 转到 RotateTarget 这个角度（相对现在的朝向），容差 1 度。
void  PIDTurnTo(int RotateTarget,int MaxSpeed =100,float tolerance=1)
{     
  RotateTarget = -RotateTarget;   // 同样取反，理由跟直行那边一样
    TPID *PID=NULL;
    PID=&PIDGyro;                 // 转向用陀螺仪这套
    PID->insidetarget=false;
    T1.clear();
    T2.clear();
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("PIDturnTo start......");
    // 同样，到位后再稳 200ms 才放行
    while(PID->insidetarget==false||T2.time()<200)
    {
        // 这里是加号，不是减号。因为 Rotate 正转和陀螺仪 yaw 的正方向是反的，
        // 加号才把两边统一起来。要是发现车往反方向转，第一个要查的就是这一行。
        UpdatePIDController( PID,  RotateTarget + Gyro.orientation(yaw,degrees),tolerance);
        OutPut_limit=speedlimit(PID->Output,MaxSpeed);
        Rotate(OutPut_limit);
        vex::task::sleep(10);//10  转向的循环周期放到 10ms，比直行慢一点
        if (T1.time()>5000) break;   // 同样 5 秒超时
    }
    Controller1.Screen.newLine();
    Controller1.Screen.print("goal=%d",RotateTarget);
    Controller1.Screen.newLine();  
    Controller1.Screen.print("angle=%.2f",Gyro.orientation(yaw,degrees));
    Move(0);
    // 这里停的是 Move 不是 Rotate。要是你们底盘上这俩控制的是同一组电机，那没事；
    // 要是分开的，转完这句其实停不住，得补一句 Rotate(0)。
}
