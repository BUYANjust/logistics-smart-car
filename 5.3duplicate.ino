#include <AccelStepper.h>
#include <AFMotor.h>
#include <MsTimer2.h>
#include <Servo.h> 
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_TCS34725.h>

LiquidCrystal_I2C lcd(0x27,16,2); //屏幕配置

  unsigned char MotorL_Dir_Pin = 2; //步进电机引脚设置
  unsigned char MotorL_Stp_Pin = 3;
  unsigned char MotorR_Dir_Pin = 5;
  unsigned char MotorR_Stp_Pin = 4;

  unsigned char Motor_Resolution = 2; 

  AccelStepper stepper1(1,MotorL_Stp_Pin,MotorL_Dir_Pin);  //AccelStepper步进电机库 电机设置
  AccelStepper stepper2(1,MotorR_Stp_Pin,MotorR_Dir_Pin);

  unsigned char Servo_Plate_Pin             = A3; //舵机引脚设置
  unsigned char Servo_Arm_Pin               = A2;
  unsigned char Servo_Left_Claw_Pin         = A1;
  unsigned char Servo_Right_Claw_Pin        = A0;

  //unsigned char Linetracker_Left_Wing_Pin    = 13; //光电巡线引脚设置
  unsigned char Linetracker_Left_Pin         = 12;
  unsigned char Linetracker_Left_Middle_Pin  = 11;
  unsigned char Linetracker_Center_Left_Pin  = 10;
  unsigned char Linetracker_Center_Right_Pin = 9;
  unsigned char Linetracker_Right_Middle_Pin = 8;
  unsigned char Linetracker_Right_Pin        = 7;
  //unsigned char Linetracker_Right_Wing_Pin   = 6;
int sen1 ;
int sen2 ;
int sen3 ;
int sen4 ;
int sen5 ;
int sen6 ;

int Linetracker1_Pin_Trigger_Time  = 0;   //从车前进的方向看，从左往右看光电第一个为1.
int Linetracker1_Pin_Untrigger_Time  = 0;

int Linetracker_Left_Middle_Trigger_Time  = 0;
int Linetracker_Left_Middle_Untrigger_Time  = 0;

int Linetracker_Center_Left_Trigger_Time  = 0;
int Linetracker_Center_Left_Untrigger_Time  = 0;

int Linetracker_Center_Right_Trigger_Time  = 0;
int Linetracker_Center_Right_Untrigger_Time  = 0;
    
int Linetracker_Right_Middle_Trigger_Time  = 0;
int Linetracker_Right_Middle_Untrigger_Time  = 0;

int Linetracker_Right_Trigger_Time  = 0;
int Linetracker_Right_Untrigger_Time  = 0;

  // int Linetracker_Right_Wing_Trigger_Time  = 0;
  // int Linetracker_Right_Wing_Untrigger_Time  = 0;

Servo myservo_left;
Servo myservo_arm;
Servo myservo_right;
Servo myservo_circle;

int pos_left;
int pos_right;
int pos0;
int pos1;
int pos4;
int pos5;
int loopnum=0;
int speed=200;

float step=1;
int step2;

int advancestep=10;
int turnstep=10;
int goal1=230;
int goal2=200;

int turn_adjust_ok=0;

unsigned char Line_Count_start =0; //数线开关
int Line_Count =0; //数线结果
int Line_Count_Time = 0; // 时间临时存储变量
int Count_Holdtime  = 20; //多少毫秒内两个都识别到判断为数到线
int Line_Delaytime = 100;  //数到线后多少毫秒后再进行数线
int Line_Count_Busy = 0; //数线程序忙碌状态

unsigned char Linetracker_Left_State=1;
unsigned char Linetracker_Right_State = 0;


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
int green=0;
int white=0;
int red=0;
int black=0;
int blue=0;
int color;

void setup()    //注，该函数需要加在主函数的setup中
{
  // lcd.init(); //初始化LCD
  // lcd.backlight(); //打开背光

  stepper1.setMaxSpeed(1000*Motor_Resolution);   // 设置电机最大速度为2000
  stepper1.setAcceleration(200*Motor_Resolution);// 设置电机加速度
  stepper2.setMaxSpeed(1000*Motor_Resolution);   
  stepper2.setAcceleration(200*Motor_Resolution);

  //LineFollow_setup(); 
  pinMode(MotorL_Dir_Pin, OUTPUT);   //电机引脚模式设置
  pinMode(MotorL_Stp_Pin, OUTPUT);
  pinMode(MotorL_Dir_Pin, OUTPUT);  
  pinMode(MotorL_Stp_Pin, OUTPUT);

//  pinMode(Linetracker_Left_Wing_Pin, INPUT_PULLUP);  //光电引脚模式设置
  pinMode(Linetracker_Left_Pin, INPUT_PULLUP);
  pinMode(Linetracker_Left_Middle_Pin, INPUT_PULLUP);
  pinMode(Linetracker_Center_Left_Pin, INPUT_PULLUP);
  pinMode(Linetracker_Center_Right_Pin, INPUT_PULLUP);
  pinMode(Linetracker_Right_Middle_Pin, INPUT_PULLUP);
  pinMode(Linetracker_Right_Pin, INPUT_PULLUP);
//  pinMode(Linetracker_Right_Wing_Pin, INPUT_PULLUP);

  myservo_circle.attach(Servo_Plate_Pin , 500, 2500);
  myservo_arm.attach(Servo_Arm_Pin, 500, 2500); //舵机设置
  myservo_left.attach(Servo_Left_Claw_Pin, 500, 2500);
  myservo_right.attach(Servo_Right_Claw_Pin, 500, 2500);

  myservo_right.write(90);
  myservo_left.write(30);
  myservo_arm.write(0);
  myservo_circle.write(0);

  Serial.begin(9600);

  if (tcs.begin()) {
Serial.println("Success! RGB sensor is connected and ready to use!");
} else {
Serial.println("Error: Could not connect to the RGB sensor!");
}

tcs.setInterrupt(false); // Disable the color sensor interruption

  delay(2000);
}

void loop(){
Linetracker();
// Serial.print(sen1);
// Serial.print(" ");
// Serial.print(sen2);
// Serial.print(" ");
// Serial.print(sen3);
// Serial.print(" ");
// Serial.print(sen4);
// Serial.print(" ");
// Serial.print(sen5);
// Serial.print(" ");
// Serial.println(sen6);
// Serial.print(" count ");
// Serial.print(Line_Count);
// Serial.print(" step");
// Serial.print(step);
// Serial.print(" busy ");
// Serial.println(Line_Busy);

//myservo_circle.write(0);  //白色
//myservo_circle.write(44); //绿色
//myservo_circle.write(85); //黑色
//myservo_circle.write(130);//红色
//myservo_circle.write(170);//蓝色

  if(step==1)//小车第一次过米字路口
{
  Line_Count_start =1;
  xunxian();
  if(Line_Count==3)
  {
  advance(180);
  while (turn_adjust_ok==0)
{
turn_adjust();
delay(200);
}
turn_adjust_ok=0;
  Line_Count_start =0;
  Line_Count=0;
  step=2;
  }
}
  if(step==2){//抓取第一个，转弯
      Line_Count_start =1;
      xunxian();
    if(Line_Count==1){
      grab();
      turn(210);
      descend(165);
      delay(200);
      stretch(30,90);
//         while (turn_adjust_ok==0)
// {
// turn_adjust();
// }
// turn_adjust_ok=0;
      Line_Count_start =0;
      Line_Count=0;
      step=3;       
    }
    }    
  if(step==3)//回原点转向
{
  Line_Count_start =1;
  xunxian();
  if(Line_Count==1)
  {
    advance(105);
    turn(110);//90°
turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0;
    step=4;
    }
}
  if(step==4)//第一次回原点，向前调整
{
    advance(40);
//     while (turn_adjust_ok==0)
// {
// turn_adjust();
// delay(200);
// }
// turn_adjust_ok=0;
delay(200);
    step=5;
    }

  if(step==5)//抓取第二个物块，转向
{
  Line_Count_start =1;
  xunxian();
  if(Line_Count==1)
  {
   grab();  
   turn(210);
   descend(165);
   delay(200);
   stretch(30,90);
   delay(500);
  while (turn_adjust_ok==0)
{
turn_adjust();
delay(200);
}
turn_adjust_ok=0;
   Line_Count_start =0;
   Line_Count=0;
   step=6;
  }
}

  if(step==6)//装载第二个物块过米字路口
{
  xunxian();
Line_Count_start =1;
  if(Line_Count==2){
    advance(120);
      while (turn_adjust_ok==0)
{
turn_adjust();
delay(200);
}
turn_adjust_ok=0;
    Line_Count=0;
    Line_Count_start =0;
    step=7;
  }
}

  if(step==7){//抓取第三个物块，转向
  Line_Count_start =1;
  xunxian();
  if(Line_Count==1){ 
    grab();  
    turn(210);//180°
    descend(165);
    stretch(30,90);
      while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
    Line_Count=0; 
    Line_Count_start =0; 
   step=8;
  }
  }
  if(step==8)//回到原点，转正
{Line_Count_start =1;
  xunxian();
  if(Line_Count==2){
    advance(90);//微调
    turn(110);//90°
    Line_Count_start =0;
    Line_Count=0; 
    step=9;
  }
}
if (step==9){//判断
  if (green==1){
    color=1;
    turn(110);
    advance(115);
  while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
step=9.1;
  }
  if (green==0){
    step=10;
  }
}
if(step==9.1){//放绿色物块
  {Line_Count_start =1;
  xunxian();
  if(Line_Count==1){
    advance(goal1);//由第一条线到目标区
      while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
    lay();
    back(30);//往后微调
   new_turn();
    descend(165);
      while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step=9.2;
  }
}
}
  if(step==9.2)//回原地
{Line_Count_start =1;
  xunxian();
  if(Line_Count==2){
    advance(100);//微调
    turn(110);//90
    Line_Count_start =0;
    Line_Count=0; 
    step=10;
  }
}
if (step==10){//判断
  if (white==1){
    color=2;
    turn(45);
    advance(90);
    while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
step=10.1;
  }
  if (white==0){
    step=11;
  }
}
if(step==10.1){//放白色物块
  {Line_Count_start =1;
  xunxian();
  if(Line_Count==1){
    advance(goal1);//由第一条线到目标区
      while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
    lay();
    back(30);//往后微调
   new_turn();
    descend(165);
      while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step=10.2;
  }
}
}
  if(step==10.2)//回到原点
{Line_Count_start =1;
  xunxian();
  if(Line_Count==2){
    advance(100);//微调
    turn(165);//90
    Line_Count_start =0;
    Line_Count=0; 
    step=11;
  }
}
if (step==11){//判断
  if (red==1){
    color=3;
    turn(0);
    advance(25);
    while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
step=11.1;
  }
  if (red==0){
    step=12;
  }
}
if(step==11.1){//放红色物块
  {Line_Count_start =1;
  xunxian();
  if(Line_Count==1){
    advance(goal1);//由第一条线到目标区
      while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
    lay();
    back(30);//往后微调
    new_turn();
    descend(165);
      while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step=11.2;
  }
}
}
  if(step==11.2)//回到原点
{Line_Count_start =1;
  xunxian();
  if(Line_Count==2){
    advance(100);//微调
    turn(210);//90
    Line_Count_start =0;
    Line_Count=0; 
    step=12;
  }
}
if (step==12){//判断
  if (black==1){
    color=4;
    turn(-45);
    advance(90);
    while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
step=12.1;
  }
  if (black==0){
    step=13;
  }
}
if(step==12.1){//放黑色物块
  {Line_Count_start =1;
  xunxian();
  if(Line_Count==1){
    advance(goal1);//由第一条线到目标区
      while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
    lay();
    back(30);//往后微调
    new_turn();
    descend(165);
      while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step=12.2;
  }
}
}
  if(step==12.2)//回到原点
{Line_Count_start =1;
  xunxian();
  if(Line_Count==2){
    advance(100);//微调
    turn(-165);//90
    Line_Count_start =0;
    Line_Count=0; 
    step=13;
  }
}
if (step==13){//判断颜色
  if (blue==1){
    color=5;
    turn(-110);
    advance(90);
    while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
step=13.1;
  }
  if (blue==0){
    step=14;
  }
}
if(step==13.1){//放蓝色物块
  {Line_Count_start =1;
  xunxian();
  if(Line_Count==1){
    advance(goal1);//由第一条线到目标区
      while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
    lay();
    back(30);//往后微调
    new_turn();
    descend(165);
      while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step=13.2;
  }
}
}
  if(step==13.2)//回到原点
{Line_Count_start =1;
  xunxian();
  if(Line_Count==2){
    advance(100);//微调
    turn(160);//90
    advance(40);
   while (turn_adjust_ok==0)
{
turn_adjust();
}
turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step=14;
  }
}
//   if(step==10)//回到原点过米字路口，准备放第二个物块
// {Line_Count_start =1;
//   xunxian();
//   if(Line_Count==3){
//     advance(90);//微调
//       while (turn_adjust_ok==0)
// {
// turn_adjust();
// }
// turn_adjust_ok=0;
//     Line_Count_start =0;
//     Line_Count=0; 
//     step=11;
//   }
// }
//   if(step==11)//放置第二个物块，转向
// {Line_Count_start =1;
//   xunxian();
//   if(Line_Count==1){
//     advance(goal1);//由第一条线到目标区
//       while (turn_adjust_ok==0)
// {
// turn_adjust();
// }
// turn_adjust_ok=0;
//     lay();
//     back(40);//往后微调
//     turn(210);
//     while (turn_adjust_ok==0)
// {
// turn_adjust();
// }
//     Line_Count_start =0;
//     Line_Count=0; 
//     step=12;
//   }
// }
//   if(step==12)//回原点转向，准备放置第三个物块
// {Line_Count_start =1;
//   xunxian();
//   if(Line_Count==3){
//     advance(90);//微调
//     turn(110);
//     advance(30);
//       while (turn_adjust_ok==0)
// {
// turn_adjust();
// }
// turn_adjust_ok=0;
//     Line_Count_start =0;
//     Line_Count=0; 
//     step=13;
//   }
// }
//   if(step==13)//放置第三个物块，转向
// {Line_Count_start =1;
//   xunxian();
//   if(Line_Count==1){
//     advance(goal1);//由第一条线到目标区
//       while (turn_adjust_ok==0)
// {
// turn_adjust();
// }
// turn_adjust_ok=0;
//     lay();
//     back(40);//往后微调
//     turn(210);
//       while (turn_adjust_ok==0)
// {
// turn_adjust();
// }
// turn_adjust_ok=0;
//     Line_Count_start =0;
//     Line_Count=0; 
//     step=14;
//   }
// }
//   if(step==14)//回原点，准备任务二
// {Line_Count_start =1;
//   xunxian();
//   if(Line_Count==3){
//     advance(60);
//     turn(50);
//     advance(30);
//       while (turn_adjust_ok==0)
// {
// turn_adjust();
// }
// turn_adjust_ok=0;
//     Line_Count_start =0;
//     Line_Count=0; 
//     step=0;
//     step2=1;
//   }
// }

  if(step2==1){//将五个物块都放入盘中
  Line_Count_start =1;
  xunxian();
    if(Line_Count==2){
     back(20);
     grab();
     turn(210);
     descend(165);
     stretch(30,90);
       while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step2=2;
    }
  }
  if(step2==2)//回到原点，准备到绿色区域
{Line_Count_start =1;
  xunxian();
  if(Line_Count==3){
    advance(50);
    turn(55);
    advance(40);
      while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step2=3;
  }
}
 if(step2==3)//到绿色区域放置
{Line_Count_start =1;
  xunxian();
  if(Line_Count==1){
    advance(goal2);//由第一条线到目标区
  while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    lay();
    back(20);//往后微调
    turn(210);
    descend(165);
          while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step2=4;
  }
}
   if(step2==4)//回原点转向，准备放置第2个物块
{Line_Count_start =1;
  xunxian();
  if(Line_Count==3){
    advance(90);//微调
    turn(165);
    advance(40);
    while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step2=5;
  }
}

 if(step2==5)//到白色区域放置
{Line_Count_start =1;
  xunxian();
  if(Line_Count==1){
    advance(goal1);//由第一条线到目标区
    while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    lay();
    back(20);//往后微调
    turn(210);
    descend(165);
       while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step2=6;
  }
}
   if(step2==6)//回原点转向，准备放置第3个物块
{Line_Count_start =1;
  xunxian();
  if(Line_Count==3){
    advance(90);//微调
    turn(165);
    advance(40);
    while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step2=7;
  }
}

 if(step2==7)//到红色区域放置
{Line_Count_start =1;
  xunxian();
  if(Line_Count==1){
    advance(goal1);//由第一条线到目标区
    while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    lay();
    back(20);//往后微调
    turn(210);
    descend(165);
       while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step2=8;
  }
}
   if(step2==8)//回原点转向，准备放置第4个物块
{Line_Count_start =1;
  xunxian();
  if(Line_Count==3){
    advance(90);//微调
    turn(165);
    advance(40);
    while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step2=9;
  }
}

 if(step2==9)//到黑色区域放置
{Line_Count_start =1;
  xunxian();
  if(Line_Count==1){
    advance(goal1);//由第一条线到目标区
    while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    lay();
    back(20);//往后微调
    turn(210);
    descend(165);
       while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step2=10;
  }
}
   if(step2==10)//回原点转向，准备放置第5个物块
{Line_Count_start =1;
  xunxian();
  if(Line_Count==3){
    advance(90);//微调
    turn(165);
    advance(40);
    while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step2=11;
  }
}

 if(step2==11)//到白色区域放置
{Line_Count_start =1;
  xunxian();
  if(Line_Count==1){
    advance(goal1);//由第一条线到目标区
    while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    lay();
    back(20);//往后微调
    turn(210);
    descend(165);
       while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step2=12;
  }
}
   if(step2==12)//OVER
{Line_Count_start =1;
  xunxian();
  if(Line_Count==3){
    advance(90);//微调
    turn(165);
    advance(40);
    while (turn_adjust_ok==0)
{
 turn_adjust();
 }
 turn_adjust_ok=0;
    Line_Count_start =0;
    Line_Count=0; 
    step2=13;
  }
}

}


// void LineFollow_setup()
// {
//   int Set_Line_Interrupt_ms = 10 ;                //每0.01秒进入一次中断
//   MsTimer2::set(Set_Line_Interrupt_ms,Linetracker);   //中断进入函数（）
//   MsTimer2::start();                              //开启定时中断函数
// }


void Linetracker(){
  sen1=digitalRead(Linetracker_Left_Pin);
  sen2=digitalRead(Linetracker_Left_Middle_Pin);
  sen3=digitalRead(Linetracker_Center_Left_Pin);
  sen4=digitalRead(Linetracker_Center_Right_Pin);
  sen5=digitalRead(Linetracker_Right_Middle_Pin);
  sen6=digitalRead(Linetracker_Right_Pin);
  //数线消抖
  if(Line_Count_start == 0)
  {   Line_Count_Busy = 0;
      Linetracker_Left_State = 0;
      Linetracker_Right_State = 0;}
  else if(Line_Count_Busy == 0 and Line_Count_start == 1)
  {
    if(digitalRead(Linetracker_Left_Pin)==1)
    {
      Linetracker_Left_State = 1;
      Line_Count_Busy = 1; 
      Line_Count_Time = millis();
    }
    if(digitalRead(Linetracker_Right_Pin)==1 and Line_Count_Busy == 1) //若同时识别到线
    {
      Linetracker_Right_State = 1;
    }
    else if(digitalRead(Linetracker_Right_Pin)==1 and Line_Count_Busy == 0) //若左侧没识别到线，右侧识别到
    {
      Linetracker_Right_State = 1;
      Line_Count_Busy = 1; 
      Line_Count_Time = millis();
    }
  }
  else if (Line_Count_Busy == 1 and Line_Count_start == 1)
  {	
    if(digitalRead(Linetracker_Left_Pin)==1) //查询光电状态
    {Linetracker_Left_State=1;}
    if(digitalRead(Linetracker_Right_Pin)==1)
    {Linetracker_Right_State=1;}

    if(Linetracker_Right_State == 1 and Linetracker_Left_State == 1) //判断
    {
      Line_Count++;
      Line_Count_Busy = 2;
      Line_Count_Time = millis();
    }
    else if (millis()-Line_Count_Time >= Count_Holdtime and (Linetracker_Right_State == 0 or Linetracker_Left_State == 0))
    {
      Line_Count_Busy = 0;
      Linetracker_Left_State = 0;
      Linetracker_Right_State = 0;
    }
  }
  if (Line_Count_Busy == 2 and millis()-Line_Count_Time >= Line_Delaytime  and (digitalRead(Linetracker_Left_Pin)==0 or digitalRead(Linetracker_Right_Pin)==0))
  {
    Line_Count_Busy = 0;
    Linetracker_Left_State = 0;
    Linetracker_Right_State = 0;
  }
}







// void Linetracker(){
//   int Hold_Time=2;  //消抖时间设置，光电保持在n*10ms内不变才改变状态

//   //unsigned char Linetracker_Left_Wing_Temporary_State = digitalRead(Linetracker_Left_Wing_Pin);   //读取传感器到临时变量
//   unsigned char Linetracker_Left_Temporary_State = digitalRead(Linetracker_Left_Pin);
//   unsigned char Linetracker_Left_Middle_Temporary_State = digitalRead(Linetracker_Left_Middle_Pin);
//   unsigned char Linetracker_Center_Left_Temporary_State = digitalRead(Linetracker_Center_Left_Pin);
//   unsigned char Linetracker_Center_Right_Temporary_State = digitalRead(Linetracker_Center_Right_Pin);
//   unsigned char Linetracker_Right_Middle_Temporary_State = digitalRead(Linetracker_Right_Middle_Pin);
//   unsigned char Linetracker_Right_Temporary_State = digitalRead(Linetracker_Right_Pin);
//   //unsigned char Linetracker_Right_Wing_Temporary_State = digitalRead(Linetracker_Right_Wing_Pin);

//   //static int Linetracker_Left_Wing_Trigger_Time  = 0;   //记录触发和未触发的时间
//   //static int Linetracker_Left_Wing_Untrigger_Time  = 0; 


//   if(Linetracker_Left_Temporary_State==1 ){
//     Linetracker1_Pin_Trigger_Time++; 
//   }
//   else if(Linetracker_Left_Temporary_State==0 ){
//     Linetracker1_Pin_Untrigger_Time++; 
//   }
//   if(Linetracker1_Pin_Trigger_Time>=Hold_Time && Linetracker1_Pin_Untrigger_Time<=Hold_Time){    //通过i来改变a的值，用a来改变pin 3 的电平
//   sen1=1;
//   Linetracker1_Pin_Trigger_Time=0;
//   Linetracker1_Pin_Untrigger_Time=0;
//   }
//   else if(Linetracker1_Pin_Untrigger_Time>=Hold_Time && Linetracker1_Pin_Trigger_Time<=Hold_Time) ///
//   {
//     sen1=0;
//     Linetracker1_Pin_Untrigger_Time=0;
//     Linetracker1_Pin_Trigger_Time=0;
//     }
//   if(Linetracker_Left_Middle_Temporary_State==1 ){
//     Linetracker_Left_Middle_Trigger_Time ++; 
//   }
//   if(Linetracker_Left_Middle_Temporary_State==0 ){
//     Linetracker_Left_Middle_Untrigger_Time++; 
//   }
//   if(Linetracker_Left_Middle_Trigger_Time >=Hold_Time && Linetracker_Left_Middle_Untrigger_Time<=Hold_Time){    //通过i来改变a的值，用a来改变pin 3 的电平
//   sen2=1;
//   Linetracker_Left_Middle_Trigger_Time =0;
//   Linetracker_Left_Middle_Untrigger_Time=0;
//   }
//   else if(Linetracker_Left_Middle_Untrigger_Time>=Hold_Time && Linetracker_Left_Middle_Untrigger_Time<=Hold_Time){
//     sen2=0;
//     Linetracker_Left_Middle_Untrigger_Time=0;
//     Linetracker_Left_Middle_Trigger_Time =0;    
//     }
//   if(Linetracker_Center_Left_Temporary_State==1 ){
//     Linetracker_Center_Left_Trigger_Time ++; 
//   }
//   if(Linetracker_Center_Left_Temporary_State==0 ){
//     Linetracker_Center_Left_Untrigger_Time++; 
//   }
//   if(Linetracker_Center_Left_Trigger_Time>=Hold_Time && Linetracker_Center_Left_Untrigger_Time<=Hold_Time){    //通过i来改变a的值，用a来改变pin 3 的电平
//   sen3=1;
//   Linetracker_Center_Left_Trigger_Time =0;
//   Linetracker_Center_Left_Untrigger_Time=0;  
//   }
//   else if(Linetracker_Center_Left_Untrigger_Time>=Hold_Time && Linetracker_Center_Left_Trigger_Time<=Hold_Time){
//     sen3=0;
//     Linetracker_Center_Left_Untrigger_Time=0;
//     Linetracker_Center_Left_Trigger_Time =0;
//     }
//   if(Linetracker_Center_Right_Temporary_State==1 ){
//      Linetracker_Center_Right_Trigger_Time++; 
//   }
//   if(Linetracker_Center_Right_Temporary_State==0 ){
//     Linetracker_Center_Right_Untrigger_Time++; 
//   }
//   if( Linetracker_Center_Right_Trigger_Time>=Hold_Time && Linetracker_Center_Right_Untrigger_Time<=Hold_Time){    //通过i来改变a的值，用a来改变pin 3 的电平
//   sen4=1;
//   Linetracker_Center_Right_Trigger_Time=0;
//   Linetracker_Center_Right_Untrigger_Time=0;
//   }
//   else if(Linetracker_Center_Right_Untrigger_Time>=Hold_Time && Linetracker_Center_Right_Trigger_Time<=Hold_Time){
//     sen4=0;
//     Linetracker_Center_Right_Untrigger_Time=0;
//     Linetracker_Center_Right_Trigger_Time=0;
//     }
//   if(Linetracker_Right_Middle_Temporary_State==1 ){
//      Linetracker_Right_Middle_Trigger_Time ++;
//   }
//   if(Linetracker_Right_Middle_Temporary_State==0 ){
//     Linetracker_Right_Middle_Untrigger_Time ++; 
//   }
//   if(Linetracker_Right_Middle_Trigger_Time>=Hold_Time && Linetracker_Right_Middle_Untrigger_Time<=Hold_Time){    //通过i来改变a的值，用a来改变pin 3 的电平
//     sen5=1;
//   Linetracker_Right_Middle_Trigger_Time =0;
//   Linetracker_Right_Middle_Untrigger_Time=0;
//   }
//   else if(Linetracker_Right_Middle_Untrigger_Time>=Hold_Time && Linetracker_Right_Middle_Trigger_Time<=Hold_Time){
//     sen5=0;
//     Linetracker_Right_Middle_Untrigger_Time=0;
//     Linetracker_Right_Middle_Trigger_Time =0;
//     }
//   if(Linetracker_Right_Temporary_State==1 ){
//      Linetracker_Right_Trigger_Time ++;
//   }
//   if(Linetracker_Right_Temporary_State==0 ){
//     Linetracker_Right_Untrigger_Time ++; 
//   }
//   if(Linetracker_Right_Trigger_Time>=Hold_Time && Linetracker_Right_Untrigger_Time<=Hold_Time){    //通过i来改变a的值，用a来改变pin 3 的电平
//   sen6=1;
//   Linetracker_Right_Trigger_Time =0;
//   Linetracker_Right_Untrigger_Time=0;
//   }
//   else if(Linetracker_Right_Untrigger_Time>=Hold_Time && Linetracker_Right_Trigger_Time<=Hold_Time){
//     sen6=0;
//     Linetracker_Right_Untrigger_Time=0;
//     Linetracker_Right_Trigger_Time =0;
//     }

//   if(sen1 == 1 and sen6 == 1  and Line_Count_start==1 and Line_Busy==0) //数线程序，判断最左侧和最右侧的光电
//   {
//     Line_Count+=1;
//     Line_Busy=1;
//   }
//   else if(Line_Count_start==1 and Line_Busy==1 and (sen1==0 or sen6 == 0) )
//   {
//     Line_Busy =0;
//   }
//   else if(Line_Count_start==0)  //关闭数线自动清零
//   {
//     Line_Count=0;
//   }
// }


void rise(int rise_number)//下降
{
  for (pos0 = 0; pos0 <= rise_number; pos0 += 1){
    myservo_arm.write(pos0);
    delay(10);    
  }
}
void descend(int descend_number)//上升
 {
  for (pos1 = descend_number; pos1 >= 0; pos1 -= 1){
    myservo_arm.write(pos1);
    delay(10);    
  }
}

void grab()//抓取放置的行为
  {
 tight();
 delay(10); 
 GetColor();
 delay(10); 
 circle();
delay(10); 
 rise(168);
 stretch(25,95);
  }

void lay()//抓取放置的行为
  {
circle();
delay(10);
stretch(25,95);
delay(10);
rise(180);
tight();
delay(10);
descend(180);
stretch(30,90);
delay(10);
rise(165);
 }

void xunxian() {
 if (sen2 ==0 && sen3 == 1 && sen4 ==1 && sen5 == 0) //直走
  {
   stepper1.move(-speed*1*Motor_Resolution);
   stepper2.move(speed*1*Motor_Resolution);
   stepper2.run();
   stepper1.run();
  }
  else if (sen2 ==0 && sen3 == 1 && sen4 ==0 && sen5 == 0) //小左
  {
   stepper1.move(-speed*0.6*Motor_Resolution);
   stepper2.move(speed*1*Motor_Resolution);
   stepper2.run();
   stepper1.run();
  }
  else if (sen2 ==1 && sen3 == 1 && sen4 ==0 && sen5 == 0) //中左
  {
   stepper1.move(-speed*0.5*Motor_Resolution);
   stepper2.move(speed*1*Motor_Resolution);
   stepper2.run();
   stepper1.run();
  }
  else if (sen2 ==1 && sen3 == 0 && sen4 ==0 && sen5 == 0) //大左
  {
   stepper1.move(-speed*0.3*Motor_Resolution);
   stepper2.move(speed*1*Motor_Resolution);
   stepper2.run();
   stepper1.run();
  }
  else if (sen2 ==0 && sen3 == 0 && sen4 ==1 && sen5 == 0) //小右
  {
   stepper1.move(-speed*1*Motor_Resolution);
   stepper2.move(speed*0.6*Motor_Resolution);
   stepper2.run();
   stepper1.run();
  }
  else if (sen2 ==0 && sen3 == 0 && sen4 ==1 && sen5 == 1 )//中右
 {
   stepper1.move(-speed*1*Motor_Resolution);
   stepper2.move(speed*0.5*Motor_Resolution);
   stepper2.run();
   stepper1.run();
  }
  else if (sen2 ==0 && sen3 == 0 && sen4 ==0 && sen5 == 1 ) //大右
  {
  stepper1.move(-speed*1*Motor_Resolution);
  stepper1.move(speed*0.3*Motor_Resolution);
  stepper2.run();
  stepper1.run();
  }
  else { //没有检测到黑线
   stepper1.move(-speed*Motor_Resolution);
   stepper2.move(speed*Motor_Resolution);
   stepper2.run();
   stepper1.run();
  }
}



  void advance(int advancestep )
{
  ClearDistance();
  stepper1.move(-advancestep*Motor_Resolution);
  stepper2.move(advancestep*Motor_Resolution);

  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0)
  {
    stepper1.run();
    stepper2.run();
  }
}

  void turn(int angle)
{
  ClearDistance();
  stepper1.move(angle*Motor_Resolution);
  stepper2.move(angle*Motor_Resolution);

  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0)
  {
    stepper1.run();
    stepper2.run();
  }
}

  void back(int back)
{
  ClearDistance();
  stepper1.move(back*Motor_Resolution);
  stepper2.move(-back*Motor_Resolution);

  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0)
  {
    stepper1.run();
    stepper2.run();
  }
}

void ClearDistance()
{
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper1.moveTo(0);
  stepper2.moveTo(0);
}

void tight(){

  for (pos_right = 90; pos_right <= 125; pos_right += 1){
    myservo_right.write(pos_right);
    delay(10);  }
  for (pos_left = 30; pos_left >= 5; pos_left -= 1){
    myservo_left.write(pos_left);
    delay(10); }
}

void stretch(int left_stretch_number,int right_stretch_number){
  for (pos_right = 125; pos_right >= right_stretch_number; pos_right -= 1){
    myservo_right.write(pos_right);
    delay(10);  }
  for (pos_left = 5; pos_left <= left_stretch_number; pos_left += 1){
    myservo_left.write(pos_left);
    delay(10); }
}

void turn_adjust() //旋转调整函数
{
  if ((sen2 ==0 && sen3 == 1 && sen4 ==1 && sen5 == 0) or (sen2 ==0 && sen3 == 1 && sen4 ==1 && sen5 == 1))
  //or (sen2 ==1 && sen3 == 1 && sen4 ==1 && sen5 == 0) )//判断可用
  
  {
  turn_adjust_ok=1; 
  stepper1.stop();
  stepper2.stop();
  }
  else if (sen2 ==0 && sen3 == 1 && sen4 ==0 && sen5 == 0) //判断可用X 向左转
  {
  turn_adjust_ok=1;    
  stepper1.stop();
  stepper2.stop();
  // stepper1.setSpeed(25*Motor_Resolution);    
  // stepper2.setSpeed(25*Motor_Resolution);    
  // stepper1.runSpeed();
  // stepper2.runSpeed();
  }
  else if (sen2 ==1 && sen3 == 1 && sen4 ==0 && sen5 == 0) //向左转
  {
  stepper1.setSpeed(50*Motor_Resolution);    
  stepper2.setSpeed(50*Motor_Resolution);    
  stepper1.runSpeed();
  stepper2.runSpeed();
  }
  else if (sen2 ==1 && sen3 == 0 && sen4 ==0 && sen5 == 0) //向左转
  {
  stepper1.setSpeed(80*Motor_Resolution);    
  stepper2.setSpeed(80*Motor_Resolution);    
  stepper1.runSpeed();
  stepper2.runSpeed();
  }
  // else if (sen2 ==0 && sen3 == 0 && sen4 ==1 && sen5 == 0) //判断可用X 向右转
  // {
  // turn_adjust_ok=1;    
  // stepper1.stop();
  // stepper2.stop();
  // // stepper1.setSpeed(-25*Motor_Resolution);    
  // // stepper2.setSpeed(-25*Motor_Resolution);    
  // // stepper1.runSpeed();
  // // stepper2.runSpeed();
  // }
  else if (sen2 ==0 && sen3 == 0 && sen4 ==1 && sen5 == 1 )//向右转
 {
  stepper1.setSpeed(-50*Motor_Resolution);    
  stepper2.setSpeed(-50*Motor_Resolution);    
  stepper1.runSpeed();
  stepper2.runSpeed();
  }
  else if (sen2 ==0 && sen3 == 0 && sen4 ==0 && sen5 == 1 ) //向右转
  {
  stepper1.setSpeed(-80*Motor_Resolution);    
  stepper2.setSpeed(-80*Motor_Resolution);    
  stepper1.runSpeed();
  stepper2.runSpeed();
  }
}
//myservo_circle.write(0);  //白色
//myservo_circle.write(44); //绿色
//myservo_circle.write(85); //黑色
//myservo_circle.write(130);//红色
//myservo_circle.write(170);//蓝色
void circle(){
if(color==5){
  myservo_circle.write(170);//蓝色
}
if(color==3){
  myservo_circle.write(130);//红色
}
if(color==4){
  myservo_circle.write(85); //黑色
}
if(color==1){
  myservo_circle.write(44); //绿色
}
if(color==2){
  myservo_circle.write(0);//白色
}
}

void GetColor()
{
  int R = 0;
  int G = 0;
  int B = 0;
  int C = 0;
  delay(600);
  tcs.getRawData(&R, &G, &B, &C);

  // Serial.print("R ");
  // Serial.print(R);
  // Serial.print(" G ");
  // Serial.print(G);
  // Serial.print(" B ");
  // Serial.print(B);
  // Serial.print(" C ");
  // Serial.print(C);

  if (C > 1000)
  {
    color= 2;
    white++;
    // Serial.println("The color detected is WHITE.");
  }
  else if (R < 50 && G < 50 && B < 50 && C < 100)
  {
    color= 4;
    black++;
    // Serial.println("The color detected is BLACK");
  }
  else if (R > G && R > B)
  {
    color= 3;
    red++;
    // Serial.println("The color detected is RED.");
  }
  else if (G > R && G > B)
  {
    color= 1;
    green++;  
    // Serial.println("The color detected is Green.");
  }
  else if (B > R && B > G)
  {
    color= 5;
    blue++;
    // Serial.println("The color detected is BLUE.");
  }
  else
  {
    color= 0;
    // Serial.println("The color detected is NO.");
  }
}

void new_turn(){
  turn(30);
  while (sen3 != 1 || sen4 !=1 )
 {
  stepper1.setSpeed(50*Motor_Resolution);    
  stepper2.setSpeed(50*Motor_Resolution);    
  stepper1.runSpeed();
  stepper2.runSpeed();
  }

}