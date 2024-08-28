////////OLED显示屏引脚///////////
#define OLED_DC 5  
//#define OLED_CS 5//CS直接接地
#define OLED_CLK 8
#define OLED_MOSI 7
#define OLED_RESET 6

/////////TB6612驱动引脚////
#define PWM 9
#define IN1 10
#define IN2 11
/////////编码器引脚////////
#define ENCODER_A 4//电机编码器引脚
#define ENCODER_B 2//电机编码器引脚 
#define ZHONGZHI 803//摆杆直立取中值,自然下垂时为0
#define POSITION 10000//位置环取值
/////////按键引脚////////
#define KEY_Memu 3
#define KEY_S 15
#define KEY_Minus 16
#define KEY_Plus 17
#define KEY_X 18

#include <SSD1306.h>
#include <PinChangeInt.h>//外部中断
#include <MsTimer2.h>//定时中断
#include <DATASCOPE.h>      //这是PC端上位机的库文件
DATASCOPE data;//实例化一个 上位机 对象，对象名称为 data
SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0);
float Position=10000,Sensor,Motor;//位置环取值参数，角位移传感器参数，电机PWM赋值参数
float Balance_Pwm,Position_Pwm;//直立环PWM参数，位置环PWM参数
int Target=10000;   //目标值
unsigned char Send_Count,Flash_Send,Flag_Stop=1;  //上位机相关变量和停止标志位
float Balance_KP=172,Balance_KD=172,Position_KP=66,Position_KD=66;  //PID系数
float Menu=1,Amplitude1=1,Amplitude2=1,Amplitude3=1,Amplitude4=1; //PID调试相关参数
int Battery_Voltage; //电池电压采样变量
/**************************************************************************
函数功能：虚拟示波器往上位机发送数据 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void DataScope(void)
{
  int i;
  data.DataScope_Get_Channel_Data(Sensor, 1);  //显示第一个数据
  data.DataScope_Get_Channel_Data(Position, 2);//显示第二个数据
//  data.DataScope_Get_Channel_Data(0, 3);//显示第三个数据
//  data.DataScope_Get_Channel_Data(0, 4);//显示第四个数据
  Send_Count = data.DataScope_Data_Generate(2); 
  for ( i = 0 ; i < Send_Count; i++)
  {
    Serial.write(DataScope_OutPut_Buffer[i]);  
  }
  delay(50);  //上位机必须严格控制发送时序
}
/**************************************************************************
函数功能：参数调节 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void Adjust(void)
{   
  int Position=200; //单击一次用户按键移动的距离     
  int  temp,temp2;   //临时变量
  static int flag,count;
  temp=My_click();   //采集单击按键
   if(temp==1)       //Menu 菜单滚动去选择需要调节的参数
  {
    if(Menu++==4) Menu=1;
  }   
     if(temp==2)     //控制电机的起停
  {
    Flag_Stop=!Flag_Stop;  //电机控制标志位
  }   
  if(temp==3)  //PID-  PID参数-
  {
          if(Menu==1)  Balance_KP-=Amplitude1;
    else  if(Menu==2)  Balance_KD-=Amplitude2;
    else  if(Menu==3)  Position_KP-=Amplitude3;
    else  if(Menu==4)  Position_KD-=Amplitude4;
  }   
   if(temp==4) //PID+  PID参数+
  {
          if(Menu==1)  Balance_KP+=Amplitude1;
    else  if(Menu==2)  Balance_KD+=Amplitude2;
    else  if(Menu==3)  Position_KP+=Amplitude3;
    else  if(Menu==4)  Position_KD+=Amplitude4;
  }   
  if(Balance_KP<=0) Balance_KP=0;  //避免参数出现负数
  if(Balance_KD<=0) Balance_KD=0;
  if(Position_KP<=0) Position_KP=0;
  if(Position_KD<=0) Position_KD=0;

  temp2=click_N_Double(100); //采集双击事件
  if(temp2==1)flag=2;//++    //单击 顺时针转动
  else  if(temp2==2)flag=1;// 双击 逆时针转动
  
  if(flag==1) //摆杆顺时针运动
  {
    Target++;
    count++;  
    if(count==Position)   flag=0,count=0;
  } 
  else  if(flag==2) //摆杆逆时针运动
  {
    Target--;
    count++;  
    if(count==Position)   flag=0,count=0;
  }
}
/**************************************************************************
函数功能：按键扫描  作者：平衡小车之家
入口参数：无
返回  值：按键状态，1：单击事件，0：无事件。
**************************************************************************/
unsigned char My_click(void){
  static unsigned char flag_key = 1; //按键按松开标志
  if (flag_key && (digitalRead(KEY_Memu) == 0 || digitalRead(KEY_S) == 0 || digitalRead(KEY_Minus) == 0 || digitalRead(KEY_Plus) == 0)) //如果发生单击事件
  {
     flag_key = 0;
    if(digitalRead(KEY_Memu)==0)  return 1;       //M键      
    if(digitalRead(KEY_S)==0)  return 2;          //S键
    if(digitalRead(KEY_Minus)==0)  return 3;      //-键 
    if(digitalRead(KEY_Plus)==0)  return 4;       //+键
  }
  else if (digitalRead(KEY_Memu) == 1 && digitalRead(KEY_S) == 1 && digitalRead(KEY_Minus) == 1 && digitalRead(KEY_Plus) == 1)     flag_key = 1;
  return 0;//无按键按下
}
/**************************************************************************
函数功能：按键扫描
入口参数：双击等待时间
返回  值：按键状态 0：无动作 1：单击 2：双击 
**************************************************************************/
u8 click_N_Double (u8 time)
{
    static  unsigned char  flag_key,count_key,double_key; 
    static  unsigned int count_single,Forever_count;
    char temp;
    temp=digitalRead(KEY_X);  //读取X按键
    if(temp==0)  Forever_count++;   //长按 标志位置1
     else        Forever_count=0;
    if(0==temp&&0==flag_key)    flag_key=1; 
    if(0==count_key)
    {
        if(flag_key==1) 
        {
          double_key++;
          count_key=1;  
        }
        if(double_key==2) 
        {
          double_key=0;
          count_single=0;
          return 2;//双击执行的指令
        }
    }
    if(1==temp)     flag_key=0,count_key=0;
    
    if(1==double_key)
    {
      count_single++;
      if(count_single>time&&Forever_count<time)
      {
      double_key=0;
      count_single=0; 
      return 1;//单击执行的指令
      }
      if(Forever_count>time)
      {
      double_key=0;
      count_single=0; 
      }
    } 
    return 0;
}
/**************************************************************************
函数功能：求次方的函数
入口参数：m,n
返回  值：m的n次幂
**************************************************************************/
uint32_t oled_pow(uint8_t m,uint8_t n)
{
  uint32_t result=1;  
  while(n--)result*=m;    
  return result;
} 
/**************************************************************************
函数功能：显示变量
入口参数：x:x坐标   y:行     num：显示的变量   len ：变量的长度
返回  值：无
**************************************************************************/
void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len)
{           
  u8 t,temp;
  u8 enshow=0;               
  for(t=0;t<len;t++)
  {
    temp=(num/oled_pow(10,len-t-1))%10;
    oled.drawchar(x+6*t,y,temp+'0');
  }  
} 
/**************************************************************************
函数功能：倾角PD控制
入口参数：角度
返回  值：倾角控制PWM
作    者：平衡小车之家
**************************************************************************/
float Balance_Control(float sensor)
{  
   float Bias;                           //倾角偏差
   static float Last_Bias,D_Bias;       //PID相关变量
   int balance;                        //PWM返回值 
   Bias=sensor-ZHONGZHI;              //求出平衡的角度中值 和机械相关
   D_Bias=Bias-Last_Bias;            //求出偏差的微分 进行微分控制
   balance=Balance_KP/10*Bias+D_Bias*Balance_KD/1000;   //===计算倾角控制的电机PWM  PD控制
   Last_Bias=Bias;                   //保持上一次的偏差
   return balance;                  //返回控制量
}
/**************************************************************************
函数功能：位置PD控制 
入口参数：编码器
返回  值：位置控制PWM
作    者：平衡小车之家
**************************************************************************/
float Position_Control(int Encoder)
{  
   static float Position_PWM,Last_Position,Position_Bias,Position_Differential; //定义相关变量
   static float Position_Least;
    Position_Least =Encoder-Target;             //===获取偏差值
    Position_Bias *=0.8;                        //===一阶低通滤波器  
    Position_Bias += Position_Least*0.2;               //===一阶低通滤波器  
    Position_Differential=Position_Bias-Last_Position; //===一阶差分
    Last_Position=Position_Bias;                       //保持上一次的偏差
    Position_PWM=Position_Bias*Position_KP/1000+Position_Differential*Position_KD/10; //===位置控制 
    return Position_PWM; //返回控制量
}
/**************************************************************************
函数功能：赋值给PWM寄存器 作者：平衡小车之家
入口参数：PWM
**************************************************************************/
void Set_PWM(int motor)
{
  if (motor< 0)     digitalWrite(IN1, HIGH),      digitalWrite(IN2, LOW);  //TB6612的电平控制
  else               digitalWrite(IN1, LOW),       digitalWrite(IN2, HIGH); //TB6612的电平控制
  analogWrite(PWM, abs(motor)+10); //赋值给PWM寄存器
}
/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
unsigned char  Turn_Off()
{
      unsigned char temp; 
      if(Sensor<(ZHONGZHI-200)||Sensor>(ZHONGZHI+200)||Flag_Stop==1) //Flag_Stop置1或者摆杆大幅度偏离平衡位置 关闭电机
      {              
       temp=1;   
       Flag_Stop=1;                                         
       digitalWrite(IN1, LOW);  //TB6612的电平控制                                           
       digitalWrite(IN2, LOW);  //TB6612的电平控制
      }
      else
      temp=0;
      return temp;      
}
/**************************************************************************
函数功能：获取角位移传感器平均值  作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
 u16 Get_Adc_Average(u8 ch,u8 times)
{
  unsigned int temp_val=0;
  unsigned char  t;
  for(t=0;t<times;t++)
  {
    temp_val+=analogRead(ch);//采集模拟量
  }
  return temp_val/times;//取平均值
}   
/**************************************************************************
函数功能：5ms控制函数 核心代码 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void control()
{  
      static float Voltage_All,Voltage_Count;//电压采样相关变量
      int Temp;//临时变量
      static unsigned char Position_Count;  //位置控制分频用的变量
      sei();//全局中断开启
      Sensor= Get_Adc_Average(5,5);     //均值滤波
      Balance_Pwm =Balance_Control(Sensor);             //===角度PD控制  
      if(++Position_Count>4) Position_Pwm=Position_Control(Position),Position_Count=0;     //===位置PD控制 
      Motor=Balance_Pwm-Position_Pwm;          //===计算电机最终PWM
      if(Motor>245)Motor=245;   //PWM参数限值，不限值超过255电机会失能
      if(Motor<-245)Motor=-245; //PWM参数限值，不限值超过255电机会失能
      if(Turn_Off()==0)
      Set_PWM(Motor);    //输出电机控制量
      Adjust(); //PID调节
      Temp = analogRead(0);  //采集一下电池电压
      Voltage_Count++;       //平均值计数器
      Voltage_All+=Temp;     //多次采样累积
      if(Voltage_Count==200) Battery_Voltage=Voltage_All*0.05371/2,Voltage_All=0,Voltage_Count=0;//求平均值
 }
 /**************************************************************************
函数功能：初始化 相当于STM32里面的Main函数 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void setup()   { 
  int fff = 1;
  TCCR1B =(TCCR1B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ
  
   oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
   oled.clear();   // clears the screen and buffer      
   pinMode(IN1, OUTPUT);          //TB6612方向控制引脚
   pinMode(IN2, OUTPUT);          //TB6612方向控制引脚，
   pinMode(PWM, OUTPUT);          //TB6612速度控制引脚
   digitalWrite(IN1, 0);          //TB6612控制引脚拉低
   digitalWrite(IN2, 0);          //TB6612控制引脚拉低
   digitalWrite(PWM, 0);          //TB6612控制引脚拉低
   pinMode(ENCODER_A, INPUT);       //编码器引脚
   pinMode(ENCODER_B, INPUT);       //编码器引脚
   Serial.begin(128000);           //开启串口
   delay(200);                      //延时等待初始化完成
   MsTimer2::set(5, control);       //使用Timer2设置5ms定时中断
   MsTimer2::start();               //中断使能
  attachInterrupt(0, READ_ENCODER_A, CHANGE);           //开启外部中断 

}
/**************************************************************************
函数功能：主循环程序体
入口参数：无
返回  值：无
**************************************************************************/
void loop()                     
{
  //=============第1行显示角度PD控制P参数=======================// 
                           oled.drawstring(0, 00, "B-KP:");
                           OLED_ShowNumber(30,00,Balance_KP,3);
                           oled.drawstring(85, 00, "A:");
                           OLED_ShowNumber(100,00,Amplitude1,3);
    //=============第2行显示角度PD控制D参数=======================//  
                           oled.drawstring(0, 01, "B-KD:");
                           OLED_ShowNumber(30,01,Balance_KD,3);
                           oled.drawstring(85, 01, "A:");
                           OLED_ShowNumber(100,01,Amplitude2,3);
    //=============第3行显示位置PD控制P参数=======================// 
                           oled.drawstring(0, 02, "P-KP:");
                           OLED_ShowNumber(30,02,Position_KP,3);
                           oled.drawstring(85, 02, "A:");
                           OLED_ShowNumber(100,02,Amplitude3,3);
    //=============第4行显示位置PD控制D参数=======================// 
                           oled.drawstring(0, 03, "P-KD:");
                           OLED_ShowNumber(30,03,Position_KD,3);
                           oled.drawstring(85, 03, "A:");
                           OLED_ShowNumber(100,03,Amplitude4,3);
    //======这是滚动菜单 选择需要调节的PD参数                      
      if(Menu==1)
      {
       oled.drawstring(65, 00, "Y");
       oled.drawstring(65, 01, "N");
       oled.drawstring(65, 02, "N");
       oled.drawstring(65, 03, "N");
      }
      else  if(Menu==2)
      {
       oled.drawstring(65, 00, "N");
       oled.drawstring(65, 01, "Y");
       oled.drawstring(65, 02, "N");
       oled.drawstring(65, 03, "N");
      }   
      else if(Menu==3)
      {     
       oled.drawstring(65, 00, "N");
       oled.drawstring(65, 01, "N");
       oled.drawstring(65, 02, "Y");
       oled.drawstring(65, 03, "N");
      }   
      else if(Menu==4)
      {       
       oled.drawstring(65, 00, "N");
       oled.drawstring(65, 01, "N");
       oled.drawstring(65, 02, "N");
       oled.drawstring(65, 03, "Y");
      } 
 //=============第五行显示电压=======================//      
           oled.drawstring(00,4,"VOLTAGE:");
           oled.drawstring(71,4,".");
           oled.drawstring(93,4,"V");
           OLED_ShowNumber(58,4,Battery_Voltage/100,2);
           OLED_ShowNumber(81,4,Battery_Voltage%100,2);
   //  if(Battery_Voltage%100<10)   OLED_ShowNumber(45,4,0,2);
 //=============第六行显示 编码器数据=======================//
        oled.drawstring(00,5,"POSITION:");
        OLED_ShowNumber(60,5,Position,5);
 //=============第七行显示位置目标值=======================//
  oled.drawstring(00,6,"TARGET:");
  OLED_ShowNumber(60,6,Target,5);
//=============第八行显示角位移传感器值和起始值=======================//
  oled.drawstring(00,7,"ADC:");
  OLED_ShowNumber(25,7,Sensor,4);  
    oled.drawstring(55,7,"ORIGIN:");  //这个值是倒立摆摆杆自然下垂的时候的值
  OLED_ShowNumber(100,7,240,3);
//=============刷新=======================//
  oled.display();
  DataScope();
}
/**************************************************************************
函数功能：外部中断读取编码器数据，具有4倍频功能 注意外部中断是跳变沿触发
入口参数：无
返回  值：无
**************************************************************************/
void READ_ENCODER_A() {
    if (digitalRead(ENCODER_A) == HIGH) {     
    if (digitalRead(ENCODER_B) == LOW)      Position++;  //根据另外一相电平判定方向
    else      Position--;
  }
    else {    
    if (digitalRead(ENCODER_B) == LOW)      Position--; //根据另外一相电平判定方向
    else     Position++;
  }
}
/**************************************************************************
函数功能：外部中断读取编码器数据，具有4倍频功能 注意外部中断是跳变沿触发
入口参数：无
返回  值：无
**************************************************************************/
void READ_ENCODER_B() {
    if (digitalRead(ENCODER_B) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(ENCODER_A) == LOW)      Position++;//根据另外一相电平判定方向
    else      Position--;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(ENCODER_A) == LOW)      Position--; //根据另外一相电平判定方向
    else     Position++;
  }
}
