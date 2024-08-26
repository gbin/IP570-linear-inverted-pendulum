////////OLED display pins///////////
#define OLED_DC 5  
//#define OLED_CS 5 // CS is directly connected to ground
#define OLED_CLK 8
#define OLED_MOSI 7
#define OLED_RESET 6

/////////TB6612 driver pins////
#define PWM 9
#define IN1 10
#define IN2 11
/////////Encoder pins////////
#define ENCODER_A 4 // Motor encoder pin
#define ENCODER_B 2 // Motor encoder pin 
#define ZHONGZHI 803 // Midpoint value for the pendulum in upright position, 0 when naturally hanging down
#define POSITION 10000 // Position loop value
/////////Button pins////////
#define KEY_Memu 3
#define KEY_S 15
#define KEY_Minus 16
#define KEY_Plus 17
#define KEY_X 18

#include <SSD1306.h>
#include <PinChangeInt.h> // External interrupt
#include <MsTimer2.h> // Timer interrupt
#include <DATASCOPE.h> // PC-side software library
DATASCOPE data; // Instantiate a PC-side object named 'data'
SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0);
float Position = 10000, Sensor, Motor; // Position loop value parameter, angle displacement sensor parameter, motor PWM assignment parameter
float Balance_Pwm, Position_Pwm; // Balance loop PWM parameter, position loop PWM parameter
int Target = 10000; // Target value
unsigned char Send_Count, Flash_Send, Flag_Stop = 1;  // Variables related to PC software and stop flag
float Balance_KP = 172, Balance_KD = 172, Position_KP = 66, Position_KD = 66;  // PID coefficients
float Menu = 1, Amplitude1 = 1, Amplitude2 = 1, Amplitude3 = 1, Amplitude4 = 1; // PID tuning related parameters
int Battery_Voltage; // Battery voltage sampling variable
/**************************************************************************
Function: Send data to the PC-side software using a virtual oscilloscope. Author: Balance Car Home
Parameters: None
Return Value: None
**************************************************************************/
void DataScope(void)
{
  int i;
  data.DataScope_Get_Channel_Data(Sensor, 1);  // Display the first data
  data.DataScope_Get_Channel_Data(Position, 2); // Display the second data
//  data.DataScope_Get_Channel_Data(0, 3); // Display the third data
//  data.DataScope_Get_Channel_Data(0, 4); // Display the fourth data
  Send_Count = data.DataScope_Data_Generate(2); 
  for ( i = 0 ; i < Send_Count; i++)
  {
    Serial.write(DataScope_OutPut_Buffer[i]);  
  }
  delay(50);  // The PC-side software must strictly control the send timing
}
/**************************************************************************
Function: Parameter adjustment. Author: Balance Car Home
Parameters: None
Return Value: None
**************************************************************************/
void Adjust(void)
{   
  int Position = 200; // The distance moved by the user button for a single click     
  int temp, temp2;   // Temporary variables
  static int flag, count;
  temp = My_click();   // Capture single button click
   if(temp == 1)       // Menu scrolls to select the parameter to adjust
  {
    if(Menu++ == 4) Menu = 1;
  }   
  if(temp == 2)     // Control the start and stop of the motor
  {
    Flag_Stop = !Flag_Stop;  // Motor control flag
  }   
  if(temp == 3)  // PID-  PID parameter-
  {
    if(Menu == 1)  Balance_KP -= Amplitude1;
    else  if(Menu == 2)  Balance_KD -= Amplitude2;
    else  if(Menu == 3)  Position_KP -= Amplitude3;
    else  if(Menu == 4)  Position_KD -= Amplitude4;
  }   
  if(temp == 4) // PID+  PID parameter+
  {
    if(Menu == 1)  Balance_KP += Amplitude1;
    else  if(Menu == 2)  Balance_KD += Amplitude2;
    else  if(Menu == 3)  Position_KP += Amplitude3;
    else  if(Menu == 4)  Position_KD += Amplitude4;
  }   
  if(Balance_KP <= 0) Balance_KP = 0;  // Avoid negative parameters
  if(Balance_KD <= 0) Balance_KD = 0;
  if(Position_KP <= 0) Position_KP = 0;
  if(Position_KD <= 0) Position_KD = 0;

  temp2 = click_N_Double(100); // Capture double click event
  if(temp2 == 1) flag = 2; // Single click clockwise rotation
  else if(temp2 == 2) flag = 1; // Double click counterclockwise rotation
  
  if(flag == 1) // Pendulum arm clockwise movement
  {
    Target++;
    count++;  
    if(count == Position)   flag = 0, count = 0;
  } 
  else if(flag == 2) // Pendulum arm counterclockwise movement
  {
    Target--;
    count++;  
    if(count == Position)   flag = 0, count = 0;
  }
}
/**************************************************************************
Function: Button scan. Author: Balance Car Home
Parameters: None
Return Value: Button state, 1: Single click event, 0: No event.
**************************************************************************/
unsigned char My_click(void){
  static unsigned char flag_key = 1; // Button press/release flag
  if (flag_key && (digitalRead(KEY_Memu) == 0 || digitalRead(KEY_S) == 0 || digitalRead(KEY_Minus) == 0 || digitalRead(KEY_Plus) == 0)) // If a single click event occurs
  {
     flag_key = 0;
    if(digitalRead(KEY_Memu) == 0)  return 1;       // M button      
    if(digitalRead(KEY_S) == 0)  return 2;          // S button
    if(digitalRead(KEY_Minus) == 0)  return 3;      // - button 
    if(digitalRead(KEY_Plus) == 0)  return 4;       // + button
  }
  else if (digitalRead(KEY_Memu) == 1 && digitalRead(KEY_S) == 1 && digitalRead(KEY_Minus) == 1 && digitalRead(KEY_Plus) == 1) flag_key = 1;
  return 0; // No button pressed
}
/**************************************************************************
Function: Button scan
Parameters: Double click wait time
Return Value: Button state 0: No action 1: Single click 2: Double click 
**************************************************************************/
u8 click_N_Double (u8 time)
{
    static  unsigned char  flag_key, count_key, double_key; 
    static  unsigned int count_single, Forever_count;
    char temp;
    temp = digitalRead(KEY_X);  // Read the X button
    if(temp == 0)  Forever_count++;   // Long press flag set to 1
    else        Forever_count = 0;
    if(0 == temp && 0 == flag_key)    flag_key = 1; 
    if(0 == count_key)
    {
        if(flag_key == 1) 
        {
          double_key++;
          count_key = 1;  
        }
        if(double_key == 2) 
        {
          double_key = 0;
          count_single = 0;
          return 2; // Command executed by double click
        }
    }
    if(1 == temp)     flag_key = 0, count_key = 0;
    
    if(1 == double_key)
    {
      count_single++;
      if(count_single > time && Forever_count < time)
      {
      double_key = 0;
      count_single = 0; 
      return 1; // Command executed by single click
      }
      if(Forever_count > time)
      {
      double_key = 0;
      count_single = 0; 
      }
    } 
    return 0;
}
/**************************************************************************
Function: Power function
Parameters: m, n
Return Value: m raised to the power of n
**************************************************************************/
uint32_t oled_pow(uint8_t m, uint8_t n)
{
  uint32_t result = 1;  
  while(n--) result *= m;    
  return result;
} 
/**************************************************************************
Function: Display variable
Parameters: x: x-coordinate   y: line   num: variable to display   len: length of the variable
Return Value: None
**************************************************************************/
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len)
{           
  u8 t, temp;
  u8 enshow = 0;               
  for(t = 0; t < len; t++)
  {
    temp = (num / oled_pow(10, len - t - 1)) % 10;
    oled.drawchar(x + 6 * t, y, temp + '0');
  }  
} 
/**************************************************************************
Function: Inclination PD control
Parameters: angle
Return Value: PWM for inclination control
Author: Balance Car Home
**************************************************************************/
float Balance_Control(float sensor)
{  
   float Bias;                           // Angle deviation
   static float Last_Bias, D_Bias;       // PID related variables
   int balance;                        // PWM return value 
   Bias = sensor - ZHONGZHI;              // Calculate the balance angle midpoint, related to the mechanism
   D_Bias = Bias - Last_Bias;            // Calculate the deviation differential for differential control
   balance = Balance_KP / 10 * Bias + D_Bias * Balance_KD / 1000;   // Calculate the motor PWM for inclination control, PD control
   Last_Bias = Bias;                   // Store the last deviation
   return balance;                  // Return the control value
}
/**************************************************************************
Function: Position PD control 
Parameters: encoder
Return Value: PWM for position control
Author: Balance Car Home
**************************************************************************/
float Position_Control(int Encoder)
{  
   static float Position_PWM, Last_Position, Position_Bias, Position_Differential; // Define related variables
   static float Position_Least;
    Position_Least = Encoder - Target;             // Calculate the deviation value
    Position_Bias *= 0.8;                        // First-order low-pass filter  
    Position_Bias += Position_Least * 0.2;               // First-order low-pass filter  
    Position_Differential = Position_Bias - Last_Position; // First-order difference
    Last_Position = Position_Bias;                       // Store the last deviation
    Position_PWM = Position_Bias * Position_KP / 1000 + Position_Differential * Position_KD / 10; // Position control 
    return Position_PWM; // Return the control value
}
/**************************************************************************
Function: Assign value to the PWM register. Author: Balance Car Home
Parameters: PWM
**************************************************************************/
void Set_PWM(int motor)
{
  if (motor < 0)     digitalWrite(IN1, HIGH),      digitalWrite(IN2, LOW);  // TB6612 level control
  else               digitalWrite(IN1, LOW),       digitalWrite(IN2, HIGH); // TB6612 level control
  analogWrite(PWM, abs(motor) + 10); // Assign value to the PWM register
}
/**************************************************************************
Function: Abnormal motor shutdown
Parameters: voltage
Return Value: 1: Abnormal  0: Normal
**************************************************************************/
unsigned char  Turn_Off()
{
      unsigned char temp; 
      if(Sensor < (ZHONGZHI - 200) || Sensor > (ZHONGZHI + 200) || Flag_Stop == 1) // Close the motor if Flag_Stop is set to 1 or the pendulum arm deviates significantly from the balanced position
      {              
       temp = 1;   
       Flag_Stop = 1;                                         
       digitalWrite(IN1, LOW);  // TB6612 level control                                           
       digitalWrite(IN2, LOW);  // TB6612 level control
      }
      else
      temp = 0;
      return temp;      
}
/**************************************************************************
Function: Obtain the average value of the angle displacement sensor. Author: Balance Car Home
Parameters: None
Return Value: None
**************************************************************************/
 u16 Get_Adc_Average(u8 ch, u8 times)
{
  unsigned int temp_val = 0;
  unsigned char  t;
  for(t = 0; t < times; t++)
  {
    temp_val += analogRead(ch); // Capture analog value
  }
  return temp_val / times; // Calculate the average value
}   
/**************************************************************************
Function: 5ms control function, core code. Author: Balance Car Home
Parameters: None
Return Value: None
**************************************************************************/
void control()
{  
      static float Voltage_All, Voltage_Count; // Voltage sampling related variables
      int Temp; // Temporary variable
      static unsigned char Position_Count;  // Variable used for position control frequency division
      sei(); // Enable global interrupts
      Sensor = Get_Adc_Average(5, 5);     // Mean filtering
      Balance_Pwm = Balance_Control(Sensor);             // Angle PD control  
      if(++Position_Count > 4) Position_Pwm = Position_Control(Position), Position_Count = 0;     // Position PD control 
      Motor = Balance_Pwm - Position_Pwm;          // Calculate the final motor PWM
      if(Motor > 245) Motor = 245;   // PWM parameter limit, exceeding 255 will disable the motor
      if(Motor < -245) Motor = -245; // PWM parameter limit, exceeding 255 will disable the motor
      if(Turn_Off() == 0)
      Set_PWM(Motor);    // Output motor control value
      Adjust(); // PID adjustment
      Temp = analogRead(0);  // Sample the battery voltage
      Voltage_Count++;       // Average value counter
      Voltage_All += Temp;     // Accumulate multiple samples
      if(Voltage_Count == 200) Battery_Voltage = Voltage_All * 0.05371 / 2, Voltage_All = 0, Voltage_Count = 0; // Calculate the average value
 }
 /**************************************************************************
Function: Initialization equivalent to the Main function in STM32. Author: Balance Car Home
Parameters: None
Return Value: None
**************************************************************************/
void setup()   { 
  int fff = 1;
  TCCR1B =(TCCR1B & 0xF8) | fff; // Adjust the counter frequency division, the frequency is increased to 31.374KHZ
  
   oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
   oled.clear();   // clears the screen and buffer      
   pinMode(IN1, OUTPUT);          // TB6612 direction control pin
   pinMode(IN2, OUTPUT);          // TB6612 direction control pin
   pinMode(PWM, OUTPUT);          // TB6612 speed control pin
   digitalWrite(IN1, 0);          // TB6612 control pin pulled low
   digitalWrite(IN2, 0);          // TB6612 control pin pulled low
   digitalWrite(PWM, 0);          // TB6612 control pin pulled low
   pinMode(ENCODER_A, INPUT);       // Encoder pin
   pinMode(ENCODER_B, INPUT);       // Encoder pin
   Serial.begin(128000);           // Start serial communication
   delay(200);                      // Delay to wait for initialization to complete
   MsTimer2::set(5, control);       // Use Timer2 to set 5ms timer interrupt
   MsTimer2::start();               // Enable interrupt
  attachInterrupt(0, READ_ENCODER_A, CHANGE);           // Enable external interrupt 

}
/**************************************************************************
Function: Main loop program body
Parameters: None
Return Value: None
**************************************************************************/
void loop()                     
{
  //=============Display Balance PD control P parameter on the first line=======================// 
                           oled.drawstring(0, 00, "B-KP:");
                           OLED_ShowNumber(30,00,Balance_KP,3);
                           oled.drawstring(85, 00, "A:");
                           OLED_ShowNumber(100,00,Amplitude1,3);
    //=============Display Balance PD control D parameter on the second line=======================//  
                           oled.drawstring(0, 01, "B-KD:");
                           OLED_ShowNumber(30,01,Balance_KD,3);
                           oled.drawstring(85, 01, "A:");
                           OLED_ShowNumber(100,01,Amplitude2,3);
    //=============Display Position PD control P parameter on the third line=======================// 
                           oled.drawstring(0, 02, "P-KP:");
                           OLED_ShowNumber(30,02,Position_KP,3);
                           oled.drawstring(85, 02, "A:");
                           OLED_ShowNumber(100,02,Amplitude3,3);
    //=============Display Position PD control D parameter on the fourth line=======================// 
                           oled.drawstring(0, 03, "P-KD:");
                           OLED_ShowNumber(30,03,Position_KD,3);
                           oled.drawstring(85, 03, "A:");
                           OLED_ShowNumber(100,03,Amplitude4,3);
    //======This is a scrolling menu to select the PD parameter to adjust                      
      if(Menu == 1)
      {
       oled.drawstring(65, 00, "Y");
       oled.drawstring(65, 01, "N");
       oled.drawstring(65, 02, "N");
       oled.drawstring(65, 03, "N");
      }
      else  if(Menu == 2)
      {
       oled.drawstring(65, 00, "N");
       oled.drawstring(65, 01, "Y");
       oled.drawstring(65, 02, "N");
       oled.drawstring(65, 03, "N");
      }   
      else if(Menu == 3)
      {     
       oled.drawstring(65, 00, "N");
       oled.drawstring(65, 01, "N");
       oled.drawstring(65, 02, "Y");
       oled.drawstring(65, 03, "N");
      }   
      else if(Menu == 4)
      {       
       oled.drawstring(65, 00, "N");
       oled.drawstring(65, 01, "N");
       oled.drawstring(65, 02, "N");
       oled.drawstring(65, 03, "Y");
      } 
 //=============Display voltage on the fifth line=======================//      
           oled.drawstring(00,4,"VOLTAGE:");
           oled.drawstring(71,4,".");
           oled.drawstring(93,4,"V");
           OLED_ShowNumber(58,4,Battery_Voltage/100,2);
           OLED_ShowNumber(81,4,Battery_Voltage%100,2);
   //  if(Battery_Voltage%100<10)   OLED_ShowNumber(45,4,0,2);
 //=============Display encoder data on the sixth line=======================//
        oled.drawstring(00,5,"POSITION:");
        OLED_ShowNumber(60,5,Position,5);
 //=============Display position target value on the seventh line=======================//
  oled.drawstring(00,6,"TARGET:");
  OLED_ShowNumber(60,6,Target,5);
//=============Display angle displacement sensor value and origin on the eighth line=======================//
  oled.drawstring(00,7,"ADC:");
  OLED_ShowNumber(25,7,Sensor,4);  
  oled.drawstring(55,7,"ORIGIN:");  // This value is the pendulum arm value when naturally hanging down
  OLED_ShowNumber(100,7,240,3);
//=============Refresh=======================//
  oled.display();
  DataScope();
}
/**************************************************************************
Function: External interrupt to read encoder data with 4x frequency functionality. Note that the external interrupt is triggered by an edge transition
Parameters: None
Return Value: None
**************************************************************************/
void READ_ENCODER_A() {
    if (digitalRead(ENCODER_A) == HIGH) {     
    if (digitalRead(ENCODER_B) == LOW)      Position++;  // Determine direction based on the level of the other phase
    else      Position--;
  }
    else {    
    if (digitalRead(ENCODER_B) == LOW)      Position--; // Determine direction based on the level of the other phase
    else     Position++;
  }
}
/**************************************************************************
Function: External interrupt to read encoder data with 4x frequency functionality. Note that the external interrupt is triggered by an edge transition
Parameters: None
Return Value: None
**************************************************************************/
void READ_ENCODER_B() {
    if (digitalRead(ENCODER_B) == LOW) { // If the interrupt is triggered on the falling edge
    if (digitalRead(ENCODER_A) == LOW)      Position++;// Determine direction based on the level of the other phase
    else      Position--;
  }
  else {   // If the interrupt is triggered on the rising edge
    if (digitalRead(ENCODER_A) == LOW)      Position--; // Determine direction based on the level of the other phase
    else     Position++;
  }
}
