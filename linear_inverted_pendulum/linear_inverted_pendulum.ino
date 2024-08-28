//////// OLED Display Pins ////////
#define OLED_DC_PIN 5  
//#define OLED_CS_PIN 5 // CS is directly connected to ground
#define OLED_CLK_PIN 8
#define OLED_MOSI_PIN 7
#define OLED_RESET_PIN 6

//////// TB6612 Driver Pins ////////
#define PWM_PIN 9
#define IN1_PIN 10
#define IN2_PIN 11

//////// Encoder Pins ////////
#define ENCODER_A_PIN 4 // Motor encoder pin
#define ENCODER_B_PIN 2 // Motor encoder pin 
#define MIDPOINT_VALUE 803 // Midpoint value for the pendulum in upright position, 0 when naturally hanging down
#define POSITION_VALUE 10000 // Position loop value

//////// Button Pins ////////
#define BUTTON_MENU_PIN 3
#define BUTTON_START_PIN 15
#define BUTTON_MINUS_PIN 16
#define BUTTON_PLUS_PIN 17
#define BUTTON_X_PIN 18

#include <PinChangeInt.h> // External interrupt
#include <MsTimer2.h> // Timer interrupt
#include <SSD1306.h>
#include <DATASCOPE.h> // PC-side software library

DATASCOPE data_scope; // Instantiate a PC-side object named 'data_scope'
SSD1306 oled(OLED_MOSI_PIN, OLED_CLK_PIN, OLED_DC_PIN, OLED_RESET_PIN, 0);

float position_value = 10000, sensor_value, motor_pwm; // Position loop value, angle displacement sensor value, motor PWM assignment
float balance_pwm, position_pwm; // Balance loop PWM, position loop PWM
int target_position = 10000; // Target position
unsigned char send_count, flash_send, stop_flag = 1;  // PC software related variables and stop flag
float balance_kp = 172, balance_kd = 172, position_kp = 66, position_kd = 66;  // PID coefficients
float menu_option = 1, amplitude1 = 1, amplitude2 = 1, amplitude3 = 1, amplitude4 = 1; // PID tuning parameters
int battery_voltage; // Battery voltage sampling variable

/**************************************************************************
 * Function: Send data to the PC-side software using a virtual oscilloscope
 * Author: Balance Car Home
 *************************************************************************/
void send_data_scope(void)
{
  data_scope.DataScope_Get_Channel_Data(sensor_value, 1);  // Display the first data
  data_scope.DataScope_Get_Channel_Data(position_value, 2); // Display the second data
  send_count = data_scope.DataScope_Data_Generate(2); 

  for (int i = 0; i < send_count; i++)
  {
    Serial.write(DataScope_OutPut_Buffer[i]);  
  }

  delay(50);  // The PC-side software must strictly control the send timing
}

/**************************************************************************
 * Function: Button scan
 * Author: Balance Car Home
 * Return Value: Button state, 1: Single click event, 0: No event.
 *************************************************************************/
unsigned char capture_click(void)
{
  static unsigned char button_flag = 1; // Button press/release flag

  if (button_flag && (digitalRead(BUTTON_MENU_PIN) == 0 || digitalRead(BUTTON_START_PIN) == 0 || digitalRead(BUTTON_MINUS_PIN) == 0 || digitalRead(BUTTON_PLUS_PIN) == 0))
  {
    button_flag = 0;
    if(digitalRead(BUTTON_MENU_PIN) == 0) return 1; // Menu button      
    if(digitalRead(BUTTON_START_PIN) == 0) return 2; // Start button
    if(digitalRead(BUTTON_MINUS_PIN) == 0) return 3; // Minus button 
    if(digitalRead(BUTTON_PLUS_PIN) == 0) return 4;  // Plus button
  }
  else if (digitalRead(BUTTON_MENU_PIN) == 1 && digitalRead(BUTTON_START_PIN) == 1 && digitalRead(BUTTON_MINUS_PIN) == 1 && digitalRead(BUTTON_PLUS_PIN) == 1)
  {
    button_flag = 1;
  }

  return 0; // No button pressed
}

/**************************************************************************
 * Function: Button scan
 * Parameters: Double click wait time
 * Return Value: Button state 0: No action 1: Single click 2: Double click 
 *************************************************************************/
u8 capture_double_click(u8 wait_time)
{
  static unsigned char button_flag, single_click_flag, double_click_flag; 
  static unsigned int single_click_count, long_press_count;
  char x_button_state = digitalRead(BUTTON_X_PIN);  // Read the X button

  if(x_button_state == 0)  long_press_count++;   // Long press flag
  else long_press_count = 0;

  if(x_button_state == 0 && button_flag == 0) button_flag = 1; 

  if(single_click_flag == 0)
  {
    if(button_flag == 1) 
    {
      double_click_flag++;
      single_click_flag = 1;  
    }

    if(double_click_flag == 2) 
    {
      double_click_flag = 0;
      single_click_count = 0;
      return 2; // Double click command
    }
  }

  if(x_button_state == 1) button_flag = 0, single_click_flag = 0;

  if(double_click_flag == 1)
  {
    single_click_count++;
    if(single_click_count > wait_time && long_press_count < wait_time)
    {
      double_click_flag = 0;
      single_click_count = 0; 
      return 1; // Single click command
    }

    if(long_press_count > wait_time)
    {
      double_click_flag = 0;
      single_click_count = 0; 
    }
  } 

  return 0;
}

/**************************************************************************
 * Function: Adjust parameters
 * Author: Balance Car Home
 *************************************************************************/
void adjust_parameters(void)
{   
  int position_increment = 200; // Distance moved by the user button for a single click     
  static int rotation_flag, rotation_count;
  int click_action = capture_click();   // Capture single button click

  if(click_action == 1) // Scroll menu to select the parameter to adjust
  {
    if(menu_option++ == 4) menu_option = 1;
  }   

  if(click_action == 2) // Control the start and stop of the motor
  {
    stop_flag = !stop_flag;  // Motor control flag
  }   

  if(click_action == 3) // PID-  PID parameter decrease
  {
    if(menu_option == 1)  balance_kp -= amplitude1;
    else if(menu_option == 2)  balance_kd -= amplitude2;
    else if(menu_option == 3)  position_kp -= amplitude3;
    else if(menu_option == 4)  position_kd -= amplitude4;
  }   

  if(click_action == 4) // PID+  PID parameter increase
  {
    if(menu_option == 1)  balance_kp += amplitude1;
    else if(menu_option == 2)  balance_kd += amplitude2;
    else if(menu_option == 3)  position_kp += amplitude3;
    else if(menu_option == 4)  position_kd += amplitude4;
  }   

  // Avoid negative parameters
  if(balance_kp <= 0) balance_kp = 0;
  if(balance_kd <= 0) balance_kd = 0;
  if(position_kp <= 0) position_kp = 0;
  if(position_kd <= 0) position_kd = 0;

  int double_click_action = capture_double_click(100); // Capture double click event
  if(double_click_action == 1) rotation_flag = 2; // Single click clockwise rotation
  else if(double_click_action == 2) rotation_flag = 1; // Double click counterclockwise rotation

  if(rotation_flag == 1) // Pendulum arm clockwise movement
  {
    target_position++;
    rotation_count++;  
    if(rotation_count == position_increment) rotation_flag = 0, rotation_count = 0;
  } 
  else if(rotation_flag == 2) // Pendulum arm counterclockwise movement
  {
    target_position--;
    rotation_count++;  
    if(rotation_count == position_increment) rotation_flag = 0, rotation_count = 0;
  }
}


/**************************************************************************
 * Function: Power function
 * Parameters: base, exponent
 * Return Value: base raised to the power of exponent
 *************************************************************************/
uint32_t oled_power(uint8_t base, uint8_t exponent)
{
  uint32_t result = 1;  
  while(exponent--) result *= base;    
  return result;
}

/**************************************************************************
 * Function: Display variable on OLED
 * Parameters: x: x-coordinate, y: line, num: variable to display, len: length of the variable
 *************************************************************************/
void oled_show_number(uint8_t x, uint8_t y, uint32_t num, uint8_t len)
{           
  u8 digit, temp;
  for(u8 i = 0; i < len; i++)
  {
    temp = (num / oled_power(10, len - i - 1)) % 10;
    oled.drawchar(x + 6 * i, y, temp + '0');
  }  
}

/**************************************************************************
 * Function: Inclination PD control
 * Parameters: sensor_value
 * Return Value: PWM for inclination control
 * Author: Balance Car Home
 *************************************************************************/
float balance_control(float sensor_value)
{  
  float bias = sensor_value - MIDPOINT_VALUE; // Calculate the balance angle midpoint
  static float last_bias, d_bias; // PID related variables
  int balance_pwm = (balance_kp / 10) * bias + (d_bias = bias - last_bias) * (balance_kd / 1000); // Calculate motor PWM for inclination control (PD control)
  last_bias = bias; // Store the last deviation
  return balance_pwm; // Return the control value
}

/**************************************************************************
 * Function: Position PD control 
 * Parameters: encoder_value
 * Return Value: PWM for position control
 * Author: Balance Car Home
 *************************************************************************/
float position_control(int encoder_value)
{  
  static float position_pwm, last_position_bias, position_bias, position_differential; // Define related variables
  static float least_position;

  least_position = encoder_value - target_position; // Calculate the deviation value
  position_bias *= 0.8; // First-order low-pass filter  
  position_bias += least_position * 0.2; // First-order low-pass filter  
  position_differential = position_bias - last_position_bias; // First-order difference
  last_position_bias = position_bias; // Store the last deviation
  position_pwm = (position_bias * position_kp / 1000) + (position_differential * position_kd / 10); // Position control 

  return position_pwm; // Return the control value
}

/**************************************************************************
 * Function: Assign value to the PWM register
 * Parameters: motor_pwm
 * Author: Balance Car Home
 *************************************************************************/
void set_pwm(int motor_pwm)
{
  if (motor_pwm < 0)
  {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);  // TB6612 level control
  }
  else
{
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH); // TB6612 level control
  }

  analogWrite(PWM_PIN, abs(motor_pwm) + 10); // Assign value to the PWM register
}

/**************************************************************************
 * Function: Abnormal motor shutdown
 * Return Value: 1: Abnormal  0: Normal
 *************************************************************************/
unsigned char turn_off_motor()
{
  unsigned char result = 0;

  if(sensor_value < (MIDPOINT_VALUE - 200) || sensor_value > (MIDPOINT_VALUE + 200) || stop_flag == 1)
  {              
    result = 1;   
    stop_flag = 1;                                         
    digitalWrite(IN1_PIN, LOW); // TB6612 level control                                           
    digitalWrite(IN2_PIN, LOW); // TB6612 level control
  }

  return result;
}

/**************************************************************************
 * Function: Get the average value of the angle displacement sensor
 * Parameters: channel, sample_times
 * Return Value: Average sensor value
 * Author: Balance Car Home
 *************************************************************************/
uint16_t get_adc_average(uint8_t channel, uint8_t sample_times)
{
  unsigned int total_value = 0;

  for(uint8_t i = 0; i < sample_times; i++)
  {
    total_value += analogRead(channel); // Capture analog value
  }

  return total_value / sample_times; // Calculate the average value
}   

/**************************************************************************
 * Function: 5ms control function, core code
 * Author: Balance Car Home
 *************************************************************************/
void control_loop()
{  
  static float voltage_sum, voltage_count; // Voltage sampling related variables
  static unsigned char position_control_counter;  // Variable for position control frequency division
  int temp_voltage; // Temporary variable

  sei(); // Enable global interrupts
  sensor_value = get_adc_average(5, 5); // Mean filtering
  balance_pwm = balance_control(sensor_value); // Angle PD control  

  if(++position_control_counter > 4)
  {
    position_pwm = position_control(position_value);
    position_control_counter = 0; 
  }

  motor_pwm = balance_pwm - position_pwm; // Calculate the final motor PWM

  // PWM parameter limit, exceeding 255 will disable the motor
  if(motor_pwm > 245) motor_pwm = 245;
  if(motor_pwm < -245) motor_pwm = -245;

  if(turn_off_motor() == 0)
  {
    set_pwm(motor_pwm); // Output motor control value
  }

  adjust_parameters(); // PID adjustment

  temp_voltage = analogRead(0);  // Sample the battery voltage
  voltage_count++; // Average value counter
  voltage_sum += temp_voltage; // Accumulate multiple samples

  if(voltage_count == 200)
  {
    battery_voltage = voltage_sum * 0.05371 / 2;
    voltage_sum = 0;
    voltage_count = 0;
  }
}

/**************************************************************************
 * Function: External interrupt to read encoder data with 4x frequency functionality.
 * Note that the external interrupt is triggered by an edge transition.
 *************************************************************************/
void read_encoder_a() {
  if (digitalRead(ENCODER_A_PIN) == HIGH) {     
    if (digitalRead(ENCODER_B_PIN) == LOW) position_value++;  // Determine direction based on the level of the other phase
    else position_value--;
  } else {    
    if (digitalRead(ENCODER_B_PIN) == LOW) position_value--; // Determine direction based on the level of the other phase
    else position_value++;
  }
}

/**************************************************************************
 * Function: External interrupt to read encoder data with 4x frequency functionality.
 * Note that the external interrupt is triggered by an edge transition.
 *************************************************************************/
void read_encoder_b() {
  if (digitalRead(ENCODER_B_PIN) == LOW) { // If the interrupt is triggered on the falling edge
    if (digitalRead(ENCODER_A_PIN) == LOW) position_value++; // Determine direction based on the level of the other phase
    else position_value--;
  } else {   // If the interrupt is triggered on the rising edge
    if (digitalRead(ENCODER_A_PIN) == LOW) position_value--; // Determine direction based on the level of the other phase
    else position_value++;
  }
}

/**************************************************************************
 * Function: Initialization equivalent to the Main function in STM32
 * Author: Balance Car Home
 *************************************************************************/
void setup()
{ 
  TCCR1B = (TCCR1B & 0xF8) | 1; // Adjust the counter frequency division, increase frequency to 31.374KHZ

  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
  oled.clear(); // Clears the screen and buffer      

  pinMode(IN1_PIN, OUTPUT); // TB6612 direction control pin
  pinMode(IN2_PIN, OUTPUT); // TB6612 direction control pin
  pinMode(PWM_PIN, OUTPUT); // TB6612 speed control pin
  digitalWrite(IN1_PIN, LOW); // TB6612 control pin pulled low
  digitalWrite(IN2_PIN, LOW); // TB6612 control pin pulled low
  digitalWrite(PWM_PIN, LOW); // TB6612 control pin pulled low

  pinMode(ENCODER_A_PIN, INPUT); // Encoder pin
  pinMode(ENCODER_B_PIN, INPUT); // Encoder pin

  Serial.begin(128000); // Start serial communication
  delay(200); // Delay to wait for initialization to complete

  MsTimer2::set(5, control_loop); // Use Timer2 to set 5ms timer interrupt
  MsTimer2::start(); // Enable interrupt

  attachInterrupt(0, read_encoder_a, CHANGE); // Enable external interrupt 
}

/**************************************************************************
 * Function: Main loop
 *************************************************************************/
void loop()                     
{
  // Display Balance PD control P parameter on the first line
  oled.drawstring(0, 0, "B-KP:");
  oled_show_number(30, 0, balance_kp, 3);
  oled.drawstring(85, 0, "A:");
  oled_show_number(100, 0, amplitude1, 3);

  // Display Balance PD control D parameter on the second line
  oled.drawstring(0, 1, "B-KD:");
  oled_show_number(30, 1, balance_kd, 3);
  oled.drawstring(85, 1, "A:");
  oled_show_number(100, 1, amplitude2, 3);

  // Display Position PD control P parameter on the third line
  oled.drawstring(0, 2, "P-KP:");
  oled_show_number(30, 2, position_kp, 3);
  oled.drawstring(85, 2, "A:");
  oled_show_number(100, 2, amplitude3, 3);

  // Display Position PD control D parameter on the fourth line
  oled.drawstring(0, 3, "P-KD:");
  oled_show_number(30, 3, position_kd, 3);
  oled.drawstring(85, 3, "A:");
  oled_show_number(100, 3, amplitude4, 3);

  // Scrolling menu to select the PD parameter to adjust                      
  if(menu_option == 1)
  {
    oled.drawstring(65, 0, "Y");
    oled.drawstring(65, 1, "N");
    oled.drawstring(65, 2, "N");
    oled.drawstring(65, 3, "N");
  }
  else if(menu_option == 2)
  {
    oled.drawstring(65, 0, "N");
    oled.drawstring(65, 1, "Y");
    oled.drawstring(65, 2, "N");
    oled.drawstring(65, 3, "N");
  }   
  else if(menu_option == 3)
  {     
    oled.drawstring(65, 0, "N");
    oled.drawstring(65, 1, "N");
    oled.drawstring(65, 2, "Y");
    oled.drawstring(65, 3, "N");
  }   
  else if(menu_option == 4)
  {       
    oled.drawstring(65, 0, "N");
    oled.drawstring(65, 1, "N");
    oled.drawstring(65, 2, "N");
    oled.drawstring(65, 3, "Y");
  } 

  // Display voltage on the fifth line      
  oled.drawstring(0, 4, "VOLTAGE:");
  oled.drawstring(71, 4, ".");
  oled.drawstring(93, 4, "V");
  oled_show_number(58, 4, battery_voltage / 100, 2);
  oled_show_number(81, 4, battery_voltage % 100, 2);

  // Display encoder data on the sixth line
  oled.drawstring(0, 5, "POSITION:");
  oled_show_number(60, 5, position_value, 5);

  // Display position target value on the seventh line
  oled.drawstring(0, 6, "TARGET:");
  oled_show_number(60, 6, target_position, 5);

  // Display angle displacement sensor value and origin on the eighth line
  oled.drawstring(0, 7, "ADC:");
  oled_show_number(25, 7, sensor_value, 4);  
  oled.drawstring(55, 7, "ORIGIN:");  // This value is the pendulum arm value when naturally hanging down
  oled_show_number(100, 7, 240, 3);

  // Refresh the display
  oled.display();

  // Send data to the PC-side software
  send_data_scope();
}

