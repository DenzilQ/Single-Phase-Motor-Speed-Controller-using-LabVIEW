#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>

#define CLK 2 //Rotary Encoder CLK Pin on Pin 2
#define SpeedSensorPin 3 //Motor Speed Sensor Interrupt1 on Pin 3 to measure pulse from HALL Effect Sensor
#define Relay1 4 //Relay 1 on Pin 4 to start/stop Motor
#define Relay2 5 //Relay 2 on Pin 5 to change Direction of Motor
#define PWMPin 6 //Motor Speed Control on PWM on Pin 6 to control Motor Speed
#define DT 8 //Rotary Encoder DT Pin on Pin 8
#define DirectionButton 9 //Direction Button on Pin 9 to change motor direction
#define StartStopButton 10 //Start/Stop Button on Pin 10 to start/stop motor

const int debounceDelay = 100; //The debounce time for switches
const int lcdinterval = 2000; //LCD refresh interval in milliseconds
const int lvinterval = 250; //LabVIEW refresh interval in milliseconds
const int RPMcorrection = 86; //RPM Correction Factor

unsigned int RPM; //Stores RPM Variable
unsigned int tachocount; //Stores pulses from tachometer

unsigned long lasttime; //Time stamp at last tachometer pulse for RPM calculation
unsigned long lastcounttime = 0; //Time stamp for the last time the tachometer pulsed to determine if the motor was stuck
unsigned long currentMillis = 0; //Used with LCD and LabVIEW interval to save the cuurrent time so that the LCD/LabVIEW can be updated
unsigned long lcdpreviousMillis = 0; //Used with LCD interval to save the previous time at which the LCD was cleared
unsigned long lvpreviousMillis = 0; //Used with LabVIEW interval to save the previous time at which the LabVIEW data was sent
unsigned long lastDebounceTime = 0;  //The last time the input pin was toggled to help with debouncing
unsigned long startTime = 0;

int i = 0; //Variables to store loop counter
int PWMSpeedSignal = 76, speedmin = 64, speedmax = 79; //Variables to store the Speed Control PWM Signals and min/max PWM Signals 
int buttonState1, buttonState2, lastButtonState1 = HIGH, lastButtonState2 = HIGH; //Stores the states of the buttons
int startFlag = 0, PIDFlag = 0, MotorStatus = 0, MotorDirection = 0; //Variables to store the Status of the motor (1 - ON, 0 - OFF) and the Direction of the motor (1 - Forward, 0 - Reverse)

const int speedgain = 1, minRPM = 300, maxRPM = 5000; //Variables to store the speed gain, min/max RPM speed
const float voltagemin = 45, currentmax = 2; //Variables for storing protection values for voltage and current    //Adjust max current for 2A
float v_adc[75], i_adc[75], voltage = 0, current = 0, power = 0;  //Variables for measuring and protection for voltage, current and power

//LabVIEW Variables
char labview = ""; //Store LabVIEW commands that are received
String PIDSpeedSignal; //Store PID Speed control signals from LabVIEW
float voltagelv, currentlv, powerlv; //Store variables to send to LabVIEW
const int voltageOffset = 1000, currentOffset = 10000, powerOffset = 1000; //Stores offset variables for LabVIEW to be formatted and displayed properly
unsigned int RPMlv = 0; //Store RPM variable for LabVIEW

LiquidCrystal_I2C lcd(0x27,20,4);  //Set the 20x4 LCD address to match the I2C(0x27) address
Encoder myEnc (2,8); //Initialize Rotary Encoder Pins
long position  = -999;

void setup() 
{
  pinMode(StartStopButton, INPUT_PULLUP); //Switch 1 for starting/stoppping as input with pullup resistor
  pinMode(DirectionButton, INPUT_PULLUP); //Switch 2 for changing direction as input with pullup resistor
  pinMode(SpeedSensorPin, INPUT); //Speed sensor pin as input for Hall Effect Sensor
  pinMode(Relay1, OUTPUT); //Relay 1 as output
  pinMode(Relay2, OUTPUT); //Relay 2 as output
  pinMode(PWMPin, OUTPUT); //PWM Pin to control motor speed

  attachInterrupt(1, SpeedSensorInterrupt, FALLING); //Call ISR SpeedSensorInterrupt whenever a falling edge event is detected on Speed Sensor

  Serial.begin(9600); //Start Serial Communication
  lcd.init(); //Initialize LCD Screen
  lcd.clear(); //Clear LCD Screen
  lcd.backlight(); //Turn on LCD Backlight
  lcd.setCursor(1,0);
  lcd.print("Single Phase Motor");
  lcd.setCursor(1,1);
  lcd.print("Speed & Direction");
  lcd.setCursor(3,2);
  lcd.print("PID Controller");
  lcd.setCursor(3,3);
  lcd.print("Using LabVIEW");
  delay(5500);
  
  long newPos = myEnc.read(); //Reads the initial position of the Rotary Encoder
  if (newPos != position) //If the Rotary Encoder position has changed
  {
    position = newPos; //Update the position with the new position
  }
  lcd.clear();
}

void loop()
{
  ButtonCheck(); //Checks to see if Switch 1 or Switch 2 is pressed for starting/stopping motor or changing direction of motor
  
  if (MotorStatus == 1) //If motor is on then manual speed control can be used
  {
    SpeedControl(); //Speed control of motor using Rotary Encoder
  }
  
  Sensors(); //Measures the voltage, current and power parameters of the motor
  MotorProtection(); //Checks the various measured parameters to ensure that the motor is not under a fault condition
  Display(); //Displays the various parameters to the LCD. This include voltage, current, power, speed and motor status
  LabVIEWComm(); //Sends and receive data to/from LabVIEW via serial communication for controlling of the motor as well as viewing of motor parameters
}

void ButtonCheck() //Checks the button states of Switch 1 and Switch 2
{
  int reading1 = digitalRead(StartStopButton); //Reads the state of the change direction button
  if (reading1 != lastButtonState1) // Checks to see if the button state changed due to noise or pressing:
  {  
    lastDebounceTime = millis(); //Reset the debouncing timer
  }

  if ((millis() - lastDebounceTime) > (debounceDelay)) //Checks to see if the debounce timer has elapsed
  {
    if (reading1 != buttonState1) //If the button state has changed:
    {
      buttonState1 = reading1;
      if (buttonState1 == LOW) //If the button is still low, therefore pressed then change direction of the motor
      {
        StartStopMotor(); //Starts or Stops the motor
      }
    }
  }
  lastButtonState1 = reading1; //Save the button state reading. Next time through the loop, it'll be the lastButtonState so it can be used to determine if the switch is pressed
  
  int reading2 = digitalRead(DirectionButton); //Reads the state of the change direction button
  if (reading2 != lastButtonState2) // Checks to see if the button state changed due to noise or pressing:
  {
    lastDebounceTime = millis(); //Reset the debouncing timer
  }

  if ((millis() - lastDebounceTime) > (debounceDelay)) //Checks to see if the debounce timer has elapsed
  {
    if (reading2 != buttonState2) //If the button state has changed:
    {
      buttonState2 = reading2;
      if (buttonState2 == LOW) //If the button is still low, therefore pressed then change direction of the motor
      {
        MotorDirectionControl(); //Change direction of motor
      }
    }
  }
  lastButtonState2 = reading2; //Save the button state reading. Next time through the loop, it'll be the lastButtonState so it can be used to determine if the switch is pressed
}

void Display() //Displays various parameters on LCD *Finished
{
  currentMillis = millis(); //Stores the current time so it can be used for LCD refresh
  if (currentMillis - lcdpreviousMillis >= lcdinterval) //Refereshes the LCD Display if the LCD Interval has beene exceeded
  {
    lcdpreviousMillis = currentMillis; //Updates the previous time the LCD was cleared with the current time
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Voltage:");
    lcd.print(voltage,1);
    lcd.print("V");
    lcd.setCursor(0,1);
    lcd.print("Current:");
    lcd.print(current,2);
    lcd.print("A");
    lcd.setCursor(0,2);
    lcd.print("Power:");
    lcd.print(power,1);
    lcd.print("W");
    lcd.setCursor(0,3);
    lcd.print("RPM:");
    lcd.print(RPM);

    lcd.setCursor(10,3); 
    if((MotorStatus == 1) && (MotorDirection == 1)) //Checks to see if Motor is on in Forward Motor Direction
    {
      lcd.print("ON:Forward");
    }
  
    if((MotorStatus == 1) && (MotorDirection == 0)) //Checks to see if Motor is on in Reverse Motor Direction
    {
      lcd.print("ON:Reverse");
    }
  
    if(MotorStatus == 0) //Checks to see if Motor is off
    {
      lcd.print("Motor OFF");
    }
  }
}

void LabVIEWComm() //Communication for LabVIEW *Finished, may need reviewing and tweaking
{
  currentMillis = millis(); //Stores the current time so it can be used for LabVIEW refresh
  if (currentMillis - lvpreviousMillis >= lvinterval) //Refereshes the LabVIEW Display if the LanVIEW Interval has beene exceeded
  {
    lvpreviousMillis = currentMillis; //Updates the previous time data was sent to LabView with the current time
  
    //Sends the various parameters to labview for display
    Serial.print(voltagelv,1);
    Serial.print('\t');
    Serial.print(currentlv,2);
    Serial.print('\t');
    Serial.print(powerlv,1);
    Serial.print('\t');
    Serial.print(RPMlv);
  
    if((MotorStatus == 1) && (MotorDirection == 1)) //Checks to see if Motor is on in Forward Direction
    {
      Serial.println("Motor ON: Forward Direction");
    }
  
    if((MotorStatus == 1) && (MotorDirection == 0)) //Checks to see if Motor is on in Reverse Direction
    {
      Serial.println("Motor ON: Reverse Direction");
    }
  
    if(MotorStatus == 0) //Checks to see if Motor is off
    {
      Serial.println("Motor OFF");
    }
  }
  
  if (Serial.available()>0) //Checks to see if any commands has been received from LabVIEW
  {
    labview = Serial.read(); //Reads the command that has been sent from LabVIEW
    if(labview == 'a' && MotorStatus == 1) //Checks to see if 'a' is read from LabVIEW and that the Motor is on, any value after 'a' represents the PID Speed Signal. 
    {
      while(1)
      {
        labview = Serial.read(); //Continues to read the PID Speed Signal from LabVIEW
        if(isDigit(labview)) //Checks to see if the PID Speed Signal from LabVIEW is a digit/number
        {
          PIDSpeedSignal += labview; //Populates the PID Speed Signal string with the numbers read after the first 'a' value
        }
        
        else if(isAlpha(labview)) //Checks to see if 'a' is read from the serial port, if the last value is 'a' it signifies that the PID/PWM/Speed signal is sent
        {
          //Shows a PID indicator on the LCD to signify that PID is on
          lcd.setCursor(17,0);
          lcd.print("PID"); 

          if(MotorDirection == 0)
          {
            speedmin = 64;
            speedmax = 130; //125 stable
          }

          if(MotorDirection == 1)
          {
            speedmin = 56;
            speedmax = 130; //100 stable
            //150
          }
          
          PWMSpeedSignal = PIDSpeedSignal.toInt(); //Converts the PID Speed Signal from LabVIEW to a number and stores it as the PWM Speed Signal
          PWMSpeedSignal = (PWMSpeedSignal + speedmin); //Offsets the PWM Speed Signal by the minimum PWM speed signal
          
          //Checks to see if the PWM Speed Signal is within minimum and maximum range, also temporarily increases the maximum range to allow PID to have more control of the motor
          if (PWMSpeedSignal <= speedmin)
          {
            PWMSpeedSignal = speedmin;
          }
          
          if (PWMSpeedSignal >= speedmax) //Adjust to +50 for load, for no load stability use lower 
          {
            PWMSpeedSignal = speedmax;
          }
          
          currentMillis = millis();
          if((currentMillis - startTime >= 3000) && (startFlag == 0)) //Waits a few seconds so that PID speed control can be used when the motor is first started
          {
            startFlag = 1;
            PIDSpeedSignal = ""; //Clears the PIDSpeedSignal string
            PWMSpeedSignal = 0;
          }

          else if(startFlag == 1) //Once the few seconds has passed the PID speed control will then be allowed
          {
            analogWrite(PWMPin, PWMSpeedSignal); //Adjusts voltage at the PWM Pin to control the Motor speed accordingly
            PIDSpeedSignal = ""; //Clears the PIDSpeedSignal string
          }
          break;
        }
      }
    }

    else if(labview != 'a')
    {
      speedmin = 64;
      speedmax = 79;
    }

    switch(labview)
    {
      case 'O': //Start Motor Command
        if(MotorStatus == 0) //Checks to see if the motor is off
        {
          StartStopMotor();
        }
        
        else if(MotorStatus == 1) //Checks to see if the motor is on
        {
          lcd.setCursor(10,3); 
          lcd.print("Motor ON   ");
        }
        delay(500);
        break;
      
      case 'S': //Stop Motor Command
        if(MotorStatus == 1) //Checks to see if the motor is on
        {
          StartStopMotor();
        }
        
        else if(MotorStatus == 0) //Checks to see if the motor is off
        {
          lcd.setCursor(10,3); 
          lcd.print("Motor OFF  ");
        }
        delay(500);
        break;

      case 'F': //Forward Motor Direction Command
        if((MotorStatus == 1) && (MotorDirection == 1)) //Checks to see if the motor is in the forward direction
        {
          lcd.clear();
          lcd.print("Motor ON:");
          lcd.setCursor(0,1);
          lcd.print("Forward Direction");
        }

        else
        {
          MotorDirectionControl();
        }
        break;
      
      case 'R': //Reverse Motor Command
        if((MotorStatus == 1) && (MotorDirection == 0)) //Checks to seee if the motor is in the reverse direction
        {
          lcd.clear();
          lcd.print("Motor ON:");
          lcd.setCursor(0,1);
          lcd.print("Reverse Direction");
        }

        else
        {
          MotorDirectionControl();
        }
        break;

      case 'U': //Speed Increase Command
        if(MotorStatus==0) //Checks to see if the motor is off
        {
          lcd.clear();
          lcd.print("Motor OFF");
        }

        else if(MotorStatus == 1)
        {
          Serial.print(voltagelv,1);
          Serial.print('\t');
          Serial.print(currentlv,2);
          Serial.print('\t');
          Serial.print(powerlv,1);
          Serial.print('\t');
          Serial.print(RPMlv);
          
          PWMSpeedSignal = PWMSpeedSignal + speedgain;
          if (RPM >= (maxRPM - 1500)) //Checks to see if the RPM is above a maximum speed 
          {
            PWMSpeedSignal = PWMSpeedSignal - 1;
            lcd.setCursor(10,3);
            lcd.print("          ");
            lcd.setCursor(10,3);
            lcd.print("MAX Speed");
            Serial.println("Speed: Maximum Speed");
          }
          
          else
          {
            lcd.setCursor(10,3); 
            lcd.print("          ");
            lcd.setCursor(10,3); 
            lcd.print("Speed ++");
            Serial.println("Speed: Speed Increased");
          }
          
          analogWrite(PWMPin, PWMSpeedSignal); //Sends the PWM Signal, 0-5V,  to Motor Speed Controller
          delay(300);
        }
        break;

      case 'D': //Speed Decrease Command
        if(MotorStatus==0) //Checks to see if the motor is off
        {
          lcd.clear();
          lcd.print("Motor OFF");
        }

        else if(MotorStatus == 1)
        {
          Serial.print(voltagelv,1);
          Serial.print('\t');
          Serial.print(currentlv,2);
          Serial.print('\t');
          Serial.print(powerlv,1);
          Serial.print('\t');
          Serial.print(RPMlv);
          
          PWMSpeedSignal = PWMSpeedSignal - speedgain;
          if (RPM <= (minRPM + 900)) //Checks to see if the RPM is below the a minimum speed
          {
            PWMSpeedSignal = PWMSpeedSignal + 1;
            
            lcd.setCursor(10,3);
            lcd.print("          ");
            lcd.setCursor(10,3);
            lcd.print("MIN Speed");
            Serial.println("Speed: Minimum Speed");
          }
          
          else
          {
            lcd.setCursor(10,3); 
            lcd.print("          ");
            lcd.setCursor(10,3); 
            lcd.print("Speed --");
            Serial.println("Speed: Speed Decreased");
          }
          
          analogWrite(PWMPin, PWMSpeedSignal); //Sends the PWM Signal, 0-5V,  to Motor Speed Controller
          delay(300);
        }
        break;
    }
  }
}

void SpeedSensorInterrupt() //Measures pulses from HALL effect sensor to calculate RPM Speed
{
  tachocount++; //Increase the tachometer count everytime the speed sensor pulses
  unsigned long period = micros() - lasttime; //Finding the period of time taken between interrupt triggering in Microseconds 
  float time_in_sec = ((float)period + RPMcorrection) / 1000000; //Converts Microseconds to seconds and considers the RPM Correction Factor
  float preRPM = 60 / time_in_sec; //Converts time taken in seconds to an RPM value by diving 60 (60 seconds in a minute) by time taken
  RPM = preRPM / 8 ;//Can be used if there are multiple pulses used, change the 8 to the amount of pulses such as an 8 or 6 pulse hall effect sensor
  lasttime = micros(); //Saves current time to last time variable
}

void SoftStart() //Soft starts the motor by increasing RPM to a specific speed using the Speed Signals
{
  if (MotorDirection == 1)
  {
    //Adjusts the current speed signals for Forward Direction
    PWMSpeedSignal = 68;
  }

  else if (MotorDirection == 0)
  {
    //Adjusts the current speed signal for Reverse direction
    PWMSpeedSignal = 74;
  }
  
  lcd.clear();
  while(RPM<600)
  {
    Sensors();
    
    lcd.setCursor(0,0);
    lcd.print("Voltage:");
    lcd.print(voltage,1);
    lcd.print("V ");
    lcd.setCursor(0,1);
    lcd.print("Current:");
    lcd.print(current,1);
    lcd.print("A");
    lcd.setCursor(0,2);
    lcd.print("Power:");
    lcd.print(power,1);
    lcd.print("W ");
    lcd.setCursor(0,3);
    lcd.print("RPM:    ");
    lcd.setCursor(4,3);
    lcd.print(RPM);
    lcd.setCursor(10,3);
    lcd.print("Starting  ");
      
    Serial.print(voltagelv,1);
    Serial.print('\t');
    Serial.print(currentlv,2);
    Serial.print('\t');
    Serial.print(powerlv,1);
    Serial.print('\t');
    Serial.print(RPMlv);
    
    if(MotorDirection == 1)
    {
      Serial.println("Motor Starting: Forward Direction");
      analogWrite(PWMPin, PWMSpeedSignal); //Ensures that TRIAC is turned on after Relay is turned on to prevent sparks on Relay Contacts
      PWMSpeedSignal++;
      if (PWMSpeedSignal >= 74)
      {
        PWMSpeedSignal = 74;
      }
    }

    else if(MotorDirection == 0)
    {
      Serial.println("Motor Starting: Reverse Direction");
      analogWrite(PWMPin, PWMSpeedSignal); //Ensures that TRIAC is turned on after Relay is turned on to prevent sparks on Relay Contacts
      PWMSpeedSignal++;
      if (PWMSpeedSignal >= 77)
      {
        PWMSpeedSignal = 77;
      }
    }
    delay(250);
  }
  current = 0;
  startTime = millis();
}

void SoftStop() //Stops the motor and shows RPM as it slows down
{
  analogWrite(PWMPin, 0); //Turns off the TRIAC
  delay(50); //Ensures that TRIAC is off to prevent sparks on Relay Contacts
  digitalWrite(Relay1, LOW); //Turns off Starter Relay
    
  voltagelv = voltageOffset;
  currentlv = currentOffset;
  powerlv = powerOffset;
   
  while(RPM>=(minRPM-50))
  {
    RPMlv = RPM + currentOffset;
    Serial.print(voltagelv,1);
    Serial.print('\t');
    Serial.print(currentlv,2);
    Serial.print('\t');
    Serial.print(powerlv,1);
    Serial.print('\t');
    Serial.print(RPMlv);
    
    if (MotorDirection == 0)
    {
      Serial.println("Switching Direction: Reverse Direction");
    }

    else if (MotorDirection == 1)
    {
      Serial.println("Switching Direction: Forward Direction");
    }
    delay(250);
  }
  startFlag = 0;
  delay(1200);
}

void StartStopMotor() //Start or Stop Motor Function
{
  if(MotorStatus == 0)
  {
    digitalWrite(Relay2, HIGH); //Motor Direction Relay Energized for Forward direction
    digitalWrite(Relay1, HIGH); //Starter Relay Energized
    MotorStatus = 1; //Indicates Motor is on
    MotorDirection = 1; //Indicates Motor is in forward direction
    SoftStart();
  }

  else if(MotorStatus == 1)
  {
    analogWrite(PWMPin, 0); //Turns off the TRIAC
    delay(50); //Slight delay to ensure that the TRIAC is off
    digitalWrite(Relay1, LOW); //Stops Motor by de-energizing starter relay
    digitalWrite(Relay2, LOW); //Motor Direction Relay De-energized
    MotorStatus = 0; //Indicates Motor is off
    MotorDirection = 0; //Indicates Motor is in reverse direction since Motor Direction relay was de-energized

    lcd.clear();
    while(RPM>=(minRPM-50))
    {
      Sensors();

      lcd.setCursor(0,0);
      lcd.print("Voltage:");
      lcd.print(voltage,1);
      lcd.print("V");
      lcd.setCursor(0,1);
      lcd.print("Current:");
      lcd.print(current,1);
      lcd.print("A");
      lcd.setCursor(0,2);
      lcd.print("Power:");
      lcd.print(power,1);
      lcd.print("W");
      lcd.setCursor(0,3);
      lcd.print("RPM:    ");
      lcd.setCursor(4,3);
      lcd.print(RPM);
      lcd.setCursor(10,3);
      lcd.print("Stopping  ");
    
      RPMlv = RPM + currentOffset;
      Serial.print(voltagelv,1);
      Serial.print('\t');
      Serial.print(currentlv,2);
      Serial.print('\t');
      Serial.print(powerlv,1);
      Serial.print('\t');
      Serial.print(RPMlv);
      Serial.println("Motor Stopping");
      
      delay(250);
    }
    startFlag = 0; //Indicates that the motor has started
  }
}

void MotorDirectionControl() //Conntrol Motor Direction Function
{
  lcd.clear();
  if((MotorStatus == 1) && (MotorDirection == 1)) //Checks to see if motor is on and in the forward Direction
  {
    MotorDirection = 0; //Indicates Motor is in Reverse Direction
    lcd.print("Switching");
    lcd.setCursor(0,1);
    lcd.print("Direction:");
    lcd.setCursor(0,2);
    lcd.print("Reverse Direction");

    SoftStop();

    digitalWrite(Relay2, LOW); //Motor Direction Relay De-energized for Reverse
    digitalWrite(Relay1, HIGH); //Turns on Starter Relay

    SoftStart();
  }

  else if((MotorStatus == 1) && (MotorDirection == 0)) //Checks to see if motor is on and in the reverse direction
  {
    MotorDirection = 1; //Indicates Motor is in Forward Direction
    lcd.print("Switching");
    lcd.setCursor(0,1);
    lcd.print("Direction:");
    lcd.setCursor(0,2);
    lcd.print("Forward Direction");    
    
    SoftStop();

    digitalWrite(Relay2, HIGH); //Motor Direction Relay Energized for Forward Direction
    digitalWrite(Relay1, HIGH); //Starter Relay energized
    
    SoftStart();
  }

  if(MotorStatus == 0) //Checks to see if Motor is off
  {
    lcd.print("Motor OFF");
    lcd.setCursor(0,1);
    lcd.print("Start Motor");
    lcd.setCursor(0,2);
    lcd.print("To Enable");
    lcd.setCursor(0,3);
    lcd.print("Direction Control");

    Serial.print(voltagelv,1);
    Serial.print('\t');
    Serial.print(currentlv,2);
    Serial.print('\t');
    Serial.print(powerlv,1);
    Serial.print('\t');
    Serial.print(RPMlv);
    Serial.println("Start Motor to Enable Direction Control");
    delay(2200);
  }
}

void SpeedControl() //Speed Controlled by Rotary Encoder
{
  long newPos = myEnc.read(); //Reads the position of the Rotary Encoder
  delay(150);
  if (newPos != position) //If the Rotary Encoder position has changed
  {
    //Sends readings to LabView to maintain serial offset and format
    Serial.print(voltagelv,1);
    Serial.print('\t');
    Serial.print(currentlv,2);
    Serial.print('\t');
    Serial.print(powerlv,1);
    Serial.print('\t');
    Serial.print(RPMlv);
    
    if(newPos < position) //If Rotary Encoder position has changed clockwise
    {
      PWMSpeedSignal = PWMSpeedSignal + speedgain;
      if (RPM >= (maxRPM - 1500)) //Checks to see if the RPM is above a maximum speed 
      {
        PWMSpeedSignal = PWMSpeedSignal - 1;
        lcd.setCursor(10,3);
        lcd.print("          ");
        lcd.setCursor(10,3);
        lcd.print("MAX Speed");
        Serial.println("Speed: Maximum Speed");
      }
  
      else
      {
        lcd.setCursor(10,3); 
        lcd.print("          ");
        lcd.setCursor(10,3); 
        lcd.print("Speed ++");
        Serial.println("Speed: Speed Increased");
      }
    }

    else if(newPos > position) //If Rotary Encoder position has changed counter clockwise
    {
      PWMSpeedSignal = PWMSpeedSignal - speedgain;
      if (RPM <= (minRPM + 900)) //Checks to see if the RPM is below the a minimum speed
      {
        PWMSpeedSignal = PWMSpeedSignal + 2;
        lcd.setCursor(10,3);
        lcd.print("          ");
        lcd.setCursor(10,3);
        lcd.print("MIN Speed");
        Serial.println("Speed: Minimum Speed");
      }

      else
      {
        lcd.setCursor(10,3); 
        lcd.print("          ");
        lcd.setCursor(10,3); 
        lcd.print("Speed --");
        Serial.println("Speed: Speed Decreased");
      }
    }
    position = newPos;
    analogWrite(PWMPin, PWMSpeedSignal); //Sends the PWM Signal, 0-5V,  to Motor Speed Controller
    delay(500);
  }
  position = newPos;
}

void Sensors() //Measure Voltage, Current, Power of Motor
{
  for(i=0;i<75;i++) //Takes 75 ADC readings from pin A0 and A1 for voltage and current measurements
  {
    v_adc[i] = analogRead(A0);
    voltage = voltage + v_adc[i];
    
    i_adc[i] = analogRead(A1);
    current = current +  i_adc[i];
  }

  voltage = (voltage/75); //Finds the average readings*/
  voltage = ((voltage * 5)/1024); //Converts the readings to its 5V equivalent
  voltage = ((13.512 * (pow(voltage, 2))) - (59.339 * voltage) + 113.49); //Scale and calibrate voltage to match measured voltage
  //voltage = (35.022*(voltage) - 16.931); //Linearly Scale
  
  current = (current/75); //Finds the average readings
  current = ((current * 5)/1024); //Converts the readings to its 5V equivalent value

  if (MotorDirection == 1) //If Motor is in Forward Direction
  {
     current = ((4.2873 * (pow(current, 2))) - (8.2188* current) + 4.5228); //Scale and calibrate current to match measured current 
  }
  
  else if (MotorDirection == 0) //If Motor is in Reverse Direction
  {
    voltage = voltage + 2.8; //Voltage Correction Factor
    current = current + 0.1; //Current Correction Factor
    current = ((4.2873 * (pow(current, 2))) - (8.2188* current) + 4.5228); //Scale and calibrate current to match measured current 
  }
  
  power = voltage * current;

  //Resets voltage, current and power to 0 once the motor is off
  if (MotorStatus == 0)
  {
    voltage = 0;
    current = 0;
    power = 0;
    RPM = 0;
  }

  //Use +voltageOffset / +currentOffset to keep consistent offset with reading buffer in LabVIEW so that they can be displayed properly
  voltagelv = voltage + voltageOffset;
  currentlv = current + currentOffset;
  powerlv = power + powerOffset;
  RPMlv = RPM + currentOffset;
}

void MotorProtection () //Incorporates various Motor Protection Function
{
  if ((voltage < voltagemin) && (MotorStatus == 1)) //Under voltage protection
  {
    lcd.clear();
    analogWrite(PWMPin,0);
    delay(50);
    digitalWrite(Relay1, LOW);
    digitalWrite(Relay2,LOW);
    
    voltagelv = voltageOffset;
    currentlv = currentOffset;
    powerlv = powerOffset;
    
    while(1)
    {
      lcd.setCursor(0,0);
      lcd.print("Motor OFF");
      lcd.setCursor(0,1);
			lcd.print("Fault Detected:");
			lcd.setCursor(0,2);
			lcd.print("Low Voltage");
      lcd.setCursor(0,3);
      lcd.print(voltage);
      lcd.print("V");
      
      //Sending fault to LabVIEW
      Serial.print(voltagelv,1);
      Serial.print('\t');
      Serial.print(currentlv,2);
      Serial.print('\t');
      Serial.print(powerlv,1);
      Serial.print('\t');
      RPMlv =  RPM + currentOffset;
      if (RPM <= minRPM)
      {
        RPMlv = currentOffset;
      }
      Serial.print(RPMlv);
      Serial.println("Motor OFF, Fault Detected:                Low Voltage - Voltage below 45V");
    }
  }

  if ((current >= currentmax) && (MotorStatus == 1) && (RPM>minRPM)) //Over current protection
  {
    lcd.clear();
    analogWrite(PWMPin,0);
    delay(50);
    digitalWrite(Relay1, LOW);
    digitalWrite(Relay2, LOW);

    voltagelv = voltageOffset;
    currentlv = currentOffset;
    powerlv = powerOffset;
    
    while(1)
    {
      lcd.setCursor(0,0);
      lcd.print("Motor OFF");
      lcd.setCursor(0,1);
			lcd.print("Fault Detected:");
			lcd.setCursor(0,2);
			lcd.print("Over Current");
      lcd.setCursor(0,3);
      lcd.print(current);
      lcd.print("A");

      //Sending fault to LabVIEW
      Serial.print(voltagelv,1);
      Serial.print('\t');
      Serial.print(currentlv,2);
      Serial.print('\t');
      Serial.print(powerlv,1);
      Serial.print('\t');
      RPMlv =  RPM + currentOffset;
      if (RPM <= minRPM)
      {
        RPMlv = currentOffset;
      }
      Serial.print(RPMlv);
      Serial.println("Motor OFF, Fault Detected:                 Over Current - Current exceeded 2.0A");
    }
  }

  if ((RPM > maxRPM) && (MotorStatus == 1) && (current >= 0.5)) //Over speed protection
  {
    lcd.clear();
    analogWrite(PWMPin,0);
    delay(50);
    digitalWrite(Relay1, LOW);
    delay(250);
    digitalWrite(Relay2, LOW);
    
    voltagelv = voltageOffset;
    currentlv = currentOffset;
    powerlv = powerOffset;
    
    while(1)
    {
      lcd.setCursor(0,0);
      lcd.print("Motor OFF");
      lcd.setCursor(0,1);
			lcd.print("Fault Detected:");
			lcd.setCursor(0,2);
			lcd.print("Over Speed");
      lcd.setCursor(0,3);
      lcd.print("RPM exceeded 5000");

      //Sending fault to LabVIEW
      Serial.print(voltagelv,1);
      Serial.print('\t');
      Serial.print(currentlv,2);
      Serial.print('\t');
      Serial.print(powerlv,1);
      Serial.print('\t');
      RPMlv =  RPM + currentOffset;
      if (RPM <= minRPM)
      {
        RPMlv = currentOffset;
      }
      Serial.print(RPMlv);
      Serial.println("Motor OFF, Fault Detected:                      Over Speed - RPM exceeded 5000");
    }
  }

  unsigned long counttime = millis();
  if (counttime - lastcounttime >= 1000) //Motor stuck protection
  {
    if ((tachocount == 0) && (MotorStatus == 1) && (current >= 0.5))
    {
      lcd.clear();
      analogWrite(PWMPin,0);
      delay(50);
      digitalWrite(Relay1, LOW);
      delay(250);
      digitalWrite(Relay2, LOW);
      
      voltagelv = voltageOffset;
      currentlv = currentOffset;
      powerlv = powerOffset;

      while(1)
      {
        lcd.setCursor(0,0);
        lcd.print("Motor OFF");
        lcd.setCursor(0,1);
			  lcd.print("Fault Detected:");
			  lcd.setCursor(0,2);
			  lcd.print("Motor Shaft Seized/");
        lcd.setCursor(0,3);
        lcd.print("No Power");

        //Sending fault to LabVIEW
        Serial.print(voltagelv,1);
        Serial.print('\t');
        Serial.print(currentlv,2);
        Serial.print('\t');
        Serial.print(powerlv,1);
        Serial.print('\t');
        if (RPM <= minRPM)
        {
          RPMlv = currentOffset;
        }
        Serial.print(RPMlv);
        Serial.println("Motor OFF, Fault Detected:             Motor Shaft Seized or No Power");
      }
    }
    tachocount = 0;
    lastcounttime = millis();
  }
}