// <  This code is to run a linear arry of 4 HC-SR04 ultrasonic sensors 
//    to detect and count human bodies that pass by this linear array
//    The programme will then count the number of budies passig in one 
//    direction and decrement the counter if a detected body passed on the other direction >
//    Averaging of many distance reading is applied depending on the distance and speed of the 
//   passing body.
//    Copyright (C) <2018>  <Wrya Monnet>
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

#include "MegunoLink.h"
Message MyCSVMessage("Data");
#define V_SOUND 349 
#define AVERAGE_SPEED   1.5 
#define TIME_BURST 0.000200    // for the HC SR04 the burst latss 200 microseconds
#define TAN_ANGLE_COVERAGE 0.3249
#define TIME_DELAY 0.000010    // a time delay added between two distance measurements

#define PRE_REPETITIONS  10.0  // the number of default repetitions 
#define LOW_THRESHOLD    10.0   // the lowest distance threshold in cm  
#define FIX_OBJECT_DISTANCE_IN 140.0   // the fixed object distance to the entrey sensor in cm
#define FIX_OBJECT_DISTANCE_OUT 140.0  // the fixed object distance to the outgoing sensor in cm 
#define FIX_OBJECT_DISTANCE 200.0  // the fixed object distance to the outgoing sensor in cm
 

#define PreOut_TRIG_PIN 2  /// D0 these two pins are for outgoing pre
#define PreOut_ECHO_PIN 3    // D1
#define ConfirmOut_TRIG_PIN 4  /// D2 these two pins are for outgoing confirmation
#define ConfirmOut_ECHO_PIN 5  // D3

#define PreIn_TRIG_PIN 6   /// D4 these two pins are for incoming Pre
#define PreIn_ECHO_PIN 7   // D5
#define ConfirmIn_TRIG_PIN 8   /// these two pins are for incoming Confirmation
#define ConfirmIn_ECHO_PIN 9

typedef enum{
    RESET = 1,
    START_IN = 2,
    END_IN = 3, 
    IN = 4,
    START_OUT = 5,
    END_OUT = 6,
    OUT = 7
} state_e;

typedef enum{
    Idle = 1,
    PreInObject = 2,
    Entering = 3,
    Entered = 4,
    EnterOK = 5,
    PreOutObject = 6,
    Exiting = 7,
    Exited = 8,
    ExitOK = 9  
} event_e;

// to define the sensors identity (4 sensors are used)
typedef enum{
    PRE_ENTRY = 1,
    ENTRY = 2,
    PRE_EXIT = 3,
    EXIT = 4
  }sensor_i;

static event_e current_event = Idle;
static int Number_of_Entries = 0;
static int Number_of_Exits = 0;


// the functions used to measure once the distance for each sensor 
float SingleDistanceMeasure_PreIn()   // the pre input detector sensor
{  unsigned long timeperiod;
  digitalWrite(PreIn_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(PreIn_TRIG_PIN, LOW); 
  timeperiod = pulseIn(PreIn_ECHO_PIN, HIGH); 
  
  return microsecondsToCentimeters(timeperiod);
  
  }

float SingleDistanceMeasure_PreOut()  // the pre out detector sensor
{ 
  unsigned long timeperiod;
  digitalWrite(PreOut_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(PreOut_TRIG_PIN, LOW); 
  timeperiod = pulseIn(PreOut_ECHO_PIN, HIGH); 
  
  return microsecondsToCentimeters(timeperiod);
  }

float SingleDistanceMeasure_In()   // the Entrey sensor
{
  unsigned long timeperiod;
  digitalWrite(ConfirmIn_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ConfirmIn_TRIG_PIN, LOW); 
  timeperiod = pulseIn(ConfirmIn_ECHO_PIN, HIGH); 
  
  return microsecondsToCentimeters(timeperiod);
  }

  float SingleDistanceMeasure_Out()    // the Exiting sensor
{
  unsigned long timeperiod;
  digitalWrite(ConfirmOut_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ConfirmOut_TRIG_PIN, LOW); 
  timeperiod = pulseIn(ConfirmOut_ECHO_PIN, HIGH); 
  
  return microsecondsToCentimeters(timeperiod);
  }

float microsecondsToCentimeters(long microseconds)   // to tranform microseconds to distance in cm
{  
    return (float)(microseconds / 29.0 / 2.0);
}

// this function detects if a moving object is passing by the sensor in the range between fixed object and the sensor
 bool Detect(sensor_i sensor,int repetitions, float LowThreshold, float fixedObstacleDistance)
 {
  float Measure;
  float MeasureSum = 0.0;
  int count = 0;
  //Serial.println(repetitions); 
 // Serial.print("the sensor in detect is:");
 // Serial.println(sensor);
   
    for (int i=0; i< repetitions; i++)
    {
      if(sensor == PRE_ENTRY)
         {Measure =  SingleDistanceMeasure_PreIn();}
      else if (sensor == ENTRY)
          {Measure =  SingleDistanceMeasure_In();}
      else if (sensor == PRE_EXIT)
          {Measure =  SingleDistanceMeasure_PreOut();}
      else if (sensor == EXIT)
          {Measure =  SingleDistanceMeasure_Out();}          
     
      if(Measure < fixedObstacleDistance &&  Measure > LowThreshold)
        {
          MeasureSum = MeasureSum + Measure;
          count = count +1;
          }
      delayMicroseconds(10);    
     }
     if(count > 0) 
     {MeasureSum = MeasureSum/float(count);}
     else
     {MeasureSum = 0.0;}
     
     if(MeasureSum < fixedObstacleDistance &&  MeasureSum > LowThreshold)
     {return true;}
     else 
     {return false;}
  
  }  

//function to write the current event encapsulating data 
void write_event(event_e event)
{
  current_event = event;
  
  }
// function to read the current event encapsulating data
event_e read_event()
{
  return current_event;
  }


int repetetions(float distance)
{
  int repetetions;
  repetetions = round((2* TAN_ANGLE_COVERAGE*distance*0.01 * V_SOUND)/((2*distance*0.01 + (2*TIME_BURST+TIME_DELAY)*V_SOUND)* AVERAGE_SPEED)); 
 // Serial.print("the repetition is:");
 // Serial.println(repetetions);
  return repetetions;
  }

void reset()
{
  bool detect_pre_in = false;
  bool detect_pre_out = false;

 
      detect_pre_in = Detect(PRE_ENTRY, PRE_REPETITIONS, LOW_THRESHOLD, FIX_OBJECT_DISTANCE_IN);
      delayMicroseconds(10);
      detect_pre_out = Detect(PRE_EXIT, PRE_REPETITIONS, LOW_THRESHOLD, FIX_OBJECT_DISTANCE_OUT);
    
      if(!(detect_pre_in ^ detect_pre_out))
        {write_event(Idle);
     //   Serial.println("Nodetect");
          }
      else if (detect_pre_in)
         {write_event(PreInObject);
         Serial.println("Indetect");}
      else if (detect_pre_out)   
         {write_event(PreOutObject);
          Serial.println("outdetect");  }  
     
}

void start_in()
{
    bool Detect_in;
    float Measure;
    int Repetetions;
    long timer;
    timer = millis();
     do{
      Measure =  SingleDistanceMeasure_In();
    }  while(!(Measure > LOW_THRESHOLD && Measure < FIX_OBJECT_DISTANCE_IN) && (millis()-timer) < 1000 );
    Repetetions = repetetions(Measure);       // find the number of possible repetitions
    if(Repetetions > 0) 
      {Detect_in = Detect(ENTRY, Repetetions, LOW_THRESHOLD, FIX_OBJECT_DISTANCE);}
    else 
       {Detect_in = false;}     
        
    if(Detect_in)
      {write_event(Entering);}
    else
      {write_event(Idle);}  
  
  }

void end_in()
{  
    bool Detect_end_in;
    float Measure;
    int Repetetions;
    long timer;
    timer = millis();
    do{
      Measure =  SingleDistanceMeasure_Out();
    } while(!(Measure > LOW_THRESHOLD && Measure < FIX_OBJECT_DISTANCE) && (millis()-timer) < 1000);
    Repetetions = repetetions(Measure);       // find the number of possible repetitions

     if(Repetetions > 0) 
      {Detect_end_in = Detect(EXIT, Repetetions, LOW_THRESHOLD, FIX_OBJECT_DISTANCE);}
    else 
       {Detect_end_in = false;}    
       
  if(Detect_end_in)
      {write_event(Entered);}
    else
      {write_event(Idle);}  
}  

void in()
{
   bool Intrue;
   float Measure;
   int Repetetions;
   int Total;
   long timer;
   timer = millis();
    do{
      Measure =  SingleDistanceMeasure_PreOut();
    } while(!(Measure > LOW_THRESHOLD && Measure < FIX_OBJECT_DISTANCE_OUT) && (millis()-timer) < 1000);
    Repetetions = repetetions(Measure);       // find the number of possible repetitions
   
    if(Repetetions > 0) 
      {Intrue = Detect(PRE_EXIT, Repetetions, LOW_THRESHOLD, FIX_OBJECT_DISTANCE_OUT);}
    else 
       {Intrue = false;}    
    
    if(Intrue)
      {write_event(EnterOK);
      Number_of_Entries++;
      Total = Number_of_Entries - Number_of_Exits;
      MyCSVMessage.Begin();
      Serial.print("Entering");
      Serial.print(",");
      Serial.print(Number_of_Entries);
      Serial.print(",");
      Serial.print(Number_of_Exits);
      Serial.print(",");
      Serial.print(Total);
      MyCSVMessage.End();
      delay(600);}
    else
      {write_event(Idle);}  
      
    
  return 0;
  }

  
void start_out()
{ 
    bool Detect_out;
    float Measure;
    int Repetetions;
    long timer;
    timer = millis();
     do{
      Measure =  SingleDistanceMeasure_Out();
    }  while(!(Measure > LOW_THRESHOLD && Measure < FIX_OBJECT_DISTANCE_OUT) && (millis()-timer) < 1000);
    Repetetions = repetetions(Measure);       // find the number of possible repetitions

    if(Repetetions > 0)  
        {Detect_out = Detect(EXIT, Repetetions, LOW_THRESHOLD, FIX_OBJECT_DISTANCE_OUT);}
    else 
        {Detect_out = false;}    
    if(Detect_out)
      {write_event(Exiting);}
    else
      {write_event(Idle);}  
    
  return 0;
  }

void end_out()
{
    bool Detect_end_out;
    float Measure;
    int Repetetions;
    long timer;
    timer = millis();
    do{
      Measure =  SingleDistanceMeasure_In();
    } while(!(Measure > LOW_THRESHOLD && Measure < FIX_OBJECT_DISTANCE) && (millis()-timer) < 1000);
    Repetetions = repetetions(Measure);       // find the number of possible repetitions
    if(Repetetions > 0) 
        {Detect_end_out = Detect(ENTRY, Repetetions, LOW_THRESHOLD, FIX_OBJECT_DISTANCE);}
    else
        {Detect_end_out = false;}
        
  if(Detect_end_out)
      {write_event(Exited);}
    else
      {write_event(Idle);}  
  return 0;
  }

void out()
{
    bool Detect_out;
    float Measure;
    int Repetetions;
    int Total;
    long timer;
    timer = millis();
    do{
      Measure =  SingleDistanceMeasure_PreIn();
    } while(!(Measure > LOW_THRESHOLD && Measure < FIX_OBJECT_DISTANCE_IN) && (millis()-timer) < 1000);
    Repetetions = repetetions(Measure);       // find the number of possible repetitions
    
    if(Repetetions > 0)
      {Detect_out = Detect(PRE_ENTRY, Repetetions, LOW_THRESHOLD, FIX_OBJECT_DISTANCE_IN);}
    else
       {Detect_out = false;}
       
    if(Detect_out)
    {
     write_event(ExitOK); 
    Number_of_Exits++;
    Total = Number_of_Entries - Number_of_Exits;
    MyCSVMessage.Begin();
    Serial.print("Exiting");
    Serial.print(",");
    Serial.print(Number_of_Entries);
    Serial.print(",");
    Serial.print(Number_of_Exits);
    Serial.print(",");
    Serial.print(Total);
    MyCSVMessage.End();
    delay(600);}
    
    else
       {write_event(Idle);}
  return 0;
  }
      
void setup()
{
  Serial.begin(115200); //serial port communication
  Serial.println("CSV Message Monitor");
  Serial.print("Data format: ");
  
  MyCSVMessage.Begin();
  Serial.print("Status");
  Serial.print(",");
  Serial.print("Entries");
  Serial.print(",");
  Serial.print("Exits");
  Serial.print(",");
  Serial.print("Total");
  MyCSVMessage.End();
  
  
  // initialising the pins
  pinMode(PreOut_TRIG_PIN, OUTPUT); // defining pinmode for trig of the 1st outgoing detector
  pinMode(PreOut_ECHO_PIN, INPUT);  // defining pinmode for echo pin
  
  pinMode(ConfirmOut_TRIG_PIN, OUTPUT); // defining pinmode for trig of the 2nd outgoing detector  
  pinMode(ConfirmOut_ECHO_PIN , INPUT);  // defining pinmode for echo pin
  pinMode(PreIn_TRIG_PIN, OUTPUT); // defining pinmode for trig   of the 1st incoming detector
  pinMode(PreIn_ECHO_PIN, INPUT);  // defining pinmode for echo pin
  pinMode(ConfirmIn_TRIG_PIN, OUTPUT); // defining pinmode for trig  of the 2nd incoming detector
  pinMode(ConfirmIn_ECHO_PIN, INPUT);  // defining pinmode for echo pin
  digitalWrite(PreIn_TRIG_PIN, LOW); // initialise all triggers to low
  digitalWrite(ConfirmOut_TRIG_PIN, LOW);
  digitalWrite(PreOut_TRIG_PIN, LOW);
  digitalWrite(ConfirmIn_ECHO_PIN, LOW);
   
}

void loop()
{
  // initialising state and event variables
   event_e Event_FSM = Idle;
   state_e Current_State = RESET;     // current state variable
   state_e Next_State = RESET;        // next state variable
   Serial.println("start"); 
   int State_Count = 0;
   while(1)
   {
    
       switch(Current_State)
       {
          case RESET:
             reset();
             Event_FSM = read_event();
          //   Serial.print("the event is"); 
          //   Serial.println(Event_FSM); 
             if(Event_FSM == Idle)
               {Next_State = RESET;}
             else if(Event_FSM == PreInObject)
               {Next_State = START_IN;}
             else if(Event_FSM == PreOutObject) 
               {Next_State = START_OUT;}
              // Serial.print("the state is:"); 
             //Serial.println(Next_State); 
             State_Count = 0;  
            break;
    
          case START_IN:
             start_in();
          //   Serial.print("the event is"); 
          //   Serial.println(Event_FSM);
             Event_FSM = read_event();
             if(Event_FSM == Idle)
               {Next_State = START_IN;
               State_Count++;}             // a counter watch-dog to bring back the state to reset if stucked in START_IN 
              else if(Event_FSM == Entering)
               {Next_State = END_IN;
               State_Count = 0;}              // reset the counter watch dog

              if(State_Count > 5)           // staying more than 5 times in a state is a return to RESET
                 {Next_State = RESET;}
               Serial.print("the state is:"); 
               Serial.println(Next_State); 
            break;  
    
          case END_IN:
              end_in();
              Event_FSM = read_event();
              if(Event_FSM == Idle)
                {Next_State = END_IN;
                 State_Count++;}
              else if(Event_FSM == Entered)  
                {Next_State = IN;
                 State_Count = 0;}
                 
              if(State_Count > 5)
                 {Next_State = RESET;}   
                Serial.print("the state is:"); 
                Serial.println(Next_State); 
            break;
            
          case IN:
              in();
              Event_FSM = read_event();
              if(Event_FSM == Idle)
                {Next_State = IN;
                State_Count++;}
              else if(Event_FSM == EnterOK)  
                {Next_State = RESET;
                State_Count = 0;}
                
              if(State_Count > 5)
                 {Next_State = RESET;}  
                Serial.print("the state is:"); 
                Serial.println(Next_State); 
             break;
             
          case START_OUT:
              start_out();
              Event_FSM = read_event();
              if(Event_FSM == Idle)
                {Next_State = START_OUT;
                 State_Count++;}
              else if(Event_FSM == Exiting)
                {Next_State = END_OUT;
                State_Count = 0;}
                
              if(State_Count > 5)
                 {Next_State = RESET;}    
                Serial.print("the state is:"); 
                Serial.println(Next_State); 
            break; 
             
          case END_OUT:
              end_out();
              Event_FSM = read_event();
              if(Event_FSM == Idle)
                {Next_State = END_OUT;
                State_Count++;}
              else if(Event_FSM == Exited)
                {Next_State = OUT;
                State_Count = 0;}
                
             if(State_Count > 5)
                 {Next_State = RESET;}     
                Serial.print("the state is:"); 
                Serial.println(Next_State); 
            break;
    
          case OUT:
            out();
            Event_FSM = read_event();
              if(Event_FSM == Idle)
                {Next_State = OUT;
                 State_Count++;}
              else if(Event_FSM == ExitOK)
                {Next_State = RESET;
                State_Count = 0;}

              if(State_Count > 5)
                 {Next_State = RESET;}    
                Serial.print("the state is:"); 
                Serial.println(Next_State); 
            break;

         default:
             break;    
        }
       Current_State = Next_State;
       delay(10);
   }
   
}
