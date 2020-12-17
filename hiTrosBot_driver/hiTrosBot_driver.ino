/*
 * Multimodal interaction and robotic active perception (inte-R-action)lab
 * 
 * Author: Uriel Martinez-Hernandez
 * Date: 15/12/2020
 * 
 * Description: Driver for controlling multiple stepper motors through C++
 * 
 * Dependencies: AccelStepper.h
 * 
 */

#include <AccelStepper.h>

#define MAX_SIZE_COMMAND 20
#define MAX_NUM_PARAMETERS 20
#define MAX_SPEED 7000

// x axis
AccelStepper stepperX(AccelStepper::DRIVER, 11, 10); //step, direction
// y axis
AccelStepper stepperY(AccelStepper::DRIVER, 9, 8); //step, direction
// z axis
AccelStepper stepperZ(AccelStepper::DRIVER, 7, 6); //step, direction
// t axis
AccelStepper stepperP(AccelStepper::DRIVER, 5, 4); //step, direction


boolean isXInitPositionReached = false;
boolean isYInitPositionReached = false;
boolean isZInitPositionReached = false;

int limitSwitchStateX = HIGH;             // the current reading from the input pin
int limitSwitchStateY = HIGH;             // the current reading from the input pin
int limitSwitchStateZ = HIGH;             // the current reading from the input pin

int lastLimitSwitchState = LOW;   // the previous reading from the input pin
int lastLimitSwitchStateX = LOW;   // the previous reading from the input pin
int lastLimitSwitchStateY = LOW;   // the previous reading from the input pin
int lastLimitSwitchStateZ = LOW;   // the previous reading from the input pin

bool limitSwitchState = false;

int dir = 0;
int count = 0;
char command[MAX_SIZE_COMMAND];
int xPosition = 0;
int yPosition = 0;
int xySpeed = 0;
char commands_char[MAX_NUM_PARAMETERS][MAX_SIZE_COMMAND];
int ncommand = 0;
char current_char;
int operationStatus = 0;
bool commandStatus = false;
int calibrationStatus = 0;

boolean inPositionX = false;
boolean inPositionY = false;   
boolean inPositionZ = false;
boolean inPositionT = false; 
boolean inHomeX = false;
boolean inHomeY = false;
boolean inHomeZ = false;
boolean inHomeT = false;

// default speed for each motor
int speedValueMotorX = 50;
int speedValueMotorY = 50;
int speedValueMotorZ = 50;
int speedValueMotorT = 50;

const int limitSwitchX = 22;    // pin number of the limit switch for motor X
const int limitSwitchY = 23;    // pin number of the limit switch for motor Y
const int limitSwitchZ = 24;    // pin number of the limit switch for motor Z
const int limitSwitchT = 25;    // pin number of the limit switch for motor Z

// debouncing values of limit swithces for calibration process
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 5;    // the debounce time; increase if the output flickers

void setup()
{  
    stepperX.move(0);
    stepperY.move(0);
    stepperZ.move(0);
    stepperP.move(0);

    stepperX.setMaxSpeed(MAX_SPEED);
    stepperY.setMaxSpeed(MAX_SPEED);
    stepperZ.setMaxSpeed(MAX_SPEED);
    stepperP.setMaxSpeed(MAX_SPEED);
  
    Serial.begin(9600);
}


void loop()
{

    if( Serial.available() > 0 )
    {
      
        for( int i = 0; i < MAX_NUM_PARAMETERS; i++ )
        {
            for( int j = 0; j < MAX_SIZE_COMMAND; j++ )
                commands_char[i][j] = '\0';
        }

        operationStatus = 0;
        count = 0;
        ncommand = 0;

        do
        {
            current_char = Serial.read();
            
            delay(3);

            if( current_char != ' ' )
            {
                commands_char[ncommand][count] = current_char;
                count++;
            }
            else
            {
                commands_char[ncommand][count] = '\0';
                count = 0;
                ncommand++;                
            }
            
        }while( current_char != '\r' );

        commandStatus = commandList(commands_char[0]);
        replyAcknowledge(commandStatus);

        if( commandStatus == true )
            replyAcknowledge(executeCommand(commands_char));

        Serial.flush();
    }
}    

/* Function for execution of commands */
bool executeCommand(char cmdReceived[][MAX_SIZE_COMMAND])
{
    int step_size_int[20];
    int abs_step_size_int[20];
    int speed_int[20];
    int xMotorPos = 0;
    int yMotorPos = 0;
    int zMotorPos = 0;
    int tMotorPos = 0;

    /* Calibration of X axis */
    if( !strcmp(cmdReceived[0],"@CALX") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            step_size_int[0] = atoi(cmdReceived[1]);
             
            stepperX.move(step_size_int[0]);
            stepperX.setSpeed(MAX_SPEED);
      
            while( stepperX.distanceToGo() != 0 )
                stepperX.runSpeedToPosition();
                
            stepperX.move(0);
            stepperX.setSpeed(MAX_SPEED);
            stepperX.runSpeedToPosition();                
  
            return true;
        }
        else       
            return false;
    }
    /* Calibration of X axis with stop condition from limit switch*/
    if( !strcmp(cmdReceived[0],"@CALXS") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            step_size_int[0] = atoi(cmdReceived[1]);
             
            stepperX.move(step_size_int[0]);
            stepperX.setSpeed(MAX_SPEED);
      
            while( stepperX.distanceToGo() != 0 )
                stepperX.runSpeedToPosition();
                
            stepperX.move(0);
            stepperX.setSpeed(MAX_SPEED);
            stepperX.runSpeedToPosition();                
  
            return true;
        }
        else       
            return false;
    }
    /* Calibration of Y axis */
    else if( !strcmp(cmdReceived[0],"@CALY") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            step_size_int[0] = atoi(cmdReceived[1]);
              
            stepperY.move(step_size_int[0]);
            stepperY.setSpeed(MAX_SPEED);
      
            while( stepperY.distanceToGo() != 0 )
                stepperY.runSpeedToPosition();
                
            stepperY.move(0);
            stepperY.setSpeed(MAX_SPEED);
            stepperY.runSpeedToPosition();                
            
            return true;            
        }
        else
            return false;
    }
    /* Calibration of Z axis */
    else if( !strcmp(cmdReceived[0],"@CALZ") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            step_size_int[0] = atoi(cmdReceived[1]);
              
            stepperZ.move(step_size_int[0]);
            stepperZ.setSpeed(MAX_SPEED);
      
            while( stepperZ.distanceToGo() != 0 )
                stepperZ.runSpeedToPosition();
                
            stepperZ.move(0);
            stepperZ.setSpeed(MAX_SPEED);
            stepperZ.runSpeedToPosition();                

            return true;
        }
        else
            return false;
    }
    /* Calibration of T axis */
    else if( !strcmp(cmdReceived[0],"@CALT") )
    {
        if( strcmp(cmdReceived[1]," ") )
        {
            step_size_int[0] = atoi(cmdReceived[1]);
              
            stepperP.move(step_size_int[0]);
            stepperP.setSpeed(MAX_SPEED);
      
            while( stepperP.distanceToGo() != 0 )
                stepperP.runSpeedToPosition();
                
            stepperP.move(0);
            stepperP.setSpeed(MAX_SPEED);
            stepperP.runSpeedToPosition();                

            return true;
        }
        else
            return false;
    }
    /* Calibration */
    else if( !strcmp(cmdReceived[0],"@CALNOW\r") )
    {
        stepperX.move(50);
        stepperX.setSpeed(50);
        while( stepperX.distanceToGo() != 0 )
            stepperX.run();
    
        stepperY.move(50);
        stepperY.setSpeed(50);
        while( stepperY.distanceToGo() != 0 )
            stepperY.run();
    
        stepperZ.move(50);
        stepperZ.setSpeed(50);
        while( stepperZ.distanceToGo() != 0 )
            stepperZ.run();
    
        //stepperP.move(100);
        //stepperP.setSpeed(100);
        //while( stepperP.distanceToGo() != 0 )
          //  stepperP.run();
            
        stepperX.setCurrentPosition(0);        
        stepperY.setCurrentPosition(0);        
        stepperZ.setCurrentPosition(0);
        stepperP.setCurrentPosition(0);
            
        calibrationStatus = 1;

        return true;
    }
    /* Move all axes to home position */
    else if( !strcmp(cmdReceived[0], "@MOVHOME\r") )
    {
        if( calibrationStatus == 1 )
        {
            inHomeX = false;
            inHomeY = false;
            inHomeZ = false;
            inHomeT = false;
    
            speed_int[0] = MAX_SPEED;
            speed_int[1] = MAX_SPEED;
            speed_int[2] = MAX_SPEED;
            speed_int[3] = MAX_SPEED;
            
            
            if( 0 < stepperX.currentPosition() )
                speed_int[0] = -1*speed_int[0];
            if( 0 < stepperY.currentPosition() )
                speed_int[1] = -1*speed_int[1];
            if( 0 < stepperZ.currentPosition() )
                speed_int[2] = -1*speed_int[2];
            if( 0 < stepperP.currentPosition() )
                speed_int[3] = -1*speed_int[3];

            stepperX.moveTo(0);
            stepperX.setSpeed(speed_int[0]);
            stepperY.moveTo(0);
            stepperY.setSpeed(speed_int[1]);
            stepperZ.moveTo(0);
            stepperZ.setSpeed(speed_int[2]);
            stepperP.moveTo(0);
            stepperP.setSpeed(speed_int[3]);
            
            do
            {                          
                if( stepperX.distanceToGo() == 0 )
                {
                    stepperX.move(0);
                    stepperX.setSpeed(0);
                    inHomeX = true;
                }
      
                if( stepperY.distanceToGo() == 0 )
                {
                    stepperY.move(0);
                    stepperY.setSpeed(0);
                    inHomeY = true;
                }
  
                if( stepperZ.distanceToGo() == 0 )
                {
                    stepperZ.move(0);
                    stepperZ.setSpeed(0);
                    inHomeZ = true;
                }
  
                if( stepperP.distanceToGo() == 0 )
                {
                    stepperP.move(0);
                    stepperP.setSpeed(0);
                    inHomeT = true;
                }
  
                stepperX.runSpeedToPosition();
                stepperY.runSpeedToPosition();
                stepperZ.runSpeedToPosition();
                stepperP.runSpeedToPosition();
                  
            }while( ( inHomeX != true ) || ( inHomeY != true ) || ( inHomeZ != true ) || ( inHomeT != true ) );

            return true;
        }
        else
            return false;    
    }
    /* Stop all motors */
    else if( !strcmp(cmdReceived[0],"@STOPALL\r") )
    {
        stepperX.move(0);
        stepperY.move(0);
        stepperZ.move(0);
        stepperP.move(0);
        stepperX.setSpeed(1000);
        stepperY.setSpeed(1000);
        stepperZ.setSpeed(1000);
        stepperP.setSpeed(1000);
    }
    /* Get position from all axis */
    else if( !strcmp(cmdReceived[0],"@GETALLPOS\r") )
    {
        xMotorPos = stepperX.currentPosition();
        yMotorPos = stepperY.currentPosition();
        zMotorPos = stepperZ.currentPosition();
        tMotorPos = stepperP.currentPosition();
        Serial.print("\n");
        Serial.print(xMotorPos);
        Serial.print(" ");
        Serial.print(yMotorPos);
        Serial.print(" ");
        Serial.print(zMotorPos);
        Serial.print(" ");
        Serial.print(tMotorPos);
        Serial.print("\n");

    }
    /* Relative movement of X axis */
    else if( !strcmp(cmdReceived[0],"@MOVRX") )
    {
        if( calibrationStatus == 1 )
        {
          if( strcmp(cmdReceived[1]," ") && strcmp(cmdReceived[2]," ") )
          {
              step_size_int[0] = atoi(cmdReceived[1]);           
              speed_int[0] = atoi(cmdReceived[2]);
                              
              if( speed_int[0] > MAX_SPEED )
                  speed_int[0] = MAX_SPEED;
                  
              stepperX.move(step_size_int[0]);
              stepperX.setSpeed(speed_int[0]);
        
              while( stepperX.distanceToGo() != 0 )
                  stepperX.runSpeedToPosition();
                  
              stepperX.move(0);
              stepperX.setSpeed(speed_int[0]);
              stepperX.runSpeedToPosition();                
    
              return true;
          }
          else
              return false;       
        }
        else       
            return false;
    }
    /* Relative movement of Y axis */
    else if( !strcmp(cmdReceived[0],"@MOVRY") )
    {
        if( calibrationStatus == 1 )
        {
          if( strcmp(cmdReceived[1]," ") && strcmp(cmdReceived[2]," ") )
          {
              step_size_int[0] = atoi(cmdReceived[1]);
              speed_int[0] = atoi(cmdReceived[2]);
               
              if( speed_int[0] > MAX_SPEED )
                  speed_int[0] = MAX_SPEED;

              stepperY.move(step_size_int[0]);
              stepperY.setSpeed(speed_int[0]);
        
              while( stepperY.distanceToGo() != 0 )
                  stepperY.runSpeedToPosition();
                  
              stepperY.move(0);
              stepperY.setSpeed(speed_int[0]);
              stepperY.runSpeedToPosition();                
    
              return true;
          }
          else
              return false;       
        }
        else       
            return false;
    }
    /* Relative movement of Z axis */
    else if( !strcmp(cmdReceived[0],"@MOVRZ") )
    {
        if( calibrationStatus == 1 )
        {
          if( strcmp(cmdReceived[1]," ") && strcmp(cmdReceived[2]," ") )
          {
              step_size_int[0] = atoi(cmdReceived[1]);
              speed_int[0] = atoi(cmdReceived[2]);
               
              if( speed_int[0] > MAX_SPEED )
                  speed_int[0] = MAX_SPEED;

              stepperZ.move(step_size_int[0]);
              stepperZ.setSpeed(speed_int[0]);
        
              while( stepperZ.distanceToGo() != 0 )
                  stepperZ.runSpeedToPosition();
                  
              stepperZ.move(0);
              stepperZ.setSpeed(speed_int[0]);
              stepperZ.runSpeedToPosition();                
    
              return true;
          }
          else
              return false;       
        }
        else       
            return false;
    }
    /* Relative movement of Theta axis */
    else if( !strcmp(cmdReceived[0],"@MOVRT") )
    {
        if( calibrationStatus == 1 )
        {
          if( strcmp(cmdReceived[1]," ") && strcmp(cmdReceived[2]," ") )
          {
              step_size_int[0] = atoi(cmdReceived[1]);
              speed_int[0] = atoi(cmdReceived[2]);
               
              if( speed_int[0] > MAX_SPEED )
                  speed_int[0] = MAX_SPEED;

              stepperP.move(step_size_int[0]);
              stepperP.setSpeed(speed_int[0]);
        
              while( stepperP.distanceToGo() != 0 )
                  stepperP.runSpeedToPosition();
                  
              stepperP.move(0);
              stepperP.setSpeed(speed_int[0]);
              stepperP.runSpeedToPosition();                
    
              return true;
          }
          else
              return false;       
        }
        else       
            return false;
    }
    /* Absolute movement of X axis */
    else if( !strcmp(cmdReceived[0],"@MOVAX") )
    {
        if( calibrationStatus == 1 )
        {
          if( strcmp(cmdReceived[1]," ") && strcmp(cmdReceived[2]," ") )
          {
              step_size_int[0] = atoi(cmdReceived[1]);
              speed_int[0] = atoi(cmdReceived[2]);
              
              if( speed_int[0] > MAX_SPEED )
                  speed_int[0] = MAX_SPEED;

              stepperX.moveTo(step_size_int[0]);
              stepperX.setSpeed(speed_int[0]);
        
              while( stepperX.distanceToGo() != 0 )
                  stepperX.runSpeedToPosition();
                  
              stepperX.move(0);
              stepperX.setSpeed(speed_int[0]);
              stepperX.runSpeedToPosition();                
    
              return true;
          }
          else
              return false;       
        }
        else       
            return false;
    }
    /* Absolute movement of Y axis */
    else if( !strcmp(cmdReceived[0],"@MOVAY") )
    {
        if( calibrationStatus == 1 )
        {
          if( strcmp(cmdReceived[1]," ") && strcmp(cmdReceived[2]," ") )
          {
              step_size_int[0] = atoi(cmdReceived[1]);
              speed_int[0] = atoi(cmdReceived[2]);
              
              if( speed_int[0] > MAX_SPEED )
                  speed_int[0] = MAX_SPEED;

              stepperY.moveTo(step_size_int[0]);
              stepperY.setSpeed(speed_int[0]);
        
              while( stepperY.distanceToGo() != 0 )
                  stepperY.runSpeedToPosition();
                  
              stepperY.move(0);
              stepperY.setSpeed(speed_int[0]);
              stepperY.runSpeedToPosition();                
    
              return true;
          }
          else
              return false;       
        }
        else       
            return false;
    }
    /* Absolute movement of Z axis */
    else if( !strcmp(cmdReceived[0],"@MOVAZ") )
    {
        if( calibrationStatus == 1 )
        {
          if( strcmp(cmdReceived[1]," ") && strcmp(cmdReceived[2]," ") )
          {
              step_size_int[0] = atoi(cmdReceived[1]);
              speed_int[0] = atoi(cmdReceived[2]);
              
              if( speed_int[0] > MAX_SPEED )
                  speed_int[0] = MAX_SPEED;

              stepperZ.moveTo(step_size_int[0]);
              stepperZ.setSpeed(speed_int[0]);
        
              while( stepperZ.distanceToGo() != 0 )
                  stepperZ.runSpeedToPosition();
                  
              stepperZ.move(0);
              stepperZ.setSpeed(speed_int[0]);
              stepperZ.runSpeedToPosition();                
    
              return true;
          }
          else
              return false;       
        }
        else       
            return false;
    }
    /* Absolute movement of Theta axis */
    else if( !strcmp(cmdReceived[0],"@MOVAT") )
    {
        if( calibrationStatus == 1 )
        {
          if( strcmp(cmdReceived[1]," ") && strcmp(cmdReceived[2]," ") )
          {
              step_size_int[0] = atoi(cmdReceived[1]);
              speed_int[0] = atoi(cmdReceived[2]);
              
              if( speed_int[0] > MAX_SPEED )
                  speed_int[0] = MAX_SPEED;

              stepperP.moveTo(step_size_int[0]);
              stepperP.setSpeed(speed_int[0]);
        
              while( stepperP.distanceToGo() != 0 )
                  stepperP.runSpeedToPosition();
                  
              stepperP.move(0);
              stepperP.setSpeed(speed_int[0]);
              stepperP.runSpeedToPosition();                
    
              return true;
          }
          else
              return false;       
        }
        else       
            return false;
    }
    /* Relative movement of all axes */
    else if( !strcmp(cmdReceived[0],"@MOVRALL") )
    {
        if( calibrationStatus == 1 )
        {
          if( strcmp(cmdReceived[1]," ") && strcmp(cmdReceived[2]," ") && strcmp(cmdReceived[3]," ") && strcmp(cmdReceived[4]," ") && 
              strcmp(cmdReceived[5]," ") && strcmp(cmdReceived[6]," ") && strcmp(cmdReceived[7]," ") && strcmp(cmdReceived[8]," "))
          {
              inPositionX = false;
              inPositionY = false;   
              inPositionZ = false;
              inPositionT = false;   

              step_size_int[0] = atoi(cmdReceived[1]);
              step_size_int[1] = atoi(cmdReceived[2]);
              step_size_int[2] = atoi(cmdReceived[3]);
              step_size_int[3] = atoi(cmdReceived[4]);
              speed_int[0] = atoi(cmdReceived[5]);
              speed_int[1] = atoi(cmdReceived[6]);
              speed_int[2] = atoi(cmdReceived[7]);
              speed_int[3] = atoi(cmdReceived[8]);
               
              if( speed_int[0] > MAX_SPEED )
                  speed_int[0] = MAX_SPEED;

              if( speed_int[1] > MAX_SPEED )
                  speed_int[1] = MAX_SPEED;

              if( speed_int[2] > MAX_SPEED )
                  speed_int[2] = MAX_SPEED;

              if( speed_int[3] > MAX_SPEED )
                  speed_int[3] = MAX_SPEED;

              if( step_size_int[0] < stepperX.currentPosition() )
                speed_int[0] = -1*speed_int[0];
              if( step_size_int[1] < stepperY.currentPosition() )
                speed_int[1] = -1*speed_int[1];
              if( step_size_int[2] < stepperZ.currentPosition() )
                speed_int[2] = -1*speed_int[2];
              if( step_size_int[3] < stepperP.currentPosition() )
                speed_int[3] = -1*speed_int[3];

              stepperX.move(step_size_int[0]);
              stepperX.setSpeed(speed_int[0]);
              stepperY.move(step_size_int[1]);
              stepperY.setSpeed(speed_int[1]);
              stepperZ.move(step_size_int[2]);
              stepperZ.setSpeed(speed_int[2]);
              stepperP.move(step_size_int[3]);
              stepperP.setSpeed(speed_int[3]);
        
              do
              {          
                  if( stepperX.distanceToGo() == 0 )
                  {
                      stepperX.move(0);
                      stepperX.setSpeed(0);
                      inPositionX = true;
                  }
        
                  if( stepperY.distanceToGo() == 0 )
                  {
                      stepperY.move(0);
                      stepperY.setSpeed(0);
                      inPositionY = true;
                  }
    
                  if( stepperZ.distanceToGo() == 0 )
                  {
                      stepperZ.move(0);
                      stepperZ.setSpeed(0);
                      inPositionZ = true;
                  }
    
                  if( stepperP.distanceToGo() == 0 )
                  {
                      stepperP.move(0);
                      stepperP.setSpeed(0);
                      inPositionT = true;
                  }
    
                  stepperX.runSpeedToPosition();
                  stepperY.runSpeedToPosition();
                  stepperZ.runSpeedToPosition();
                  stepperP.runSpeedToPosition();
                    
              }while( ( inPositionX != true ) || ( inPositionY != true ) || ( inPositionZ != true ) || ( inPositionT != true ) );
    
              return true;
          }
          else
              return false;       
        }
        else       
            return false;
    }
    /* Absolute movement of all axes */
    else if( !strcmp(cmdReceived[0],"@MOVAALL") )
    {
        if( calibrationStatus == 1 )
        {
          if( strcmp(cmdReceived[1]," ") && strcmp(cmdReceived[2]," ") && strcmp(cmdReceived[3]," ") && strcmp(cmdReceived[4]," ") && 
              strcmp(cmdReceived[5]," ") && strcmp(cmdReceived[6]," ") && strcmp(cmdReceived[7]," ") && strcmp(cmdReceived[8]," "))
          {
              inPositionX = false;
              inPositionY = false;   
              inPositionZ = false;
              inPositionT = false;   

              step_size_int[0] = atoi(cmdReceived[1]);
              step_size_int[1] = atoi(cmdReceived[2]);
              step_size_int[2] = atoi(cmdReceived[3]);
              step_size_int[3] = atoi(cmdReceived[4]);
              speed_int[0] = atoi(cmdReceived[5]);
              speed_int[1] = atoi(cmdReceived[6]);
              speed_int[2] = atoi(cmdReceived[7]);
              speed_int[3] = atoi(cmdReceived[8]);
               
              if( speed_int[0] > MAX_SPEED )
                  speed_int[0] = MAX_SPEED;

              if( speed_int[1] > MAX_SPEED )
                  speed_int[1] = MAX_SPEED;

              if( speed_int[2] > MAX_SPEED )
                  speed_int[2] = MAX_SPEED;

              if( speed_int[3] > MAX_SPEED )
                  speed_int[3] = MAX_SPEED;

              if( step_size_int[0] < stepperX.currentPosition() )
                speed_int[0] = -1*speed_int[0];
              if( step_size_int[1] < stepperY.currentPosition() )
                speed_int[1] = -1*speed_int[1];
              if( step_size_int[2] < stepperZ.currentPosition() )
                speed_int[2] = -1*speed_int[2];
              if( step_size_int[3] < stepperP.currentPosition() )
                speed_int[3] = -1*speed_int[3];
               
              stepperX.moveTo(step_size_int[0]);
              stepperX.setSpeed(speed_int[0]);
              stepperY.moveTo(step_size_int[1]);
              stepperY.setSpeed(speed_int[1]);
              stepperZ.moveTo(step_size_int[2]);
              stepperZ.setSpeed(speed_int[2]);
              stepperP.moveTo(step_size_int[3]);
              stepperP.setSpeed(speed_int[3]);
        
              do
              {          
                  if( stepperX.distanceToGo() == 0 )
                  {
                      stepperX.move(0);
                      stepperX.setSpeed(0);
                      inPositionX = true;
                  }
        
                  if( stepperY.distanceToGo() == 0 )
                  {
                      stepperY.move(0);
                      stepperY.setSpeed(0);
                      inPositionY = true;
                  }
    
                  if( stepperZ.distanceToGo() == 0 )
                  {
                      stepperZ.move(0);
                      stepperZ.setSpeed(0);
                      inPositionZ = true;
                  }
    
                  if( stepperP.distanceToGo() == 0 )
                  {
                      stepperP.move(0);
                      stepperP.setSpeed(0);
                      inPositionT = true;
                  }
    
                  stepperX.runSpeedToPosition();
                  stepperY.runSpeedToPosition();
                  stepperZ.runSpeedToPosition();
                  stepperP.runSpeedToPosition();
                    
              }while( ( inPositionX != true ) || ( inPositionY != true ) || ( inPositionZ != true ) || ( inPositionT != true ) );
    
              return true;
          }
          else
              return false;       
        }
        else       
            return false;
    }
    else if( !strcmp(cmdReceived[0],"@COMSTATUS\r") )
    {
        //if( Serial )
        //    return true;
        //else
        //    return false;
        return true;
    }
    else if( !strcmp(cmdReceived[0],"@CALSTATUS\r") )
    {
        if( calibrationStatus == 1 )
            return true;
        else
            return false;
    }
    else
        return false;
}

/* Send reply ACK/NACK to client */
void replyAcknowledge(bool cmdStatus)
{
//    delay(100);
    if( cmdStatus == true )
        sendACK();
    else
        sendNACK();

    Serial.flush();
}

/* Print ACK message */
void sendACK()
{
    Serial.print("ACK\n");
}

/* Print NACK message */
void sendNACK()
{
    Serial.print("NACK\n");
}

/* Check the command received */
bool commandList(char *cmdReceived)
{
    char *commandArray[] = {"@CALSTART\r","@CALX","@CALXS","@CALY","@CALZ","@CALT","@CALSTATUS\r","@CALNOW\r","@CALEND\r","@MOVHOME\r","@MOVRX","@MOVRY",
                            "@MOVRZ","@MOVRT","@MOVAX","@MOVAY","@MOVAZ","@MOVAT","@MOVRALL","@MOVAALL","@STOPALL\r","@GETALLPOS\r","@COMSTATUS\r"};
    int ncommands = 22;
    
    for( int i = 0; i < ncommands; i++ )
    {
        if( !strcmp(commandArray[i], cmdReceived) )
            return true;
    }
    
    return false;
}

bool getLimitSwitchStatus(int limitSwitchPin)
{
    int reading = digitalRead(limitSwitchPin);  //0 = x, 1 = y, 2 = z, 3 = t
    // If the switch changed, due to noise or pressing:
    if (reading != lastLimitSwitchState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // if the button state has changed:
      if (reading != limitSwitchPin) {
        limitSwitchState = reading;
      }
    }
    
    // save the reading. Next time through the loop, it'll be the lastButtonState:
    lastLimitSwitchState = reading;

    return limitSwitchState;
}

