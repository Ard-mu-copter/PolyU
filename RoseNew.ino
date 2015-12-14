//FL : CW
//FR : CCW
//BL : CCW
//BR : CW


/*
**    Hong Kong Polytechnic Unversity 2014-2015 Academic Year
**            
**                King Ho
**             
**         Ard-mu-copter: A Simple Open Source Quadcopter Platform
**
**    Edited on "ArduPilot-Arduino-1.0.3-windows" IDE.
**    This program is arget to run on Ardupilot (apm2.52) control board which will drive a quadcopter
*/



/*libraries for Ardupilot's hardware operation and some ready equations to control the flying*/ 
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_HAL.h>        
#include <AP_HAL_AVR.h>
#include <PID.h>
#include <AP_RangeFinder.h>
#include <Filter.h>
#include <AP_Baro.h>
#include <AP_Declination.h>
#include <AP_HAL_Empty.h>
#include <AP_Compass.h> // Compass Library
#include <AP_GPS.h>   //GPS Library
#include <stdlib.h>

#define GRAVITY -10

float yaw_current_location = 0;    //for speed mode of yaw control only
int motorsOff = 0;


/*Motors is connected(indirectly) on output channels*/
#define MOTOR_FR   0    // Front right motors, output channel 1
#define MOTOR_BL   1    // back left motors, output channel 2
#define MOTOR_FL   2    // Front left motors, output channel 3  
#define MOTOR_BR   3    // back right motors, output channel 4

//Specify the hardware is apm2.x, the "hal" variable name cannot be changed, it is used inside "AP_HAL_MAIN()"
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;  //create an aliase for AP_HAL_AVR_APM2, when we using other version of apm board we can simply change "AP_HAL_AVR_APM2". 

//IMU(accelerometer and gyroscopes)
AP_InertialSensor_MPU6000 imu;

// filter for sonar
ModeFilterInt16_Size5 mode_filter(2);
//sonar
AP_RangeFinder_MaxsonarXL *sonar;


//for barometer
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
static uint32_t timer;

//for compass
AP_Compass_HMC5843 compass;
uint32_t timer_compass;


//for GPS
AP_GPS_UBLOX gps;
#define T6 1000000
#define T7 10000000
uint32_t timer_gps;
uint32_t timer_serial;


extern int AutoZcontrol;
extern int AutoXYcontrol;



//1.Initialize IMU DMP function.  IMU DMP function has already encapsulated in the <AP_InertialSensor.h>. 
//2.Specify indicated output channels with output frequench 490Hz.
//3.Select input pin 0 in the APM 2.52 as sonar analog input. Driver is in the library #include <AP_RangeFinder.h>
//4.Set up the PID parameter to do PID control .
//   P means proportional   I means integral   D means derivative 
//   parameter setup details can be found in Ard-mu-copter: A Simple Open Source Quadcopter Platform : page 5 Section Tuning PID Coefficients




void setup(){

  
    //  LED_Test();
     IMU_Init();
     
     //Compass_Init();

     Output_Init();
     
     //GPS_Init();
     
     Sonar_Init();
     

     
     //Baro_Init();
     
    
     
     PID_Init();
     
  //   Position_Init();

}

// each loop contain three steps:
// 1.  read input if no auto control /  compute roll pitch yaw error from outerloop if auto control is on
//                            AUX1 is auto control button:       1 is on /  0 is off
//     details of computing outerloop error to convert to innerloop error:     Ard-mu-copter: A Simple Open Source Quadcopter Platform  page 4   line 11 to line 14 
//                                                                                                                                      page 5   line 22 to line 25
// 2 . Receive  roll pitch yaw error as input to PID_Caliburation() which is PID angluar controller. Computing U1 U2 U3 U4 value to convert to PWM output of 4 motors
//     details of computing oU1 U2 U3 U4:     Ard-mu-copter: A Simple Open Source Quadcopter Platform  page 5    line 13 to line 116
// 3.  Print the data of pitch roll yaw height of copter to matlab in order to draw the details of these variables





void loop(){
  
     read_input();
     
     IMU_Update();
    // Compass_Update();
   //  PosXY_Update();
     
     Sonar_update();
   //  Baro_Update();
     
   //  GPS_Update();

   

     
    if(AutoZcontrol == 1)
    {
      
      
      
    Z_PositionPID();

  //  XY_PositionPID();
    
    }

     PID_Caliburation();
     
     Matlab_print();
     
   

}
//The AP_HAL_MAIN macro expands to a main function (either an "int main (void)" * or "int main (int argc, const char * argv[])", depending on platform)
AP_HAL_MAIN(); 

