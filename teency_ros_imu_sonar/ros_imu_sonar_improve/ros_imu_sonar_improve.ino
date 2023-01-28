#include <micro_ros_arduino.h>
#include <Wire.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
//#include <rexy_msg/msg/sonar.hpp>

#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>

// ---------------------- ros var----------------------------- 
rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_sonar;

std_msgs__msg__Int32MultiArray msg_sonar;

sensor_msgs__msg__Imu msg_imu;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ----------------------- imu var---------------------------
#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU;
byte imuINTPin = 8;


// ----------------------- sonar var---------------------------
#include <NewPing.h>
#define SONAR_NUM     2 // Number of sensors.
#define MAX_DISTANCE 2000 // Maximum distance (in mm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
int32_t mm[SONAR_NUM];              // Where the ping distances are stored.

uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {        // Sensor object array.
  NewPing(14, 15, MAX_DISTANCE),    // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(16, 17, MAX_DISTANCE)

};

// ---------------------- ros fun ----------------------------- 


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(1000);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{    
  RCSOFTCHECK(rcl_publish(&publisher_imu, &msg_imu, NULL));
  RCSOFTCHECK(rcl_publish(&publisher_sonar, &msg_sonar, NULL));
}



// ----------------------- imu  fun---------------------------

// This function is called whenever an interrupt is detected by the arduino
void interrupt_handler()
{
 
    refrash_Mag();
    refrash_Acc();
    refrash_Gyro();
    get_dis();

}

  
   void refrash_Mag(){
    msg_imu.orientation.x= myIMU.getQuatI();
    msg_imu.orientation.y = myIMU.getQuatJ();
    msg_imu.orientation.z = myIMU.getQuatK();
    msg_imu.orientation.w = myIMU.getQuatReal();
    
  }

  void refrash_Acc(){
      msg_imu.linear_acceleration.x= myIMU.getLinAccelX();//Acc[0];
      msg_imu.linear_acceleration.y= myIMU.getLinAccelY();//Acc[1];
      msg_imu.linear_acceleration.z= myIMU.getLinAccelZ();//Acc[2];
  }

   void refrash_Gyro(){
    msg_imu.angular_velocity.x= myIMU.getGyroX();//Acc[0];
    msg_imu.angular_velocity.y= myIMU.getGyroY();//Acc[1];
    msg_imu.angular_velocity.z= myIMU.getGyroZ();//Acc[2]; 
  }

// ----------------------- sonar  fun---------------------------

void get_dis(){
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) {msg_sonar.data.data=mm;} // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      mm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
 }
  
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    mm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM*10;
}


  
  
//  ------------------------ teency -------------------------------
void setup() {
 //  ------------------------init ros -------------------------------
  
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),"imu_state"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_sonar,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),"sonar_state"));
    
  // create timer,
  const unsigned int timer_timeout = 30;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));


 // int32_t value[2] = {0,0};
  msg_sonar.data.capacity = 2; 
  msg_sonar.data.size = 2;
  msg_sonar.data.data = (int32_t*) malloc(msg_sonar.data.capacity * sizeof(int32_t));
  
 //  ------------------------init imu -------------------------------
  Wire.begin();
    if (myIMU.begin(BNO080_DEFAULT_ADDRESS, Wire, imuINTPin) == false)
      while (1);

  myIMU.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  // prepare interrupt on falling edge (= signal of new data available)
  attachInterrupt(digitalPinToInterrupt(imuINTPin), interrupt_handler, FALLING);
  // enable interrupts right away to not miss first reports
  interrupts();
  
  //Enable dynamic calibration for accel, gyro, and mag
  myIMU.calibrateAll(); //Turn on cal for Accel, Gyro, and Mag

  //Enable Game Rotation Vector output
  myIMU.enableGameRotationVector(30);  //Send data update every 50ms
  myIMU.enableMagnetometer(30);        //Send data update every 50ms
  myIMU.enableGyro(30);                //Send data update every 50ms
  myIMU.enableLinearAccelerometer(30); //Send data update every 50ms
  
 //  ------------------------init sonar  -------------------------------
 
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() {
  myIMU.dataAvailable();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(30)));
}
