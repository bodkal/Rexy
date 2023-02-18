#include <micro_ros_arduino.h>
#include <Wire.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rexy_msg/msg/sonar.h>
#include <sensor_msgs/msg/imu.h>


// ---------------------- ros var----------------------------- 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_sonar;

sensor_msgs__msg__Imu msg_imu;
rexy_msg__msg__Sonar sonar_msg;


  enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
  } state;


#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)\


 bool create_entities()
  {

    allocator = rcl_get_default_allocator();
    
    // create init_options
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
    ROSIDL_GET_MSG_TYPE_SUPPORT(rexy_msg, msg, Sonar),"sonar_state"));

   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    return true;
  }


    void destroy_entities(){
          rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
    rcl_publisher_fini(&publisher_sonar, &node);
    rcl_publisher_fini(&publisher_imu, &node);

    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    }



// ----------------------- sonar var---------------------------
#include <NewPing.h>
#define SONAR_NUM     2 // Number of sensors.
#define MAX_DISTANCE 4000 // Maximum distance (in mm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
int32_t mm[SONAR_NUM];              // Where the ping distances are stored.

uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {        // Sensor object array.
  NewPing(14, 15, MAX_DISTANCE),    // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(16, 17, MAX_DISTANCE)

};

// ----------------------- imu var---------------------------
#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU;
byte imuINTPin = 8;

// ---------------------- ros fun ----------------------------- 



// ----------------------- imu  fun---------------------------

// This function is called whenever an interrupt is detected by the arduino
void interrupt_handler()
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    refrash_Ori();
    refrash_Acc();
    refrash_Gyro();

    get_dis();


}

   void refrash_Ori(){
    msg_imu.orientation.x= myIMU.getQuatI();
    msg_imu.orientation.y = myIMU.getQuatJ();
    msg_imu.orientation.z = myIMU.getQuatK();
    msg_imu.orientation.w = myIMU.getQuatReal();
  }

  void refrash_Acc(){
      msg_imu.linear_acceleration.x= myIMU.getLinAccelX();
      msg_imu.linear_acceleration.y= myIMU.getLinAccelY();
      msg_imu.linear_acceleration.z= myIMU.getLinAccelZ();
  }

   void refrash_Gyro(){
    msg_imu.angular_velocity.x= myIMU.getGyroX();
    msg_imu.angular_velocity.y= myIMU.getGyroY();
    msg_imu.angular_velocity.z= myIMU.getGyroZ(); 
  }

// ----------------------- sonar  fun---------------------------

void get_dis(){
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) {sonar_msg.right=mm[0];sonar_msg.left=mm[1];  } 
   // Sensor ping cycle complete, do something with the results.
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
  state = WAITING_AGENT;
  
  delay(1000);


//  ------------------------init sonar  -------------------------------
 
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

   delay(1000);

 //  ------------------------init imu -------------------------------
  Wire.begin();
  while (myIMU.begin(BNO080_DEFAULT_ADDRESS, Wire, imuINTPin) == false){delay(100);}
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  // prepare interrupt on falling edge (= signal of new data available)
  attachInterrupt(digitalPinToInterrupt(imuINTPin), interrupt_handler, FALLING);
  // enable interrupts right away to not miss first reports
  interrupts();
  
  //Enable dynamic calibration for accel, gyro, and mag
  myIMU.calibrateAll(); //Turn on cal for Accel, Gyro, and Mag

  //Send data update every 50ms
  myIMU.enableRotationVector(30);  
  myIMU.enableLinearAccelerometer(30); 
  myIMU.enableGyro(30);            

  while(!calibrite()){delay(100);}
  
 
}


bool calibrite(){

   myIMU.saveCalibration(); //Saves the current dynamic calibration data (DCD) to memory
    myIMU.requestCalibrationStatus(); //Sends command to get the latest calibration status

   // for(int counter = 100;counter>0;--counter){
    while(1){

        if(myIMU.dataAvailable() == true)
        {
          if(myIMU.calibrationComplete() == true)
          {
            return true;
          }  
        }
     }
return false;
}
int reduse_rate=0;
void loop() {
    
      switch (state) {
      case WAITING_AGENT:
        digitalWrite(LED_PIN, LOW);
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
      case AGENT_AVAILABLE:
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) {
          destroy_entities();
        };
        break;
      case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {

         myIMU.dataAvailable();
         reduse_rate++;
         if(reduse_rate==10000){
         RCSOFTCHECK(rcl_publish(&publisher_imu, &msg_imu, NULL));
         RCSOFTCHECK(rcl_publish(&publisher_sonar, &sonar_msg, NULL));
         reduse_rate=0;
         }
       // RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
        }
        break;
      case AGENT_DISCONNECTED:
        digitalWrite(LED_PIN, LOW);
        destroy_entities();
        state = WAITING_AGENT;
        break;
      default:
        break;
    }
}
