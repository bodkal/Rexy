#include <Servo.h>

#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <picko_core_msgs/srv/on_off.h>
#include <picko_core_msgs/srv/grasp_object.h>
#include <picko_core_msgs/srv/spin_sticker.h>
#include <picko_core_msgs/srv/release_object.h>
#include <std_msgs/msg/int64.h>


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){};}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define SOFT_FINGERS  "soft_fingers"
#define PARALLEL_FINGERS  "parallel_fingers"
#define STICKER "sticker"
#define VACUUM  "vacuum"

Servo sticker_servo;

const int main_valve_pin = 15;
const int change_gripper_pin = 16;
const int grasp_object_pin = A22;
const int pot_pin = 18;

const int propvalve_zero = 2170;

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_service_t service_main_valve;
rcl_service_t service_spin_sticker;
rcl_service_t service_change_gripper;
rcl_service_t service_grasp_object;
rcl_service_t service_release_object;

rcl_publisher_t publisher;

std_msgs__msg__Int64 pub_msg;


picko_core_msgs__srv__OnOff_Response res_on_off;
picko_core_msgs__srv__OnOff_Request req_on_off;

picko_core_msgs__srv__SpinSticker_Response res_spin_sticker;
picko_core_msgs__srv__SpinSticker_Request req_spin_sticker;

picko_core_msgs__srv__GraspObject_Response res_grasp_object;
picko_core_msgs__srv__GraspObject_Request req_grasp_object;

picko_core_msgs__srv__ReleaseObject_Response res_release_object;
picko_core_msgs__srv__ReleaseObject_Request req_release_object;


  bool is_same_gripper(rosidl_runtime_c__String new_name,const char* GRIPPER){
  return strcmp(new_name.data, GRIPPER) == 0;
  }
  
void P_controller_fingers(int target)
{
  double pos = analogRead(pot_pin);
 

  // time difference
  double deltaT = 0.01;

  // PID constants
  double kp = 5;
  double kd = 0;
  double ki = 0.5;

  double a = 0.5;
  double eintegral = 0;
  double e = 0;
  int pwmVal = 0;
  
  const int max_u=300;

  
  unsigned long start_converge_time=0;
  do {

    static double perv_pos = pos ;
    pos = (0.4 * analogRead(pot_pin) + 13.81) * a + (1 - a) * perv_pos;
    
    static double eprev = e;
    // error
    e = target - pos;

    // derivative
    static double dedt = (e - eprev) / (deltaT);

    // integral
    if (abs(e) > 3) {
      eintegral = eintegral + e * deltaT;
    }
    else { eintegral = 0; }

    pwmVal = propvalve_zero;
    if (abs(e) > 3) {
      start_converge_time = millis();
      pwmVal = propvalve_zero + constrain(kp * e + kd * dedt + ki * eintegral, -max_u, max_u);
    }
    analogWrite(grasp_object_pin, pwmVal);

  } while (millis()-start_converge_time>200);
}


void grasp_object_callback(const void * req_grasp_object, void * res_grasp_object) {
  picko_core_msgs__srv__GraspObject_Request * req_in = (picko_core_msgs__srv__GraspObject_Request *) req_grasp_object;
  picko_core_msgs__srv__GraspObject_Response * res_out = (picko_core_msgs__srv__GraspObject_Response *) res_grasp_object;

  res_out->success = true;

  if (is_same_gripper(req_in->gripper_name ,PARALLEL_FINGERS) && req_in->data >= 50 && req_in->data <= 500) {
    int grasp_value = req_in->data + propvalve_zero + 50 ; // finerg force betwen 50 -  500;
    analogWrite(grasp_object_pin, grasp_value);
  }
  else if (is_same_gripper(req_in->gripper_name,STICKER) ||
           is_same_gripper(req_in->gripper_name,VACUUM) || 
           is_same_gripper(req_in->gripper_name, SOFT_FINGERS)) {
    analogWrite(grasp_object_pin, LOW);
  }
  else {
    res_out->success = false;
  }
}

void release_object_callback(const void * req_release_object, void * res_release_object) {
  picko_core_msgs__srv__ReleaseObject_Request * req_in = (picko_core_msgs__srv__ReleaseObject_Request *) req_release_object;
  picko_core_msgs__srv__ReleaseObject_Response * res_out = (picko_core_msgs__srv__ReleaseObject_Response *) res_release_object;
  res_out->success = true;

  
  const int max_value = 4096;
  const int min_value = 2596;

  if (is_same_gripper(req_in->gripper_name, PARALLEL_FINGERS) && req_in->data >= 0 && req_in->data <= 67) {
    P_controller_fingers(req_in->data);
  }
  else if (is_same_gripper(req_in->gripper_name,STICKER) && req_in->data >= 150) {

    int iterations_number = 150 ;
    int step_size = (max_value - min_value) / iterations_number;
    int delta_sleep = int((req_in->data / iterations_number));
    for (int i = 0; i < iterations_number; ++i ) {
      analogWrite(grasp_object_pin, min_value + i * step_size);
      delay(delta_sleep);
    }

  }
  else if (is_same_gripper(req_in->gripper_name, VACUUM) ||
           is_same_gripper(req_in->gripper_name, SOFT_FINGERS)) {
     analogWrite(grasp_object_pin, max_value);
  }
  else {
    res_out->success = false;
  }
}


void main_valve_callback(const void * req_on_off, void * res_on_off) {
  picko_core_msgs__srv__OnOff_Request * req_in = (picko_core_msgs__srv__OnOff_Request *) req_on_off;
  picko_core_msgs__srv__OnOff_Response * res_out = (picko_core_msgs__srv__OnOff_Response *) res_on_off;

  int main_valve = HIGH ;// HIGH propotional valve close. LOW - propotional valve open
  if (req_in->on) {
    main_valve = LOW;
  }
  digitalWrite(main_valve_pin , main_valve);
  res_out->success = true;
}


void spin_sticker_callback(const void * req_spin_sticker, void * res_spin_sticker) {
  picko_core_msgs__srv__SpinSticker_Request * req_in = (picko_core_msgs__srv__SpinSticker_Request *) req_spin_sticker;
  picko_core_msgs__srv__SpinSticker_Response * res_out = (picko_core_msgs__srv__SpinSticker_Response *) res_spin_sticker;

  const int pull_sticker = 180;
  const int release_sticker = 0;
  const int  motor_stop = 90;
  res_out->success = true;

  if (req_in->pull_time >= 0 && req_in->release_time >= 0) {
    sticker_servo.write(pull_sticker);
    delay(req_in->pull_time);

    // Release pressure from the sprint
    sticker_servo.write(release_sticker);
    delay(req_in->release_time);

    // Stop the motor
    sticker_servo.write(motor_stop);
  }
  else {
    res_out->success = false;
  }
}

void change_gripper_callback(const void * req_on_off, void * res_on_off) {
  picko_core_msgs__srv__OnOff_Request * req_in = (picko_core_msgs__srv__OnOff_Request *) req_on_off;
  picko_core_msgs__srv__OnOff_Response * res_out = (picko_core_msgs__srv__OnOff_Response *) res_on_off;

  int change_gripper_value = LOW; //HIGH is to release gripper and LOW is to lock the gripper
  if (req_in->on) {
    change_gripper_value = HIGH;
  }
  digitalWrite(change_gripper_pin, change_gripper_value);
    res_out->success = true;

}


void setup() {
  set_microros_transports();

  sticker_servo.attach(A8);
  pinMode(change_gripper_pin, OUTPUT);
  pinMode(main_valve_pin, OUTPUT);
  pinMode(grasp_object_pin, OUTPUT);
  // To prevent droping gripper on start
  digitalWrite(change_gripper_pin, HIGH);
  digitalWrite(main_valve_pin, HIGH);

  delay(1000);

  allocator = rcl_get_default_allocator();
  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "gripper_controller", "", &support));

// create publisher
  RCCHECK(rclc_publisher_init_default(&publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),"/gripper_controller/connection"));

  // create service
  RCCHECK(rclc_service_init_default(&service_main_valve, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(picko_core_msgs, srv, OnOff), "/gripper_controller/main_valve"));
  RCCHECK(rclc_service_init_default(&service_change_gripper, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(picko_core_msgs, srv, OnOff), "/gripper_controller/change_gripper"));
  RCCHECK(rclc_service_init_default(&service_spin_sticker, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(picko_core_msgs, srv, SpinSticker), "/gripper_controller/spin_sticker"));
  RCCHECK(rclc_service_init_default(&service_grasp_object, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(picko_core_msgs, srv, GraspObject), "/gripper_controller/grasp_object"));
  RCCHECK(rclc_service_init_default(&service_release_object, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(picko_core_msgs, srv, ReleaseObject), "/gripper_controller/release_object"));


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

  RCCHECK(rclc_executor_add_service(&executor, &service_spin_sticker, &req_spin_sticker, &res_spin_sticker, spin_sticker_callback));
  RCCHECK(rclc_executor_add_service(&executor, &service_main_valve, &req_on_off, &res_on_off, main_valve_callback));
  RCCHECK(rclc_executor_add_service(&executor, &service_change_gripper, &req_on_off, &res_on_off, change_gripper_callback));
  RCCHECK(rclc_executor_add_service(&executor, &service_grasp_object, &req_grasp_object, &res_grasp_object, grasp_object_callback));
  RCCHECK(rclc_executor_add_service(&executor, &service_release_object, &req_release_object, &res_release_object, release_object_callback));



// Assigning dynamic memory to the gripper_name char sequence
req_grasp_object.gripper_name.capacity = 20;
req_grasp_object.gripper_name.data = (char*) malloc(req_grasp_object.gripper_name.capacity * sizeof(char));
req_grasp_object.gripper_name.size = 0;

// Assigning dynamic memory to the gripper_name char sequence
req_release_object.gripper_name.capacity = 20;
req_release_object.gripper_name.data = (char*) malloc(req_grasp_object.gripper_name.capacity * sizeof(char));
req_release_object.gripper_name.size = 0;
}




void loop() {
  pub_msg.data=millis();
  RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
}
