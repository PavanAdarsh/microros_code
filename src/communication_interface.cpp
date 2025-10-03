#include "communication_interface.hpp"

#include "config.hpp"
#include "diagnostics.hpp"
#include "thruster_interface.hpp"

geometry_msgs__msg__Vector3 linearAcceleration_msg;
geometry_msgs__msg__Vector3 angular_vel_msg;
geometry_msgs__msg__Vector3 magnetic_field_msg;
geometry_msgs__msg__Vector3 orientation_msg;
std_msgs__msg__Int32MultiArray pwm_msg;
std_msgs__msg__Float32 depth_msg;
std_msgs__msg__Bool calibration_msg;
std_msgs__msg__Int16 led_msg;
//std_msgs__msg__Bool dropper_control;

rcl_publisher_t linearAcceleration_pub;
rcl_publisher_t angular_vel_pub;
rcl_publisher_t magnetic_field_pub;
rcl_publisher_t orientation_pub;
rcl_publisher_t depth_pub;

rcl_subscription_t pwm_sub;
rcl_subscription_t calibration_sub;
rcl_subscription_t led_sub;

void initializeCommunication() {
  set_microros_serial_transports(Serial);
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_node_t node;
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  rclc_publisher_init_default(
    &linearAcceleration_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "/sensors/linear_acceleration");
  rclc_publisher_init_default(
    &angular_vel_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "/sensors/angular_velocity");
  rclc_publisher_init_default(
    &magnetic_field_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "/sensors/magnetic_field");
  rclc_publisher_init_default(
    &orientation_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "/sensors/orientation");
  rclc_publisher_init_default(
    &depth_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/sensors/depth");
  
  rclc_subscription_init_default(
    &pwm_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/control/pwm");
  rclc_subscription_init_default(
    &calibration_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/control/calibration");
  rclc_subscription_init_default(
    &led_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/control/led");

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &pwm_sub, &pwm_msg, &throttleCb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &calibration_sub, &calibration_msg, &calibrationCb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &ledCb, ON_NEW_DATA);
}

void sendDepth(float depth) {
  depth_msg.data = depth;
  rcl_ret_t ret5 = rcl_publish(&depth_pub, &depth_msg, NULL);
  if (ret5 != RCL_RET_OK) {
    printf("Error publishing depth: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
}}

void sendIMUReadings(float ax, float ay, float az, float gx, float gy, float gz,
                     float mx, float my, float mz) {
  linearAcceleration_msg.x = ax * G;
  linearAcceleration_msg.y = ay * G;
  linearAcceleration_msg.z = az * G;

  angular_vel_msg.x = gx;
  angular_vel_msg.y = gy;
  angular_vel_msg.z = gz;

  magnetic_field_msg.x = mx;
  magnetic_field_msg.y = my;
  magnetic_field_msg.z = mz;

rcl_ret_t ret1 = rcl_publish(&linearAcceleration_pub, &linearAcceleration_msg, NULL);
if (ret1 != RCL_RET_OK) {
    printf("Error publishing linear acceleration: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
}
rcl_ret_t ret2 = rcl_publish(&angular_vel_pub, &angular_vel_msg, NULL);
if (ret2 != RCL_RET_OK) {
    printf("Error publishing angular velocity: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
}
rcl_ret_t ret3 = rcl_publish(&magnetic_field_pub, &magnetic_field_msg, NULL);
if (ret3 != RCL_RET_OK) {
    printf("Error publishing magnetic field: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
}
}

void sendOrientation(float roll, float pitch, float yaw) {
  orientation_msg.x = roll;
  orientation_msg.y = pitch;
  orientation_msg.z = yaw;

  rcl_ret_t ret4 = rcl_publish(&orientation_pub, &orientation_msg, NULL);
  if (ret4 != RCL_RET_OK) {
    printf("Error publishing orientation: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
  } }

void throttleCb(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * pwm_msg =
    (const std_msgs__msg__Int32MultiArray *)msgin;
  
  int32_t pwm_values[NUMBER_OF_THRUSTERS];
    for (size_t i = 0; i < pwm_msg->data.size && i < NUMBER_OF_THRUSTERS; i++) {
        int32_t val = pwm_msg->data.data[i];
  }
  setThrusterThrottle(pwm_values);
}

void calibrationCb(const void * msgin) {
  const std_msgs__msg__Bool * calib =
    (const std_msgs__msg__Bool *)msgin;
  
  bool calibration_mode = calibration_msg.data;
  callUpdateOffset(calibration_mode);
}

void ledCb(const void * msgin) {
  const std_msgs__msg__Int16 * led =
    (const std_msgs__msg__Int16 *)msgin;

  int16_t led_indicator = led_msg.data;
  setLED(led_indicator);

}

// void dropperCb(const std_msgs::Bool& dropper_control){
//   bool activate = dropper_control.data;
//   if(activate)
//   activateDropper();
// }

void checkForCommands() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
