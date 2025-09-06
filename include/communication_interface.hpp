#ifndef COMMUNICATION_INTERFACE_HPP
#define COMMUNICATION_INTERFACE_HPP

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/logging_macros.h>
#include <Arduino.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/vector3.h>
//#include <sensor_msgs/msg/magnetic_field.h>

extern rcl_publisher_t depth_pub;
extern rcl_publisher_t imu_pub;
extern rcl_publisher_t orientation_pub;

extern rcl_subscription_t throttle_sub;
extern rcl_subscription_t calibration_sub;
extern rcl_subscription_t led_sub;

extern rclc_executor_t executor;
extern rcl_node_t node;
extern rcl_allocator_t allocator;
extern rclc_support_t support;

void initializeCommunication();
//publishers:
void sendDepth(float depth);
void sendIMUReadings(float ax, float ay, float az, float gx, float gy, float gz,
                     float mx, float my, float mz);
void sendOrientation(float roll, float pitch, float yaw);

//subscribers:
void throttleCb(const void * msgin);
void calibrationCb(const void * msgin);
void ledCb(const void * msgin);
//void dropperCb(const void * msgin);

void checkForCommands();
#endif  // COMMUNICATION_INTERFACE_HPP