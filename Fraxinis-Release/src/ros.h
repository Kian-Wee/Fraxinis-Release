#pragma once
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

#include <misc_fn.h>

// rcl_service_impl_t payload_subscriber;
rcl_subscription_t payload_subscriber;
std_msgs__msg__Bool payload_in_msg;
// rcl_publisher_t payload_publisher;
// std_msgs__msg__Bool payload_out_msg;

rcl_subscription_t counter_subscriber;
// rcl_service_impl_t counter_subscriber;
std_msgs__msg__Bool counter_in_msg;
// rcl_publisher_t counter_publisher;
// std_msgs__msg__Bool counter_out_msg;

rcl_subscription_t thrust_subscriber;
std_msgs__msg__Int32 thrust_in_msg;
// rcl_publisher_t thrust_publisher;
// std_msgs__msg__Int32 thrust_out_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void error_loop()
{
    while (1)
    {
        // digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
        delay(100);
    }
}

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            error_loop();            \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }

IPAddress agent_ip(192, 168, 163, 93);
size_t agent_port = 8888;
char ssid[] = "*****";
char psk[] = "*****";

void payload_subscription_callback(const void *msgin)
{
	const std_msgs__msg__Bool *payload_in_msg = (const std_msgs__msg__Bool *)msgin;
}

void counter_subscription_callback(const void *msgin)
{
	const std_msgs__msg__Bool *counter_in_msg = (const std_msgs__msg__Bool *)msgin;
}

void thrust_subscription_callback(const void *msgin)
{
	const std_msgs__msg__Int32 *counter_in_msg = (const std_msgs__msg__Int32 *)msgin;
}

void setupROS()
{

    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &payload_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "micro_ros_control_node"));
    RCCHECK(rclc_executor_add_subscription(&executor, &payload_subscriber, &payload_in_msg, &payload_subscription_callback, ON_NEW_DATA));

    RCCHECK(rclc_subscription_init_default(
        &counter_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "micro_ros_control_node"));
    RCCHECK(rclc_executor_add_subscription(&executor, &counter_subscriber, &counter_in_msg, &counter_subscription_callback, ON_NEW_DATA));

    RCCHECK(rclc_subscription_init_default(
        &thrust_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "micro_ros_control_node"));
    RCCHECK(rclc_executor_add_subscription(&executor, &thrust_subscriber, &thrust_in_msg, &thrust_subscription_callback, ON_NEW_DATA));
// RCCHECK(rclc_publisher_init_default(
//   &thrust_publisher,
//   &node,
//   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//   "micro_ros_control_node"));

// There are 2 main sources of blocking code, the reading of PWM and the release of servo, which are both time dependant
// If using a dual core S3, seperate these 2 tasks to run on a thread so that the other 2 tasks are responsive
#ifdef ESP32S3
	xTaskCreatePinnedToCore(doTask0,
							"Task 0",
							237680,
							NULL,
							1,
							NULL,
							pro_cpu);

	// Start Task 1 (in Core 1)
	xTaskCreatePinnedToCore(doTask1,
							"Task 1",
							6000,
							NULL,
							1,
							NULL,
							app_cpu);
#endif

println("Finish setup");
}

