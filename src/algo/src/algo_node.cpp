#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "ropigo/SimpleWrite.h"
#include "std_msgs/Int16MultiArray.h"

#include <math.h>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <thread>

#define MAX_LINE_FOLLOWING_SPEED 0.2

#define LINE_FOLLOWER_LENGTH 5
#define IR_THREASHOLD 900

#define MAX_LINEAR_SPEED 0.8f
#define MIN_LINEAR_SPEED -0.8f

#define MAX_ANGULAR_LINEAR_SPEED 1.0f
#define MIN_ANGULAR_LINEAR_SPEED -1.0f

#define MAX_ANGULAR_SPEED 1.0f
#define MIN_ANGULAR_SPEED -1.0f

#define FREQUENCY 10          // Hz
#define FREQUENCY_ENCODERS 10 // Hz

#define US_HEAD_ROTATION_INC 0.1

#define ASYNC_LOOP

// STATICS

#define WHEEL_DIAMETER 62.5                                  // mm
#define PULSE_BY_ROTATION 18                                 // pulses
#define DISTANCE_WHEEL_TO_CENTER 48.75                       // mm
#define DISTANCE_WHEEL_TO_WHEEL DISTANCE_WHEEL_TO_CENTER * 2 // mm

// MACROS

#define PID_INIT(PID, KP, KI, KD, MAX, MIN) \
  do                                        \
  {                                         \
    PID.kp = atof(KP);                      \
    PID.ki = atof(KI);                      \
    PID.kd = atof(KD);                      \
    PID.control_type = #PID;                \
    PID.period = 1.0 / FREQUENCY_ENCODERS;  \
    PID.max_control = MAX;                  \
    PID.min_control = MIN;                  \
  } while (0)

#include "decision_modes.hpp"
#include "robot_actions.hpp"
#define GEN_ENUM
#include "decision_modes.hpp"
#include "robot_actions.hpp"
#undef GEN_ENUM

class Semaphore
{
public:
  Semaphore(int count_ = 0)
      : count(count_)
  {
  }

  inline void notify()
  {
    std::unique_lock<std::mutex> lock(mtx);
    count++;
    //notify the waiting thread
    cv.notify_one();
  }
  inline void wait()
  {
    std::unique_lock<std::mutex> lock(mtx);
    while (count == 0)
    {
      //wait on the mutex until notify is called
      cv.wait(lock);
    }
    count--;
  }

private:
  std::mutex mtx;
  std::condition_variable cv;
  int count;
};

std::mutex mutex_line_follower;
Semaphore semaphore_wait_first_for_encoders(0);

int16_t line_follower[LINE_FOLLOWER_LENGTH];
float line_follower_sidemost = 0, line_follower_side = 0; // coeffs for how hard the robot should turn when the right/left is detected

struct robot_t
{
  float angular_speed;
  float linear_speed;
  float us_head_position;
  float angular_position;
} Robot;

struct PID_t
{
  float kp, ki, kd;
  float max_control, min_control;
  float last_error;
  float total_error;
  float period = 1.0;
  std::string control_type;
} pid_angular_linear, pid_angular, pid_linear;

struct depth_t
{
  std::mutex mutex;
  int16_t depth;
} depth;
struct encoder_callback_t
{
  uint16_t init_count = 1;
  std::mutex mutex;
  bool new_value = false;
  float tick_speed;
  std::chrono::_V2::steady_clock::time_point last_updated_instant;
} encoder_l, encoder_r;

struct robot_speed_control_t
{
  float linear;
  float angular;
};

union robot_orders_t
{
  robot_speed_control_t advance_speed;
  float turning_position;
  float us_head_position;
};

struct robot_controls_t
{
  tagrobot_actions_t action;
  robot_orders_t order;
};

#define ROBOT_ADVANCE(STRUCT_NAME, LINEAR_SPEED, ANGULAR_SPEED) \
  do                                                            \
  {                                                             \
    STRUCT_NAME->action = tagrobot_actions_t::ADVANCING;        \
    STRUCT_NAME->order.advance_speed.linear = LINEAR_SPEED;     \
    STRUCT_NAME->order.advance_speed.angular = ANGULAR_SPEED;   \
  } while (0)

#define ROBOT_TURN(STRUCT_NAME, ANGLE)                 \
  do                                                   \
  {                                                    \
    STRUCT_NAME->action = tagrobot_actions_t::TURNING; \
    STRUCT_NAME->order.turning_position = ANGLE;       \
  } while (0)

#define ROBOT_US_SENSE(STRUCT_NAME, ANGLE)                \
  do                                                      \
  {                                                       \
    STRUCT_NAME->action = tagrobot_actions_t::US_SENSING; \
    STRUCT_NAME->order.us_head_position = ANGLE;          \
  } while (0)

bool get_l_r_encoders(encoder_callback_t *enc_l, encoder_callback_t *enc_r, float *speed_r, float *speed_l)
{
  enc_l->mutex.lock();
  enc_r->mutex.lock();

  *speed_r = enc_r->tick_speed;
  *speed_l = enc_l->tick_speed;

  const bool new_value = enc_l->new_value && enc_r->new_value;

  if (new_value)
  {
    enc_l->new_value = false;
    enc_r->new_value = false;
  }

  enc_r->mutex.unlock();
  enc_l->mutex.unlock();

  return new_value;
}

void encoder_callback(encoder_callback_t *enc, const std_msgs::Int16::ConstPtr &msg, const int16_t last)
{
  auto instant = std::chrono::steady_clock::now();
  enc->mutex.lock();

  enc->tick_speed = (float)(msg->data - last) / (std::chrono::duration<float>(instant - enc->last_updated_instant).count());
  enc->new_value = true;
  enc->last_updated_instant = instant;

  if (enc->init_count-- > 0)
  {
    semaphore_wait_first_for_encoders.notify();
  }

  enc->mutex.unlock();
}

void lwheel_callback(const std_msgs::Int16::ConstPtr &msg)
{
  static int16_t last = msg->data;

  ROS_INFO("Lwheel msg: %d", msg->data);

  encoder_callback(&encoder_l, msg, last);

  last = msg->data;
}

void rwheel_callback(const std_msgs::Int16::ConstPtr &msg)
{
  static int16_t last = msg->data;

  ROS_INFO("Rwheel msg: %d", msg->data);

  encoder_callback(&encoder_r, msg, last);

  last = msg->data;
}

void line_follower_callback(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
#if LINE_FOLLOWER_LENGTH != 5
#error "LINE_FOLLOWER_LENGTH preprocessor is different from what has been designed in this function"
#endif
  bool discarded = false;
  for (int ii = 0; ii < LINE_FOLLOWER_LENGTH; ++ii)
  {
    int16_t value = msg->data[ii];
    ROS_INFO("Line Follower @%d: %d", ii, value);
    if (value < 0 || value > 1000)
    {
      ROS_WARN("Discarded values");
      discarded = true;
    }
  }

  if (!discarded)
  {
    mutex_line_follower.lock();
    std::copy(std::begin(msg->data), std::end(msg->data), std::begin(line_follower));
    mutex_line_follower.unlock();
  }
}

void ultrasonic_callback(const std_msgs::Int16::ConstPtr &msg)
{
  depth.mutex.lock();
  depth.depth = msg->data;
  depth.mutex.unlock();
  ROS_INFO("Ultrasonic depth sensed: %d", msg->data);
}

float make_between_control_bounds(PID_t *pid, float *value)
{
  if (*value > pid->max_control)
  {
    *value = pid->max_control;
  }
  else if (*value < pid->min_control)
  {
    *value = pid->min_control;
  }
}

float compute_pid(PID_t *pid, float setpoint, float sensed_output)
{
  const float error = setpoint - sensed_output;
  pid->total_error += error;

  const float p_term = pid->kp * error;
  const float i_term = pid->ki * pid->total_error * pid->period;
  const float d_term = pid->kd * (pid->last_error - error) / pid->period;
  float control_signal = p_term + i_term + d_term;

  make_between_control_bounds(pid, &control_signal);
  ROS_INFO("[PID][%- 15s] setpoint: % 8.4f, error: % 8.4f, p: % 8.4f, i: % 8.5f, d: % 8.4f -> control: % 8.4f", pid->control_type.c_str(), setpoint, error, p_term, i_term, d_term, control_signal);

  pid->last_error = error;

  return control_signal;
}

inline float get_linear_speed_from_ticks(const float ticks)
{
  return ticks * (2 * M_PI / PULSE_BY_ROTATION) * WHEEL_DIAMETER / 2.0;
}

inline float get_angular_speed(const float linear_speed_r, const float linear_speed_l)
{
  return (linear_speed_r - linear_speed_l) / (DISTANCE_WHEEL_TO_WHEEL);
}

inline float get_linear_speed(const float linear_speed_r, const float linear_speed_l)
{
  return 1e-3 * (linear_speed_r + linear_speed_l) / 2.0;
}

inline float get_angular_position(const float linear_speed_r, const float linear_speed_l, float last_angular_position)
{
  static auto last_chrono = std::chrono::steady_clock::now();
  auto current_chrono = std::chrono::steady_clock::now();
  // only works when speed of right wheel equals the speed of the left
  const float ret = last_angular_position + 2.0 * get_linear_speed(linear_speed_r, linear_speed_l) / (DISTANCE_WHEEL_TO_WHEEL * 1e-3) * std::chrono::duration<float>(current_chrono - last_chrono).count();
  last_chrono = current_chrono;
  return ret;
}

inline float get_radius_from_icc(const float linear_speed_r, const float linear_speed_l)
{
  if (linear_speed_l == linear_speed_r)
  {
    return std::numeric_limits<float>::max();
  }

  return (DISTANCE_WHEEL_TO_WHEEL * 1e-3) / 2.0 * (linear_speed_r + linear_speed_l) / (linear_speed_r - linear_speed_l);
}

float line_follower_sum(const int16_t local_line_follower[LINE_FOLLOWER_LENGTH], const float leftmost, const float left, const float middle, const float right, const float rightmost)
{
  float sum = 0.0;

#if LINE_FOLLOWER_LENGTH != 5
#error "LINE_FOLLOWER_LENGTH preprocessor is different from what has been designed in this function"
#endif

  sum += (local_line_follower[0] > IR_THREASHOLD ? 1.0 : 0.0) * rightmost;
  sum += (local_line_follower[1] > IR_THREASHOLD ? 1.0 : 0.0) * right;
  sum += (local_line_follower[2] > IR_THREASHOLD ? 1.0 : 0.0) * middle;
  sum += (local_line_follower[3] > IR_THREASHOLD ? 1.0 : 0.0) * left;
  sum += (local_line_follower[4] > IR_THREASHOLD ? 1.0 : 0.0) * leftmost;

  ROS_INFO("Sum: % 8.4f", sum);

  return sum;
}

void process_line_following(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{
  int16_t local_line_follower[LINE_FOLLOWER_LENGTH];

  mutex_line_follower.lock();
  std::copy(std::begin(line_follower), std::end(line_follower), std::begin(local_line_follower));
  mutex_line_follower.unlock();

  if (line_follower_sum(local_line_follower, 1, 1, 1, 1, 1) == 0)
  {
    *mode = tagrobot_decision_mode_t::DEPTH_SENSING;
    ROBOT_ADVANCE(controls, 0, 0);
    return;
  }

  for (int ii = 0; ii < LINE_FOLLOWER_LENGTH; ++ii)
  {
    ROS_INFO("Line Follower activation @%d: %d", ii, local_line_follower[ii] > IR_THREASHOLD);
  }

  const float angle_of_line = line_follower_sum(local_line_follower,
                                                -line_follower_sidemost,
                                                -line_follower_side,
                                                0,
                                                line_follower_side,
                                                line_follower_sidemost);
  const float linear_speed = std::max(0.0f, std::abs(line_follower_sum(local_line_follower,
                                                                       -MAX_LINE_FOLLOWING_SPEED, //leftmost
                                                                       -MAX_LINE_FOLLOWING_SPEED / 2,
                                                                       MAX_LINE_FOLLOWING_SPEED,
                                                                       -MAX_LINE_FOLLOWING_SPEED / 2,
                                                                       -MAX_LINE_FOLLOWING_SPEED))); // rightmost

  // if (linear_speed == 0)
  // {
  //   ROBOT_TURN(controls, angle_of_line > 0 ? M_PI_2 : -M_PI_2);
  //   return;
  // }
  ROS_INFO("Wanting to turn: % 8.4f", angle_of_line);
  ROBOT_ADVANCE(controls, linear_speed, angle_of_line);
  return;
}

void process_depth_sensing(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{
  depth.mutex.lock();
  int16_t local_depth = depth.depth;
  depth.mutex.unlock();

  if (Robot.us_head_position == 0)
  {
    // scan from left to right
    ROBOT_US_SENSE(controls, -M_PI_2);
    return;
  }
  else if (Robot.us_head_position >= M_PI_2)
  {
    ROS_WARN("No openings found, retrying");
    ROBOT_US_SENSE(controls, -M_PI_2);
    return;
  }

  if (local_depth > 40)
  {
    // it's an opening
    Robot.angular_position = 0;
    ROBOT_TURN(controls, Robot.us_head_position);
    *mode = tagrobot_decision_mode_t::LINE_ACQUISITION;
    return;
  }

  ROBOT_US_SENSE(controls, Robot.angular_position + US_HEAD_ROTATION_INC);
}

void process_line_acquisition(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{
  int16_t local_line_follower[LINE_FOLLOWER_LENGTH];

  mutex_line_follower.lock();
  std::copy(std::begin(line_follower), std::end(line_follower), std::begin(local_line_follower));
  mutex_line_follower.unlock();

  if (line_follower_sum(local_line_follower, 1, 1, 1, 1, 1) > 0)
  {
    *mode = tagrobot_decision_mode_t::LINE_FOLLOWING;
  }
  ROBOT_ADVANCE(controls, MAX_LINE_FOLLOWING_SPEED, 0);
}

robot_controls_t decision_state_machine(tagrobot_decision_mode_t *mode)
{
  robot_controls_t ret;
  switch (*mode)
  {
  case tagrobot_decision_mode_t::LINE_FOLLOWING:
    process_line_following(mode, &ret);
    break;
  case tagrobot_decision_mode_t::DEPTH_SENSING:
    process_depth_sensing(mode, &ret);
    break;
  case tagrobot_decision_mode_t::LINE_ACQUISITION:
    // process_line_acquisition(mode, &ret);
    break;
  default:
    ROS_ERROR("State not implemented");
  }
  return ret;
}

int main(int argc, char **argv)
{
  if (argc != 13)
  {
    ROS_ERROR("incorrect number of parameters. Got %d", argc);
    return 1;
  }

  const bool finite_cyle = atoi(argv[1]) > 0;
  int running_iterations = atoi(argv[1]) * FREQUENCY;

  PID_INIT(pid_angular_linear, argv[2], argv[3], argv[4], MAX_ANGULAR_LINEAR_SPEED, MIN_ANGULAR_LINEAR_SPEED);
  PID_INIT(pid_linear, argv[5], argv[6], argv[7], MAX_LINEAR_SPEED, MIN_LINEAR_SPEED);
  PID_INIT(pid_angular, argv[8], argv[9], argv[10], MAX_ANGULAR_LINEAR_SPEED, MIN_ANGULAR_LINEAR_SPEED);
  line_follower_sidemost = atof(argv[11]);
  line_follower_side = atof(argv[12]);

  ros::init(argc, argv, "algo_node");

  ros::NodeHandle n;

#ifdef ASYNC_LOOP
  ros::AsyncSpinner spinner(0);
  spinner.start();
#endif

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Publisher cmd_us_pub = n.advertise<std_msgs::Int16>("/cmd_us", 1);

  ros::Subscriber lwheel_sub = n.subscribe("/lwheel", 1, lwheel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber rwheel_sub = n.subscribe("/rwheel", 1, rwheel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber line_follower_sub = n.subscribe("/line_follower", 1, line_follower_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber us_sub = n.subscribe("/us_dist", 1, ultrasonic_callback, ros::TransportHints().tcpNoDelay());

  ros::Rate loop_rate(FREQUENCY);

  ros::ServiceClient led_r_on = n.serviceClient<ropigo::SimpleWrite>("/led_on_right");
  ros::ServiceClient led_r_off = n.serviceClient<ropigo::SimpleWrite>("/led_off_right");
  ros::ServiceClient led_l_on = n.serviceClient<ropigo::SimpleWrite>("/led_on_left");
  ros::ServiceClient led_l_off = n.serviceClient<ropigo::SimpleWrite>("/led_off_left");
  ropigo::SimpleWrite srv;
  if (!led_r_off.call(srv) || !led_l_off.call(srv))
  {
    ROS_ERROR("leds off");
    return 1;
  }

// wait for both encoders to have their first value
#ifdef ASYNC_LOOP
  semaphore_wait_first_for_encoders.wait();
  semaphore_wait_first_for_encoders.wait();
#else
  ros::spinOnce(); // if synchronous, all callbacks are executed when calling `spinOnce`, so all 2 encoders ahve their first value then
#endif

  float last_angular_position = 0;

  tagrobot_decision_mode_t robot_decision = tagrobot_decision_mode_t::LINE_FOLLOWING, last_decision = robot_decision;

  while (ros::ok() && (!finite_cyle || running_iterations-- > 0))
  {
    float speed_ticks_l, speed_ticks_r;

    const bool new_enc_value = get_l_r_encoders(&encoder_l, &encoder_r, &speed_ticks_r, &speed_ticks_l);
    // if (new_enc_value)
    // {
    const robot_controls_t robot_control = decision_state_machine(&robot_decision);

    const float speed_linear_r = get_linear_speed_from_ticks(speed_ticks_r);
    const float speed_linear_l = get_linear_speed_from_ticks(speed_ticks_l);

    // Robot controls
    switch (robot_control.action)
    {
    case tagrobot_actions_t::ADVANCING:
      ROS_INFO("tOTO: %f", robot_control.order.advance_speed.angular);
      Robot.angular_speed = compute_pid(&pid_angular_linear, 0, robot_control.order.advance_speed.angular);
      Robot.linear_speed = robot_control.order.advance_speed.linear; // compute_pid(&pid_linear, robot_control.order.advance_speed.linear, get_linear_speed(speed_linear_r, speed_linear_l));
      break;
    case tagrobot_actions_t::TURNING:
      last_angular_position = get_angular_position(speed_linear_r, speed_linear_l, last_angular_position);
      Robot.angular_speed = compute_pid(&pid_angular, robot_control.order.turning_position, last_angular_position);
      Robot.linear_speed = 0;
      break;
    case tagrobot_actions_t::US_SENSING:
      Robot.linear_speed = 0;
      Robot.angular_speed = 0;
      break;
    }

    // Head controls
    switch (robot_control.action)
    {
    case tagrobot_actions_t::ADVANCING:
    case tagrobot_actions_t::TURNING:
      Robot.us_head_position = 0;
      break;
    case tagrobot_actions_t::US_SENSING:
      Robot.us_head_position = robot_control.order.us_head_position;
      break;
    }

    ROS_INFO("Sensed wheel speeds: R: % 10.5f, L: % 10.5f", speed_linear_r, speed_linear_l);
    ROS_INFO("Chosen action is: %s", get_string_robot_actions_t(robot_control.action));

    if (robot_decision != last_decision)
    {
      ROS_INFO("New decision has been taken: %s", get_string_robot_decision_mode_t(robot_decision));
      last_decision = robot_decision;
    }
    // }

    ROS_INFO("Controls: linear_speed: % 10.5f, angular_speed: % 10.5f, angular_position: % 10.5f, head position: % 10.5f", Robot.linear_speed, Robot.angular_speed, Robot.angular_position, Robot.us_head_position);

    geometry_msgs::Twist vel;
    vel.linear.x = Robot.linear_speed;
    vel.angular.z = Robot.angular_speed;
    cmd_vel_pub.publish(vel);

    std_msgs::Int16 us_pos;
    us_pos.data = (int16_t)Robot.us_head_position;
    cmd_us_pub.publish(us_pos);

    std::thread([&] {
      if (Robot.angular_speed > 0)
      {
        led_l_on.call(srv);
      }
      else if (Robot.angular_speed < 0)
      {
        led_r_on.call(srv);
      }
    }).detach();

    std::thread([&] {
      if (Robot.angular_speed > 0.0)
      {
        led_r_off.call(srv);
      }
      else if (Robot.angular_speed < 0.0)
      {
        led_l_off.call(srv);
      }
    }).detach();

#ifndef ASYNC_LOOP
    ros::spinOnce();
#endif
    const bool met = loop_rate.sleep();

    if (!met)
    {
      ROS_ERROR("Cycle was too long to match desired frequency");
    }
  }

  led_r_on.call(srv);
  led_l_on.call(srv);

  geometry_msgs::Twist vel; // defaults to all 0
  cmd_vel_pub.publish(vel);

  return 0;
}
