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

#define MAX_LINE_FOLLOWING_SPEED 0.3

#define LINE_FOLLOWER_LENGTH 5
#define IR_THREASHOLD 900

#define MAX_LINEAR_SPEED 0.8f
#define MIN_LINEAR_SPEED 0.0f

#define MAX_ANGULAR_LINEAR_SPEED 1.0f
#define MIN_ANGULAR_LINEAR_SPEED -1.0f

#define MAX_ANGULAR_SPEED 0.3f
#define MIN_ANGULAR_SPEED -0.3f

#define MAX_ANGULAR_SPEED_WALL 0.15f
#define MIN_ANGULAR_SPEED_WALL -0.15f

#define FREQUENCY 10          // Hz
#define FREQUENCY_ENCODERS 10 // Hz

#define US_HEAD_ROTATION_INC 0.1
#define DEPTH_THRESHOLD 25           // cm
#define DEPTH_FOLLOWING_THRESHOLD 20 // cm
#define DEPTH_FOLLOWING_SPEED 0.3
#define DEPTH_WAIT_BEFORE_ACQUISITION 1000 //ms

#define MAX_ERROR_WHEN_TURNED 0.05

#define ASYNC_LOOP
#define DEBUG

#ifdef DEBUG
// #define ARBITRARY_DEBUG
#else
#undef ROS_INFO
#define ROS_INFO(...)
#endif

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
#include "right_angle_directions.hpp"
#include "us_head_positions.hpp"

#define GEN_ENUM
#include "decision_modes.hpp"
#include "robot_actions.hpp"
#include "right_angle_directions.hpp"
#include "us_head_positions.hpp"
#undef GEN_ENUM

#define GEN_VALUES
#include "us_head_positions.hpp"
#undef GEN_VALUES

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
  tagus_head_positions_t us_head_position = tagus_head_positions_t::US_ZERO;
  float angular_position; // be careful when using this attr, reset beforehand and make sure all actions performed by the robot are compatible (turn only is OK)
} Robot;

struct PID_t
{
  float kp, ki, kd;
  float max_control, min_control;
  float last_error;
  float total_error;
  float period = 1.0;
  std::string control_type;
} pid_angular_linear, pid_angular_linear_wall, pid_angular, pid_linear;

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
  tagus_head_positions_t us_head_position;
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

#define ROBOT_ADVANCE_WALL(STRUCT_NAME, LINEAR_SPEED, ANGULAR_SPEED) \
  do                                                                 \
  {                                                                  \
    STRUCT_NAME->action = tagrobot_actions_t::ADVANCING_WALL;        \
    STRUCT_NAME->order.advance_speed.linear = LINEAR_SPEED;          \
    STRUCT_NAME->order.advance_speed.angular = ANGULAR_SPEED;        \
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

  const float abs_diff = std::abs(Robot.linear_speed) - std::abs(Robot.angular_speed);
  const float diff = Robot.linear_speed - Robot.angular_speed;
  // if the robot is goint forward, ticks are of course increasing ! speeds are positive
  // when turning, make sure to count the ticks negatively if the angular position is increasing to the right
  const float sign = abs_diff >= 0.01 ? 1 : (diff >= 0 ? sign : -sign);

  *speed_r = (abs_diff >= 0.001 ? 1 : (diff >= 0 ? 1 : -1)) * enc_r->tick_speed;
  *speed_l = (abs_diff >= 0.001 ? 1 : (diff >= 0 ? -1 : 1)) * enc_l->tick_speed;

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
  if (msg->data > 0 && (msg->data - last) >= 0 && (msg->data - last) < 1000)
  {

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
  else
  {
    ROS_WARN("Discarded encoder value");
  }
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
    if (value < 0 || value > 1500)
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
  static int16_t last = msg->data;
  ROS_INFO("Ultrasonic depth sensed: %d", msg->data);
  if (msg->data >= 0 && std::abs(msg->data - last) < 100)
  {
    // static int16_t last_value = msg->data;
    // if (std::abs(last_value - msg->data) <= 20)
    // {
    depth.mutex.lock();
    depth.depth = msg->data;
    depth.mutex.unlock();
    last = msg->data;
    // last_value = msg->data;
    // }
  }
  else
  {
    ROS_WARN("Discarded ultrasonic value");
  }
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
  const float d_term = pid->kd * (error - pid->last_error) / pid->period;
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

  return sum;
}

bool is_depth_sensor_state_detected()
{
  depth.mutex.lock();
  const int16_t local_depth = depth.depth;
  depth.mutex.unlock();
  return local_depth < DEPTH_THRESHOLD;
}

bool is_depth_sensor_state_detected_nearly()
{
  depth.mutex.lock();
  const int16_t local_depth = depth.depth;
  depth.mutex.unlock();
  return local_depth < (int16_t)((float)DEPTH_THRESHOLD * 2);
}

tagus_head_positions_t get_opposite_side(const tagus_head_positions_t position)
{
  switch (position)
  {
  case tagus_head_positions_t::US_LEFT:
    return US_RIGHT;
  case tagus_head_positions_t::US_RIGHT:
    return US_LEFT;
  case tagus_head_positions_t::US_ZERO:
    return tagus_head_positions_t::US_ZERO;
  default:
    ROS_ERROR("Position %s has no opposite", get_string_us_head_positions_t(position));
  }
  return position;
}

void follow_line_right_angle_turn(tagrobot_decision_mode_t *mode, robot_controls_t *controls, int16_t local_line_follower[LINE_FOLLOWER_LENGTH])
{
  static const float ANGLE = M_PI;
  static bool init_done = false;
  static tagright_angle_directions_t right_angle_turning;

  if (!init_done)
  {
    const float side = line_follower_sum(local_line_follower,
                                         1,
                                         1,
                                         0,
                                         -1,
                                         -1);

    right_angle_turning = side > 0 ? tagright_angle_directions_t::LEFT : tagright_angle_directions_t::RIGHT;

    Robot.angular_position = 0; // reset the angular position before asking to watch the angle of the turn

    ROS_INFO("Right angle detected, now turning: %s", get_string_right_angle_directions_t(right_angle_turning));
    init_done = true;
    *mode = tagrobot_decision_mode_t::LINE_FOLLOWING_RIGHT_ANGLE;
  }
  else if (std::abs(Robot.angular_position) > (ANGLE / 4.0f))
  {
    const float side_reached = line_follower_sum(local_line_follower,
                                                 right_angle_turning == tagright_angle_directions_t::LEFT,
                                                 1,
                                                 1,
                                                 1,
                                                 right_angle_turning == tagright_angle_directions_t::RIGHT);

    if (side_reached > 0)
    {
      ROS_INFO("Right angle stop turning: %s", get_string_right_angle_directions_t(right_angle_turning));
      *mode = tagrobot_decision_mode_t::LINE_FOLLOWING;
      init_done = false;
      return;
    }
  }

  switch (right_angle_turning)
  {
  case tagright_angle_directions_t::LEFT:
    ROBOT_TURN(controls, ANGLE);
    return;
  case tagright_angle_directions_t::RIGHT:
    ROBOT_TURN(controls, -ANGLE);
    return;
  }
}

void process_line_following(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{
  static int counter_right_angle = 0, counter_us = 0;
  float max_speed = MAX_LINE_FOLLOWING_SPEED;
  if (is_depth_sensor_state_detected() && counter_us++ > 0)
  {
    counter_us = 0;
    *mode = tagrobot_decision_mode_t::DEPTH_SENSING;
    return;
  }
  else if (is_depth_sensor_state_detected_nearly())
  {
    ROS_INFO("Line nearly detected");
    max_speed = MAX_LINE_FOLLOWING_SPEED / 2;
  }

  int16_t local_line_follower[LINE_FOLLOWER_LENGTH];
  mutex_line_follower.lock();
  std::copy(std::begin(line_follower), std::end(line_follower), std::begin(local_line_follower));
  mutex_line_follower.unlock();

  for (int ii = 0; ii < LINE_FOLLOWER_LENGTH; ++ii)
  {
    ROS_INFO("Line Follower activation @%d: %d", ii, local_line_follower[ii] > IR_THREASHOLD);
  }

  const int right_angle = line_follower_sum(local_line_follower,
                                            2,
                                            2,
                                            1,
                                            2,
                                            2);

  switch (right_angle)
  {
  case 5:
  case 7:
    if (counter_right_angle++ > 0)
    {
      follow_line_right_angle_turn(mode, controls, local_line_follower); // immediately start to turn, with the current information availables
                                                                         // *mode = tagrobot_decision_mode_t::LINE_FOLLOWING_RIGHT_ANGLE;
      counter_right_angle = 0;
    }
    return;
  case 0:
    *mode = tagrobot_decision_mode_t::LINE_ACQUISITION;
    return;
  }

  const float angle_of_line = line_follower_sum(local_line_follower,
                                                -line_follower_sidemost,
                                                -line_follower_side,
                                                0,
                                                line_follower_side,
                                                line_follower_sidemost);

  const float linear_speed = std::min(
      std::max(0.2f, line_follower_sum(local_line_follower,
                                        -MAX_LINE_FOLLOWING_SPEED, //leftmost
                                        -MAX_LINE_FOLLOWING_SPEED / 1.7,
                                        MAX_LINE_FOLLOWING_SPEED,
                                        -MAX_LINE_FOLLOWING_SPEED / 1.7,
                                        -MAX_LINE_FOLLOWING_SPEED)), // rightmost
      max_speed);

  ROS_INFO("Wanting to turn: % 8.4f", angle_of_line);
  ROBOT_ADVANCE(controls, linear_speed, angle_of_line);
}

void process_line_following_right_angle(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{

  int16_t local_line_follower[LINE_FOLLOWER_LENGTH];
  mutex_line_follower.lock();
  std::copy(std::begin(line_follower), std::end(line_follower), std::begin(local_line_follower));
  mutex_line_follower.unlock();

  follow_line_right_angle_turn(mode, controls, local_line_follower);
}

void process_depth_sensing(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{
  if (!is_depth_sensor_state_detected())
  {
    // no walls detected

    // turn the sensor to the correct side (of the wall)
    const tagus_head_positions_t opposite_side = get_opposite_side(Robot.us_head_position);
    ROBOT_US_SENSE(controls, opposite_side);
    *mode = tagrobot_decision_mode_t::ROTATION_AFTER_DEPTH_SENSED_APERTURE;
    return;
  }

  switch (Robot.us_head_position)
  {
  case tagus_head_positions_t::US_ZERO:
    ROBOT_US_SENSE(controls, US_RIGHT);
    break;
  case tagus_head_positions_t::US_RIGHT:
    ROBOT_US_SENSE(controls, US_LEFT);
    break;
  case tagus_head_positions_t::US_LEFT:
    ROBOT_US_SENSE(controls, US_ZERO);
    break;
  }

  // the measure cannot be trusted yet (not knowing if the servo is still turning)
  *mode = tagrobot_decision_mode_t::DEPTH_SENSING_WAITING;
}

void process_depth_sensing_waiting(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{
  ROBOT_ADVANCE(controls, 0, 0);
  static const int WAITING_ITERATIONS = (float)(DEPTH_WAIT_BEFORE_ACQUISITION * FREQUENCY) / 1000;
  static int counter = WAITING_ITERATIONS;

  if (counter == WAITING_ITERATIONS)
  {
    ROS_INFO("Now depth-sensing-waiting for %d iterations...", WAITING_ITERATIONS);
  }
  counter--;

  if (counter == 0)
  {
    counter = WAITING_ITERATIONS;
    *mode = tagrobot_decision_mode_t::DEPTH_SENSING;
    ROS_INFO("Finished depth-sensing-waiting after %d iterations", WAITING_ITERATIONS);
  }
}

void process_rotation_after_depth_sensed_aperture(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{
  Robot.angular_position = 0;
  *mode = tagrobot_decision_mode_t::ROTATION_WAITING_COMPLETION_APERTURE;
}

void process_rotation_waiting_completion_aperture(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{
  const tagus_head_positions_t opposite_side = get_opposite_side(Robot.us_head_position);
  float position_wanted = 0;

  // the previous state turned the sensor to the position where the wall will be
  switch (opposite_side)
  {
  case tagus_head_positions_t::US_RIGHT:
    position_wanted = -M_PI_2; // turn right
    break;
  case tagus_head_positions_t::US_LEFT:
    position_wanted = M_PI_2; // turn left
    break;
  default:
    return;
  }
  ROBOT_TURN(controls, position_wanted);

  if (std::abs(Robot.angular_position) > std::abs(position_wanted) - MAX_ERROR_WHEN_TURNED)
  {
    ROBOT_ADVANCE(controls, 0, 0);
    *mode = tagrobot_decision_mode_t::DEPTH_FOLLOWING_WALL;
  }
}

void process_depth_following_wall(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{
  int16_t local_line_follower[LINE_FOLLOWER_LENGTH];
  mutex_line_follower.lock();
  std::copy(std::begin(line_follower), std::end(line_follower), std::begin(local_line_follower));
  mutex_line_follower.unlock();
  if (line_follower_sum(local_line_follower, 1, 1, 1, 1, 1) > 0.0f)
  {
    ROBOT_US_SENSE(controls, US_ZERO);
    *mode = tagrobot_decision_mode_t::LINE_FOLLOWING;
    return;
  }

  depth.mutex.lock();
  const int16_t local_depth = depth.depth;
  depth.mutex.unlock();
  ROBOT_ADVANCE_WALL(controls, DEPTH_FOLLOWING_SPEED, local_depth);
}

void process_line_acquisition(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{
  if (is_depth_sensor_state_detected())
  {
    *mode = tagrobot_decision_mode_t::DEPTH_SENSING;
    return;
  }

  int16_t local_line_follower[LINE_FOLLOWER_LENGTH];

  mutex_line_follower.lock();
  std::copy(std::begin(line_follower), std::end(line_follower), std::begin(local_line_follower));
  mutex_line_follower.unlock();

  if (line_follower_sum(local_line_follower, 1, 1, 1, 1, 1) > 0.0f)
  {
    *mode = tagrobot_decision_mode_t::LINE_FOLLOWING;
    return;
  }
  const float max_speed = is_depth_sensor_state_detected_nearly() ? MAX_LINE_FOLLOWING_SPEED / 2 : MAX_LINE_FOLLOWING_SPEED;
  if (is_depth_sensor_state_detected_nearly())
  {
    ROS_INFO("Line nearly detected");
  }
  ROBOT_ADVANCE(controls, max_speed, 0);
}

robot_controls_t decision_state_machine(tagrobot_decision_mode_t *mode)
{
  ROS_INFO("Processing decision: %s", get_string_robot_decision_mode_t(*mode));
  robot_controls_t ret;
  switch (*mode)
  {
  case tagrobot_decision_mode_t::LINE_FOLLOWING:
    process_line_following(mode, &ret);
    break;
  case tagrobot_decision_mode_t::LINE_FOLLOWING_RIGHT_ANGLE:
    process_line_following_right_angle(mode, &ret);
    break;
  case tagrobot_decision_mode_t::LINE_ACQUISITION:
    process_line_acquisition(mode, &ret);
    break;
  case tagrobot_decision_mode_t::DEPTH_SENSING:
    process_depth_sensing(mode, &ret);
    break;
  case tagrobot_decision_mode_t::DEPTH_SENSING_WAITING:
    process_depth_sensing_waiting(mode, &ret);
    break;
  case tagrobot_decision_mode_t::ROTATION_AFTER_DEPTH_SENSED_APERTURE:
    process_rotation_after_depth_sensed_aperture(mode, &ret);
    break;
  case tagrobot_decision_mode_t::ROTATION_WAITING_COMPLETION_APERTURE:
    process_rotation_waiting_completion_aperture(mode, &ret);
    break;
  case tagrobot_decision_mode_t::DEPTH_FOLLOWING_WALL:
    process_depth_following_wall(mode, &ret);
    break;
  default:
    ROS_ERROR("State not implemented");
  }
  return ret;
}

int main(int argc, char **argv)
{
  if (argc != 16)
  {
    ROS_ERROR("incorrect number of parameters. Got %d", argc);
    return 1;
  }

  const bool finite_cyle = atoi(argv[1]) > 0;
  int running_iterations = atoi(argv[1]) * FREQUENCY;

  PID_INIT(pid_angular_linear, argv[2], argv[3], argv[4], MAX_ANGULAR_LINEAR_SPEED, MIN_ANGULAR_LINEAR_SPEED);
  PID_INIT(pid_linear, argv[5], argv[6], argv[7], MAX_LINEAR_SPEED, MIN_LINEAR_SPEED);
  PID_INIT(pid_angular, argv[8], argv[9], argv[10], MAX_ANGULAR_SPEED, MIN_ANGULAR_SPEED);
  PID_INIT(pid_angular_linear_wall, argv[11], argv[12], argv[13], MAX_ANGULAR_SPEED_WALL, MIN_ANGULAR_SPEED_WALL);
  line_follower_sidemost = atof(argv[14]);
  line_follower_side = atof(argv[15]);

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

  tagrobot_decision_mode_t robot_decision = tagrobot_decision_mode_t::LINE_FOLLOWING, last_decision = robot_decision;

  while (ros::ok() && (!finite_cyle || running_iterations-- > 0))
  {
    float speed_ticks_l, speed_ticks_r;

    const bool new_enc_value = get_l_r_encoders(&encoder_l, &encoder_r, &speed_ticks_r, &speed_ticks_l);
    const float speed_linear_r = get_linear_speed_from_ticks(speed_ticks_r);
    const float speed_linear_l = get_linear_speed_from_ticks(speed_ticks_l);
    Robot.angular_position = get_angular_position(speed_linear_r, speed_linear_l, Robot.angular_position);

#ifndef ARBITRARY_DEBUG
    const robot_controls_t robot_control = decision_state_machine(&robot_decision);
#else
    robot_controls_t robot_control;
    robot_control.action = ARBITRARY;
#endif

    // Robot controls
    switch (robot_control.action)
    {
    case tagrobot_actions_t::ADVANCING:
      Robot.angular_speed = compute_pid(&pid_angular_linear, 0, robot_control.order.advance_speed.angular);
      Robot.linear_speed = robot_control.order.advance_speed.linear; // compute_pid(&pid_linear, robot_control.order.advance_speed.linear, get_linear_speed(speed_linear_r, speed_linear_l));
      break;
    case tagrobot_actions_t::ADVANCING_WALL:
      Robot.angular_speed = compute_pid(&pid_angular_linear_wall, DEPTH_FOLLOWING_THRESHOLD, robot_control.order.advance_speed.angular);
      Robot.linear_speed = robot_control.order.advance_speed.linear; // compute_pid(&pid_linear, robot_control.order.advance_speed.linear, get_linear_speed(speed_linear_r, speed_linear_l));
      break;
    case tagrobot_actions_t::TURNING:
    {
      const float angle = robot_control.order.turning_position;
      Robot.angular_speed = compute_pid(&pid_angular, angle, Robot.angular_position);
      Robot.linear_speed = (angle * Robot.angular_speed > 0 ? 1 : -1) * std::abs(Robot.angular_speed);
    }
    break;
    case tagrobot_actions_t::US_SENSING:
      Robot.linear_speed = 0;
      Robot.angular_speed = 0;
      break;
#ifdef DEBUG
    case tagrobot_actions_t::ARBITRARY:
    {
      // const float angle = 0;
      // Robot.angular_speed = compute_pid(&pid_angular, angle, Robot.angular_position);
      // // always stay on the same circle : if target is to go left, then if overshoot, do not go right but instead go back follwoing the same trajectory
      // Robot.linear_speed = (angle * Robot.angular_speed > 0 ? 1 : -1) * std::abs(Robot.angular_speed);
    }
    break;
#endif
    }

    // Head controls
    switch (robot_control.action)
    {
    case tagrobot_actions_t::US_SENSING:
      Robot.us_head_position = robot_control.order.us_head_position;
      break;
      // default:
      //   Robot.us_head_position = tagus_head_positions_t::US_ZERO;
      //   break;
    }

    ROS_INFO("Sensed wheel speeds: R: % 10.5f, L: % 10.5f", speed_linear_r, speed_linear_l);
    ROS_INFO("Chosen action is: %s", get_string_robot_actions_t(robot_control.action));

    if (robot_decision != last_decision)
    {
      ROS_INFO("New decision has been taken: %s", get_string_robot_decision_mode_t(robot_decision));
      last_decision = robot_decision;
    }

    ROS_INFO("Controls: linear_speed: % 10.5f, angular_speed: % 10.5f, angular_position: % 10.5f, head position: %s", Robot.linear_speed, Robot.angular_speed, Robot.angular_position, get_string_us_head_positions_t(Robot.us_head_position));

    geometry_msgs::Twist vel;
    vel.linear.x = Robot.linear_speed;
    vel.angular.z = Robot.angular_speed;
    cmd_vel_pub.publish(vel);

    static tagus_head_positions_t last_head_pos = tagus_head_positions_t::US_LEFT;
    if (last_head_pos != Robot.us_head_position)
    {
      std_msgs::Int16 us_pos;
      us_pos.data = get_value_us_head_positions_t(Robot.us_head_position);
      cmd_us_pub.publish(us_pos);
      last_head_pos = Robot.us_head_position;
    }

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
      ROS_WARN("Cycle was too long to match desired frequency");
    }
  }

  led_r_on.call(srv);
  led_l_on.call(srv);

  geometry_msgs::Twist vel; // defaults to all 0
  cmd_vel_pub.publish(vel);

  return 0;
}
