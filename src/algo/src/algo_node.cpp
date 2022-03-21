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

#define LINE_FOLLOWER_LENGTH 5
#define IR_THREASHOLD 400

#define MAX_LINEAR_SPEED 2.0f
#define MIN_LINEAR_SPEED -2.0f

#define MAX_ANGULAR_LINEAR_SPEED 0.3f
#define MIN_ANGULAR_LINEAR_SPEED -0.3f

#define MAX_ANGULAR_SPEED 1.0f
#define MIN_ANGULAR_SPEED -1.0f

#define FREQUENCY 10 // Hz

// STATICS

#define WHEEL_DIAMETER 62.5                                  // mm
#define PULSE_BY_ROTATION 18                                 // pulses
#define DISTANCE_WHEEL_TO_CENTER 48.75                       // mm
#define DISTANCE_WHEEL_TO_WHEEL DISTANCE_WHEEL_TO_CENTER * 2 // mm

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

struct PID_t
{
  float kp, ki, kd;
  float max_control, min_control;
  float last_error;
  float total_error;
  float period = 1.0;
  std::string control_type;
} pid_angular_linear, pid_angular, pid_linear;

struct encoder_callback_t
{
  uint16_t init_count = 1;
  std::mutex mutex;
  bool new_value = false;
  float tick_speed;
} encoder_l, encoder_r;

enum robot_actions_t
{
  TURNING,
  ADVANCING
};

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
  enc->mutex.lock();

  enc->tick_speed = msg->data - last;
  enc->new_value = true;

  if (enc->init_count-- > 0)
  {
    semaphore_wait_first_for_encoders.notify();
  }

  enc->mutex.unlock();
}

void lwheelCallback(const std_msgs::Int16::ConstPtr &msg)
{
  static int16_t last = msg->data;

  ROS_INFO("Lwheel msg: %d", msg->data);

  encoder_callback(&encoder_l, msg, last);

  last = msg->data;
}

void rwheelCallback(const std_msgs::Int16::ConstPtr &msg)
{
  static int16_t last = msg->data;

  ROS_INFO("Rwheel msg: %d", msg->data);

  encoder_callback(&encoder_r, msg, last);

  last = msg->data;
}

void lineFollowerCallback(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
  ROS_INFO("Line Follower: [%d]", msg->data[0]);
  mutex_line_follower.lock();
  std::copy(std::begin(msg->data), std::end(msg->data), std::begin(line_follower));
  mutex_line_follower.unlock();
}

bool isIrActivated(int16_t value)
{
  return value > IR_THREASHOLD;
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
  ROS_INFO("[PID][%- 15s] error: % 10.5f, p: % 10.5f, i: % 10.5f, d: % 10.5f ---> control: % 10.5f", pid->control_type.c_str(), error, p_term, i_term, d_term, control_signal);

  pid->last_error = error;

  return control_signal;
}

inline float get_linear_speed_from_ticks(const float ticks)
{
  return ticks * (2 * M_PI / PULSE_BY_ROTATION) * WHEEL_DIAMETER / 2.0;
}

inline float get_angular_speed(const float linear_speed_r, const float linear_speed_l)
{
  return (linear_speed_r - linear_speed_l) / DISTANCE_WHEEL_TO_WHEEL;
}

inline float get_linear_speed(const float linear_speed_r, const float linear_speed_l)
{
  return (linear_speed_r - linear_speed_l) / 2.0;
}

inline float get_radius_from_icc(const float linear_speed_r, const float linear_speed_l)
{
  if (linear_speed_l == linear_speed_r)
  {
    return std::numeric_limits<float>::max();
  }

  return DISTANCE_WHEEL_TO_WHEEL / 2.0 * (linear_speed_r + linear_speed_l) / (linear_speed_r - linear_speed_l);
}

int main(int argc, char **argv)
{
  if (argc != 8)
  {
    ROS_ERROR("incorrect number of parameters. Got %d", argc);
    return 1;
  }

  const bool finite_cyle = atoi(argv[1]) > 0;
  int running_iterations = atoi(argv[1]) * FREQUENCY;

  pid_angular_linear.kp = atof(argv[2]);
  pid_angular_linear.ki = atof(argv[3]);
  pid_angular_linear.kd = atof(argv[4]);
  pid_angular_linear.control_type = "angular&linear speed";

  pid_angular.kp = atof(argv[5]);
  pid_angular.ki = atof(argv[6]);
  pid_angular.kd = atof(argv[7]);
  pid_angular.control_type = "angular speed";

  pid_angular_linear.period = 1.0 / 8.0; // tre period of the encoders
  pid_angular_linear.max_control = MAX_ANGULAR_LINEAR_SPEED;
  pid_angular_linear.min_control = MIN_ANGULAR_LINEAR_SPEED;

  pid_angular.period = 1.0 / 8.0;
  pid_angular.max_control = MAX_ANGULAR_SPEED;
  pid_angular.min_control = MIN_ANGULAR_SPEED;

  ros::init(argc, argv, "algo_node");

  ros::NodeHandle n;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Subscriber lwheel_sub = n.subscribe("/lwheel", 1, lwheelCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber rwheel_sub = n.subscribe("/rwheel", 1, rwheelCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber line_follower_sub = n.subscribe("/line_follower", 1, lineFollowerCallback, ros::TransportHints().tcpNoDelay());

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

  int16_t local_line_follower[LINE_FOLLOWER_LENGTH];

  // wait for both encoders to have their first value
  semaphore_wait_first_for_encoders.wait();
  semaphore_wait_first_for_encoders.wait();

  float angular_speed = 0, linear_speed = 0;

  robot_actions_t current_action = robot_actions_t::ADVANCING;

  while (ros::ok() && (!finite_cyle || running_iterations-- > 0))
  {
    // mutex_line_follower.lock();
    // std::copy(std::begin(line_follower), std::end(line_follower), std::begin(local_line_follower));
    // mutex_line_follower.unlock();

    float speed_ticks_l, speed_ticks_r;

    const bool new_enc_value = get_l_r_encoders(&encoder_l, &encoder_r, &speed_ticks_r, &speed_ticks_l);
    if (new_enc_value)
    {
      PID_t *chosen_pid;

      const float speed_linear_r = get_linear_speed_from_ticks(speed_ticks_r);
      const float speed_linear_l = get_linear_speed_from_ticks(speed_ticks_l);

      switch (current_action)
      {
      case robot_actions_t::ADVANCING:
        chosen_pid = &pid_angular_linear;
        angular_speed = compute_pid(chosen_pid, 0, get_angular_speed(speed_linear_r, speed_linear_l));
        break;
      case robot_actions_t::TURNING:
        chosen_pid = &pid_angular;
        break;
      }

      ROS_INFO("wheel speeds: R: % 10.5f, L: % 10.5f", speed_linear_r, speed_linear_l);
    }

    // const float linear_distance_from_last_sensed = linear_distance_from_differential_wheel_distances(
    //     pulses_to_linear_distance(lwheel - last_lwheel),
    //     pulses_to_linear_distance(rwheel - last_rwheel));

    // const float diff_linear_speed = linear_distance_from_last_sensed * FREQUENCY * 1e-3;

    // ROS_INFO("linear diff speed: %f", diff_linear_speed);

    // float x = 0.15 * isIrActivated(local_line_follower[2])                                             // center
    //           + 0.1 * (isIrActivated(local_line_follower[1]) + isIrActivated(local_line_follower[3]))  // internals left & right
    //           + 0.1 * (isIrActivated(local_line_follower[0]) + isIrActivated(local_line_follower[4])); // external left & right

    // ROS_INFO("x speed: %f", x);

    // float theta = 0 * isIrActivated(local_line_follower[2])                                                //
    //               + 0.1 * (-isIrActivated(local_line_follower[3]) + isIrActivated(local_line_follower[1])) //
    //               + 0.15 * (-isIrActivated(local_line_follower[4]) + isIrActivated(local_line_follower[0]));

    // ROS_INFO("theta diff: %f", theta);

    // float linear_speed = compute_pid(&pid_linear, 0.5, diff_linear_speed);

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    // accelerate(0.5, acceleration_time, &linear_speed, &time_left);
    geometry_msgs::Twist vel;
    vel.linear.x = 0.5;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = angular_speed;

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    cmd_vel_pub.publish(vel);

    std::thread([&] {
      if (angular_speed > 0)
      {
        led_l_on.call(srv);
      }
      else if (angular_speed < 0)
      {
        led_r_on.call(srv);
      }
    }).detach();

    std::thread([&] {
      if (angular_speed > 0)
      {
        led_r_off.call(srv);
      }
      else if (angular_speed < 0)
      {
        led_l_off.call(srv);
      }
    }).detach();

    const bool met = loop_rate.sleep();

    if (!met)
    {
      ROS_ERROR("Cycle was too long to match desired frequency");
    }
  }

  led_r_on.call(srv);
  led_l_on.call(srv);

  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0;

  cmd_vel_pub.publish(vel);

  return 0;
}
