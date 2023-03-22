  class thread
  {
  public:
    typedef __gthread_t native_handle_type;
    struct _Impl_base;
    typedef shared_ptr<_Impl_base> __shared_base_type;


    class id
    {
      native_handle_type _M_thread;

    public:
      id() noexcept : _M_thread() { }

      explicit
      id(native_handle_type __id) : _M_thread(__id) { }

    private:
      friend class thread;
      friend class hash<thread::id>;

      friend bool
      operator==(thread::id __x, thread::id __y) noexcept
      { return __gthread_equal(__x._M_thread, __y._M_thread); }

      friend bool
      operator<(thread::id __x, thread::id __y) noexcept
      { return __x._M_thread < __y._M_thread; }

      template<class _CharT, class _Traits>
 friend basic_ostream<_CharT, _Traits>&
 operator<<(basic_ostream<_CharT, _Traits>& __out, thread::id __id);
    };



    struct _Impl_base
    {
      __shared_base_type _M_this_ptr;

      inline virtual ~_Impl_base();

      virtual void _M_run() = 0;
    };

    template<typename _Callable>
      struct _Impl : public _Impl_base
      {
 _Callable _M_func;

 _Impl(_Callable&& __f) : _M_func(std::forward<_Callable>(__f))
 { }

 void
 _M_run() { _M_func(); }
      };

  private:
    id _M_id;

  public:
    thread() noexcept = default;


    thread(thread&) = delete;
    thread(const thread&) = delete;

    thread(thread&& __t) noexcept
    { swap(__t); }

    template<typename _Callable, typename... _Args>
      explicit
      thread(_Callable&& __f, _Args&&... __args)
      {

 __asm ("" : : "r" (&pthread_create));

        _M_start_thread(_M_make_routine(std::__bind_simple(
                std::forward<_Callable>(__f),
                std::forward<_Args>(__args)...)));
      }

    ~thread()
    {
      if (joinable())
 std::terminate();
    }

    thread& operator=(const thread&) = delete;

    thread& operator=(thread&& __t) noexcept
    {
      if (joinable())
 std::terminate();
      swap(__t);
      return *this;
    }

    void
    swap(thread& __t) noexcept
    { std::swap(_M_id, __t._M_id); }

    bool
    joinable() const noexcept
    { return !(_M_id == id()); }

    void
    join();

    void
    detach();

    thread::id
    get_id() const noexcept
    { return _M_id; }



    native_handle_type
    native_handle()
    { return _M_id._M_thread; }


    static unsigned int
    hardware_concurrency() noexcept;

  private:
    void
    _M_start_thread(__shared_base_type);

    template<typename _Callable>
      shared_ptr<_Impl<_Callable>>
      _M_make_routine(_Callable&& __f)
      {

 return std::make_shared<_Impl<_Callable>>(std::forward<_Callable>(__f));
      }
  };

  inline thread::_Impl_base::~_Impl_base() = default;

  inline void
  swap(thread& __x, thread& __y) noexcept
  { __x.swap(__y); }

  inline bool
  operator!=(thread::id __x, thread::id __y) noexcept
  { return !(__x == __y); }

  inline bool
  operator<=(thread::id __x, thread::id __y) noexcept
  { return !(__y < __x); }

  inline bool
  operator>(thread::id __x, thread::id __y) noexcept
  { return __y < __x; }

  inline bool
  operator>=(thread::id __x, thread::id __y) noexcept
  { return !(__x < __y); }



  template<>
    struct hash<thread::id>
    : public __hash_base<size_t, thread::id>
    {
      size_t
      operator()(const thread::id& __id) const noexcept
      { return std::_Hash_impl::hash(__id._M_thread); }
    };

  template<class _CharT, class _Traits>
    inline basic_ostream<_CharT, _Traits>&
    operator<<(basic_ostream<_CharT, _Traits>& __out, thread::id __id)
    {
      if (__id == thread::id())
 return __out << "thread::id of a non-executing thread";
      else
 return __out << __id._M_thread;
    }







  namespace this_thread
  {
 


    inline thread::id
    get_id() noexcept { return thread::id(__gthread_self()); }


    inline void
    yield() noexcept
    {

      __gthread_yield();

    }

    void
    __sleep_for(chrono::seconds, chrono::nanoseconds);


    template<typename _Rep, typename _Period>
      inline void
      sleep_for(const chrono::duration<_Rep, _Period>& __rtime)
      {
 auto __s = chrono::duration_cast<chrono::seconds>(__rtime);
 auto __ns = chrono::duration_cast<chrono::nanoseconds>(__rtime - __s);

 __gthread_time_t __ts =
   {
     static_cast<std::time_t>(__s.count()),
     static_cast<long>(__ns.count())
   };
 ::nanosleep(&__ts, 0);



      }


    template<typename _Clock, typename _Duration>
      inline void
      sleep_until(const chrono::time_point<_Clock, _Duration>& __atime)
      { sleep_for(__atime - _Clock::now()); }

 
  }



}
# 12 "src/algo/src/algo_node.cpp" 2
# 64 "src/algo/src/algo_node.cpp"
# 1 "src/algo/src/decision_modes.hpp" 1
# 1 "src/algo/src/enum_functions.hpp" 1
# 2 "src/algo/src/decision_modes.hpp" 2

enum tagrobot_decision_mode_t
{
    LINE_FOLLOWING,
    LINE_ACQUISITION,
    DEPTH_SENSING,
    ROTATION,
}
;
# 65 "src/algo/src/algo_node.cpp" 2
# 1 "src/algo/src/robot_actions.hpp" 1
# 1 "src/algo/src/enum_functions.hpp" 1
# 2 "src/algo/src/robot_actions.hpp" 2

enum tagrobot_actions_t
{
    ADVANCING,
    TURNING,
    US_SENSING,

    ARBITRARY,

}
;
# 66 "src/algo/src/algo_node.cpp" 2
# 1 "src/algo/src/right_angle_directions.hpp" 1
# 1 "src/algo/src/enum_functions.hpp" 1
# 2 "src/algo/src/right_angle_directions.hpp" 2

enum tagright_angle_directions_t
{
    NONE,
    LEFT,
    RIGHT,
}
;
# 67 "src/algo/src/algo_node.cpp" 2
# 1 "src/algo/src/us_head_positions.hpp" 1
# 1 "src/algo/src/enum_functions.hpp" 1
# 2 "src/algo/src/us_head_positions.hpp" 2

enum tagus_head_positions_t
{
    US_ZERO,
    TOTO,
    US_RIGHT,
    US_LEFT,
}
;
# 68 "src/algo/src/algo_node.cpp" 2


# 1 "src/algo/src/decision_modes.hpp" 1
# 1 "src/algo/src/enum_functions.hpp" 1
# 2 "src/algo/src/decision_modes.hpp" 2

const char *get_string_robot_decision_mode_t(enum tagrobot_decision_mode_t index) { switch (index) {
{
    case LINE_FOLLOWING: return "LINE_FOLLOWING"; break;
    case LINE_ACQUISITION: return "LINE_ACQUISITION"; break;
    case DEPTH_SENSING: return "DEPTH_SENSING"; break;
    case ROTATION: return "ROTATION"; break;
}
default: return "Unknown value"; } } ;
# 71 "src/algo/src/algo_node.cpp" 2
# 1 "src/algo/src/robot_actions.hpp" 1
# 1 "src/algo/src/enum_functions.hpp" 1
# 2 "src/algo/src/robot_actions.hpp" 2

const char *get_string_robot_actions_t(enum tagrobot_actions_t index) { switch (index) {
{
    case ADVANCING: return "ADVANCING"; break;
    case TURNING: return "TURNING"; break;
    case US_SENSING: return "US_SENSING"; break;

    case ARBITRARY: return "ARBITRARY"; break;

}
default: return "Unknown value"; } } ;
# 72 "src/algo/src/algo_node.cpp" 2
# 1 "src/algo/src/right_angle_directions.hpp" 1
# 1 "src/algo/src/enum_functions.hpp" 1
# 2 "src/algo/src/right_angle_directions.hpp" 2

const char *get_string_right_angle_directions_t(enum tagright_angle_directions_t index) { switch (index) {
{
    case NONE: return "NONE"; break;
    case LEFT: return "LEFT"; break;
    case RIGHT: return "RIGHT"; break;
}
default: return "Unknown value"; } } ;
# 73 "src/algo/src/algo_node.cpp" 2
# 1 "src/algo/src/us_head_positions.hpp" 1
# 1 "src/algo/src/enum_functions.hpp" 1
# 2 "src/algo/src/us_head_positions.hpp" 2

const char *get_string_us_head_positions_t(enum tagus_head_positions_t index) { switch (index) {
{
    case US_ZERO: return "US_ZERO"; break;
    case TOTO: return "TOTO"; break;
    case US_RIGHT: return "US_RIGHT"; break;
    case US_LEFT: return "US_LEFT"; break;
}
default: return "Unknown value"; } } ;
# 74 "src/algo/src/algo_node.cpp" 2



# 1 "src/algo/src/us_head_positions.hpp" 1
# 1 "src/algo/src/enum_functions.hpp" 1
# 2 "src/algo/src/us_head_positions.hpp" 2

int16_t get_value_us_head_positions_t(enum tagus_head_positions_t index) { switch (index) {
{
    case US_ZERO: *ret = 96; break;
    static_assert(false, "TOTO" " is does not have any associated values, use DECL_ENUM_VALUE instead")
    case US_RIGHT: *ret = 0; break;
    case US_LEFT: *ret = 255; break;
}
} return ret; } ;
# 78 "src/algo/src/algo_node.cpp" 2


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

    cv.notify_one();
  }
  inline void wait()
  {
    std::unique_lock<std::mutex> lock(mtx);
    while (count == 0)
    {

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

int16_t line_follower[5];
float line_follower_sidemost = 0, line_follower_side = 0;
float turn_rate_turn_mode = 0;

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
# 192 "src/algo/src/algo_node.cpp"
bool get_l_r_encoders(encoder_callback_t *enc_l, encoder_callback_t *enc_r, float *speed_r, float *speed_l)
{
  enc_l->mutex.lock();
  enc_r->mutex.lock();

  const float abs_diff = std::abs(Robot.linear_speed) - std::abs(Robot.angular_speed);
  const float diff = Robot.linear_speed - Robot.angular_speed;


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

  do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 241, __PRETTY_FUNCTION__, "Lwheel msg: %d", msg->data); } } while(0);

  encoder_callback(&encoder_l, msg, last);

  last = msg->data;
}

void rwheel_callback(const std_msgs::Int16::ConstPtr &msg)
{
  static int16_t last = msg->data;

  do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 252, __PRETTY_FUNCTION__, "Rwheel msg: %d", msg->data); } } while(0);

  encoder_callback(&encoder_r, msg, last);

  last = msg->data;
}

void line_follower_callback(const std_msgs::Int16MultiArray::ConstPtr &msg)
{



  bool discarded = false;
  for (int ii = 0; ii < 5; ++ii)
  {
    int16_t value = msg->data[ii];
    do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 268, __PRETTY_FUNCTION__, "Line Follower @%d: %d", ii, value); } } while(0);
    if (value < 0 || value > 1500)
    {
      do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Warn); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Warn),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Warn); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 271, __PRETTY_FUNCTION__, "Discarded values"); } } while(0);
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
  do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 289, __PRETTY_FUNCTION__, "Ultrasonic depth sensed: %d", msg->data); } } while(0);
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
  do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 315, __PRETTY_FUNCTION__, "[PID][%- 15s] setpoint: % 8.4f, error: % 8.4f, p: % 8.4f, i: % 8.5f, d: % 8.4f -> control: % 8.4f", pid->control_type.c_str(), setpoint, error, p_term, i_term, d_term, control_signal); } } while(0);

  pid->last_error = error;

  return control_signal;
}

inline float get_linear_speed_from_ticks(const float ticks)
{
  return ticks * (2 * 3.14159265358979323846 / 18) * 62.5 / 2.0;
}

inline float get_angular_speed(const float linear_speed_r, const float linear_speed_l)
{
  return (linear_speed_r - linear_speed_l) / (48.75 * 2);
}

inline float get_linear_speed(const float linear_speed_r, const float linear_speed_l)
{
  return 1e-3 * (linear_speed_r + linear_speed_l) / 2.0;
}

inline float get_angular_position(const float linear_speed_r, const float linear_speed_l, float last_angular_position)
{
  static auto last_chrono = std::chrono::steady_clock::now();
  auto current_chrono = std::chrono::steady_clock::now();

  const float ret = last_angular_position + 2.0 * get_linear_speed(linear_speed_r, linear_speed_l) / (48.75 * 2 * 1e-3) * std::chrono::duration<float>(current_chrono - last_chrono).count();
  last_chrono = current_chrono;
  return ret;
}

inline float get_radius_from_icc(const float linear_speed_r, const float linear_speed_l)
{
  if (linear_speed_l == linear_speed_r)
  {
    return std::numeric_limits<float>::max();
  }

  return (48.75 * 2 * 1e-3) / 2.0 * (linear_speed_r + linear_speed_l) / (linear_speed_r - linear_speed_l);
}

float line_follower_sum(const int16_t local_line_follower[5], const float leftmost, const float left, const float middle, const float right, const float rightmost)
{
  float sum = 0.0;





  sum += (local_line_follower[0] > 900 ? 1.0 : 0.0) * rightmost;
  sum += (local_line_follower[1] > 900 ? 1.0 : 0.0) * right;
  sum += (local_line_follower[2] > 900 ? 1.0 : 0.0) * middle;
  sum += (local_line_follower[3] > 900 ? 1.0 : 0.0) * left;
  sum += (local_line_follower[4] > 900 ? 1.0 : 0.0) * leftmost;

  return sum;
}

int16_t angle_to_depth_sensor_command(float angle)
{
  angle = std::min(std::max((float)-1.57079632679489661923, angle), (float)1.57079632679489661923);
  return (int16_t)(2 * 255 * angle / 3.14159265358979323846);
}

bool is_depth_sensor_state_detected()
{
  depth.mutex.lock();
  const int16_t local_depth = depth.depth;
  depth.mutex.unlock();
  return local_depth > 20;
}

void process_line_following(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{
  if (is_depth_sensor_state_detected())
  {
    *mode = tagrobot_decision_mode_t::DEPTH_SENSING;
    return;
  }

  static const float ANGLE = 2 * 3.14159265358979323846;
  static tagright_angle_directions_t right_angle_turning = tagright_angle_directions_t::NONE;

  int16_t local_line_follower[5];
  mutex_line_follower.lock();
  std::copy(std::begin(line_follower), std::end(line_follower), std::begin(local_line_follower));
  mutex_line_follower.unlock();

  for (int ii = 0; ii < 5; ++ii)
  {
    do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 406, __PRETTY_FUNCTION__, "Line Follower activation @%d: %d", ii, local_line_follower[ii] > 900); } } while(0);
  }

  const float right_angle = line_follower_sum(local_line_follower,
                                              2,
                                              2,
                                              1,
                                              2,
                                              2);

  if (right_angle_turning > tagright_angle_directions_t::NONE
      && std::abs(Robot.angular_position) > 1 / 4 * ANGLE)
  {
    const float side_reached = line_follower_sum(local_line_follower,
                                                 1,
                                                 1,
                                                 1,
                                                 1,
                                                 1);

    if (side_reached > 0)
    {
      right_angle_turning = tagright_angle_directions_t::NONE;
    }
    do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 430, __PRETTY_FUNCTION__, "Right angle keep turning: %s", get_string_right_angle_directions_t(right_angle_turning)); } } while(0);
  }

  if (right_angle == 5 || right_angle == 7)
  {
    const float side = line_follower_sum(local_line_follower,
                                         1,
                                         0,
                                         0,
                                         0,
                                         -1);
    const tagright_angle_directions_t right_angle_turning_temp = side > 0 ? tagright_angle_directions_t::LEFT : tagright_angle_directions_t::RIGHT;
    if (right_angle_turning_temp != right_angle_turning)
    {

      Robot.angular_position = 0;

      right_angle_turning = right_angle_turning_temp;

      do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 449, __PRETTY_FUNCTION__, "Right angle detected, now turning: %s", get_string_right_angle_directions_t(right_angle_turning)); } } while(0);
    }
  }
  else if (right_angle_turning == tagright_angle_directions_t::NONE && right_angle == 0)
  {

    *mode = tagrobot_decision_mode_t::LINE_ACQUISITION;
    return;
  }

  switch (right_angle_turning)
  {
  case tagright_angle_directions_t::LEFT:
    do { controls->action = tagrobot_actions_t::TURNING; controls->order.turning_position = ANGLE; } while (0);
    return;
  case tagright_angle_directions_t::RIGHT:
    do { controls->action = tagrobot_actions_t::TURNING; controls->order.turning_position = -ANGLE; } while (0);
    return;
  }

  const float angle_of_line = line_follower_sum(local_line_follower,
                                                -line_follower_sidemost,
                                                -line_follower_side,
                                                0,
                                                line_follower_side,
                                                line_follower_sidemost);

  const float linear_speed = std::max(0.15f, line_follower_sum(local_line_follower,
                                                               -0.5,
                                                               -0.5 / 2,
                                                               0.5,
                                                               -0.5 / 2,
                                                               -0.5));

  do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 483, __PRETTY_FUNCTION__, "Wanting to turn: % 8.4f", angle_of_line); } } while(0);
  do { controls->action = tagrobot_actions_t::ADVANCING; controls->order.advance_speed.linear = linear_speed; controls->order.advance_speed.angular = angle_of_line; } while (0);
  return;
}

void process_depth_sensing(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{
  depth.mutex.lock();
  int16_t local_depth = depth.depth;
  depth.mutex.unlock();

  if (Robot.us_head_position == 0)
  {

    do { controls->action = tagrobot_actions_t::US_SENSING; controls->order.us_head_position = -1.57079632679489661923; } while (0);
    return;
  }
  else if (Robot.us_head_position >= 1.57079632679489661923)
  {
    do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Warn); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Warn),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Warn); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 502, __PRETTY_FUNCTION__, "No openings found, retrying"); } } while(0);
    do { controls->action = tagrobot_actions_t::US_SENSING; controls->order.us_head_position = -1.57079632679489661923; } while (0);
    return;
  }

  if (local_depth > 40)
  {

    Robot.angular_position = 0;
    do { controls->action = tagrobot_actions_t::TURNING; controls->order.turning_position = Robot.us_head_position; } while (0);
    *mode = tagrobot_decision_mode_t::LINE_ACQUISITION;
    return;
  }

  do { controls->action = tagrobot_actions_t::US_SENSING; controls->order.us_head_position = Robot.angular_position + 0.1; } while (0);
}

void process_line_acquisition(tagrobot_decision_mode_t *mode, robot_controls_t *controls)
{
  int16_t local_line_follower[5];

  mutex_line_follower.lock();
  std::copy(std::begin(line_follower), std::end(line_follower), std::begin(local_line_follower));
  mutex_line_follower.unlock();

  if (line_follower_sum(local_line_follower, 1, 1, 1, 1, 1) > 0.0f)
  {
    *mode = tagrobot_decision_mode_t::LINE_FOLLOWING;
    return;
  }
  do { controls->action = tagrobot_actions_t::ADVANCING; controls->order.advance_speed.linear = 0.5; controls->order.advance_speed.angular = 0; } while (0);
}

robot_controls_t decision_state_machine(tagrobot_decision_mode_t *mode)
{
  do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 537, __PRETTY_FUNCTION__, "Processing decision: %s", get_string_robot_decision_mode_t(*mode)); } } while(0);
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
    process_line_acquisition(mode, &ret);
    break;
  default:
    do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Error); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Error),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Error); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 551, __PRETTY_FUNCTION__, "State not implemented"); } } while(0);
  }
  return ret;
}

int main(int argc, char **argv)
{
  if (argc != 14)
  {
    do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Error); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Error),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Error); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 560, __PRETTY_FUNCTION__, "incorrect number of parameters. Got %d", argc); } } while(0);
    return 1;
  }

  const bool finite_cyle = atoi(argv[1]) > 0;
  int running_iterations = atoi(argv[1]) * 10;

  do { pid_angular_linear.kp = atof(argv[2]); pid_angular_linear.ki = atof(argv[3]); pid_angular_linear.kd = atof(argv[4]); pid_angular_linear.control_type = "pid_angular_linear"; pid_angular_linear.period = 1.0 / 10; pid_angular_linear.max_control = 1.0f; pid_angular_linear.min_control = -1.0f; } while (0);
  do { pid_linear.kp = atof(argv[5]); pid_linear.ki = atof(argv[6]); pid_linear.kd = atof(argv[7]); pid_linear.control_type = "pid_linear"; pid_linear.period = 1.0 / 10; pid_linear.max_control = 0.8f; pid_linear.min_control = 0.0f; } while (0);
  do { pid_angular.kp = atof(argv[8]); pid_angular.ki = atof(argv[9]); pid_angular.kd = atof(argv[10]); pid_angular.control_type = "pid_angular"; pid_angular.period = 1.0 / 10; pid_angular.max_control = 0.3f; pid_angular.min_control = -0.3f; } while (0);
  line_follower_sidemost = atof(argv[11]);
  line_follower_side = atof(argv[12]);
  turn_rate_turn_mode = atof(argv[13]);

  ros::init(argc, argv, "algo_node");

  ros::NodeHandle n;


  ros::AsyncSpinner spinner(0);
  spinner.start();


  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Publisher cmd_us_pub = n.advertise<std_msgs::Int16>("/cmd_us", 1);

  ros::Subscriber lwheel_sub = n.subscribe("/lwheel", 1, lwheel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber rwheel_sub = n.subscribe("/rwheel", 1, rwheel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber line_follower_sub = n.subscribe("/line_follower", 1, line_follower_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber us_sub = n.subscribe("/us_dist", 1, ultrasonic_callback, ros::TransportHints().tcpNoDelay());

  ros::Rate loop_rate(10);

  ros::ServiceClient led_r_on = n.serviceClient<ropigo::SimpleWrite>("/led_on_right");
  ros::ServiceClient led_r_off = n.serviceClient<ropigo::SimpleWrite>("/led_off_right");
  ros::ServiceClient led_l_on = n.serviceClient<ropigo::SimpleWrite>("/led_on_left");
  ros::ServiceClient led_l_off = n.serviceClient<ropigo::SimpleWrite>("/led_off_left");
  ropigo::SimpleWrite srv;
  if (!led_r_off.call(srv) || !led_l_off.call(srv))
  {
    do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Error); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Error),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Error); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 600, __PRETTY_FUNCTION__, "leds off"); } } while(0);
    return 1;
  }



  semaphore_wait_first_for_encoders.wait();
  semaphore_wait_first_for_encoders.wait();




  tagrobot_decision_mode_t robot_decision = tagrobot_decision_mode_t::LINE_FOLLOWING, last_decision = robot_decision;

  while (ros::ok() && (!finite_cyle || running_iterations-- > 0))
  {
    float speed_ticks_l, speed_ticks_r;

    const bool new_enc_value = get_l_r_encoders(&encoder_l, &encoder_r, &speed_ticks_r, &speed_ticks_l);
    const float speed_linear_r = get_linear_speed_from_ticks(speed_ticks_r);
    const float speed_linear_l = get_linear_speed_from_ticks(speed_ticks_l);
    Robot.angular_position = get_angular_position(speed_linear_r, speed_linear_l, Robot.angular_position);




    robot_controls_t robot_control;
    robot_control.action = ARBITRARY;



    switch (robot_control.action)
    {
    case tagrobot_actions_t::ADVANCING:
      Robot.angular_speed = compute_pid(&pid_angular_linear, 0, robot_control.order.advance_speed.angular);
      Robot.linear_speed = robot_control.order.advance_speed.linear;
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

    case tagrobot_actions_t::ARBITRARY:
    {
      Robot.us_head_position = turn_rate_turn_mode;




    }
    break;

    }


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

    do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 673, __PRETTY_FUNCTION__, "Sensed wheel speeds: R: % 10.5f, L: % 10.5f", speed_linear_r, speed_linear_l); } } while(0);
    do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 674, __PRETTY_FUNCTION__, "Chosen action is: %s", get_string_robot_actions_t(robot_control.action)); } } while(0);

    if (robot_decision != last_decision)
    {
      do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 678, __PRETTY_FUNCTION__, "New decision has been taken: %s", get_string_robot_decision_mode_t(robot_decision)); } } while(0);
      last_decision = robot_decision;
    }

    do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Info); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Info),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Info); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 682, __PRETTY_FUNCTION__, "Controls: linear_speed: % 10.5f, angular_speed: % 10.5f, angular_position: % 10.5f, head position: % 10.5f", Robot.linear_speed, Robot.angular_speed, Robot.angular_position, Robot.us_head_position); } } while(0);

    geometry_msgs::Twist vel;
    vel.linear.x = Robot.linear_speed;
    vel.angular.z = Robot.angular_speed;
    cmd_vel_pub.publish(vel);

    std_msgs::Int16 us_pos;
    us_pos.data = angle_to_depth_sensor_command(Robot.us_head_position);
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




    const bool met = loop_rate.sleep();

    if (!met)
    {
      do { do { if (__builtin_expect((!::ros::console::g_initialized),0)) { ::ros::console::initialize(); } } while(0); static ::ros::console::LogLocation __rosconsole_define_location__loc = {false, false, ::ros::console::levels::Count, 0}; if (__builtin_expect((!__rosconsole_define_location__loc.initialized_),0)) { initializeLogLocation(&__rosconsole_define_location__loc, "ros" "." "unknown_package", ::ros::console::levels::Warn); } if (__builtin_expect((__rosconsole_define_location__loc.level_ != ::ros::console::levels::Warn),0)) { setLogLocationLevel(&__rosconsole_define_location__loc, ::ros::console::levels::Warn); checkLogLocationEnabled(&__rosconsole_define_location__loc); } bool __rosconsole_define_location__enabled = __rosconsole_define_location__loc.logger_enabled_ && (true);; if (__builtin_expect((__rosconsole_define_location__enabled),0)) { ::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, "src/algo/src/algo_node.cpp", 722, __PRETTY_FUNCTION__, "Cycle was too long to match desired frequency"); } } while(0);
    }
  }

  led_r_on.call(srv);
  led_l_on.call(srv);

  geometry_msgs::Twist vel;
  cmd_vel_pub.publish(vel);

  return 0;
}
