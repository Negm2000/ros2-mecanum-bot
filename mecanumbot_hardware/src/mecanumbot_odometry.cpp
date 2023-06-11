#include "mecanumbot_hardware/mecanumbot_odometry.hpp"


using namespace debict::mecanumbot::hardware;

Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_x_(0.0),
  linear_y_(0.0),
  angular_(0.0),
  wheel_separation_x_(0.0),
  wheel_separation_y_(0.0),
  wheel_radius_(0.0),
  front_left_wheel_old_pos_(0.0),
  front_right_wheel_old_pos_(0.0),
  rear_left_wheel_old_pos_(0.0),
  rear_right_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_x_accumulator_(velocity_rolling_window_size),
  linear_y_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size),
  encoder_ticks_per_revolution_(26)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  timestamp_ = time;
  resetAccumulators();
}

bool Odometry::update(double front_left_pos, double front_right_pos, double rear_left_pos, double rear_right_pos, const rclcpp::Time & time)
{

  const double delta_time = (time - timestamp_).seconds();
  if (std::abs(delta_time) < 1.0e-4) return false;
  timestamp_ = time;  


  const double delta_front_left_wheel = front_left_pos - front_left_wheel_old_pos_;
  const double delta_front_right_wheel = front_right_pos - front_right_wheel_old_pos_;
  const double delta_rear_left_wheel = rear_left_pos - rear_left_wheel_old_pos_;
  const double delta_rear_right_wheel = rear_right_pos - rear_right_wheel_old_pos_;

  front_left_wheel_old_pos_ = front_left_pos;
  front_right_wheel_old_pos_ = front_right_pos;
  rear_left_wheel_old_pos_ = rear_left_pos;
  rear_right_wheel_old_pos_ = rear_right_pos;

  // Get each wheel's change in radians and use that to calculate the change in x, y, and theta

  const double front_left_wheel_delta = delta_front_left_wheel / encoder_ticks_per_revolution_ *  2 * M_PI ;
  const double front_right_wheel_delta = delta_front_right_wheel / encoder_ticks_per_revolution_  * 2 * M_PI  ;
  const double rear_left_wheel_delta = delta_rear_left_wheel / encoder_ticks_per_revolution_ * 2 * M_PI ;
  const double rear_right_wheel_delta = delta_rear_right_wheel / encoder_ticks_per_revolution_ * 2 * M_PI ;


  // Forward kinematics

  double delta_linear_x = wheel_radius_*(front_left_wheel_delta + front_right_wheel_delta + rear_left_wheel_delta + rear_right_wheel_delta) / 4.0;
  double delta_linear_y = wheel_radius_*(-front_left_wheel_delta + front_right_wheel_delta + rear_left_wheel_delta - rear_right_wheel_delta) / 4.0;
  double delta_angular =  wheel_radius_*(-front_left_wheel_delta + front_right_wheel_delta - rear_left_wheel_delta + rear_right_wheel_delta) / (4.0 * (wheel_separation_x_ + wheel_separation_y_));
  
  // Compute linear velocities

  delta_linear_x /= delta_time;
  delta_linear_y /= delta_time;
  delta_angular /= delta_time;

  // Compute linear and angular average velocities

  linear_x_accumulator_.accumulate(delta_linear_x);
  linear_y_accumulator_.accumulate(delta_linear_y);
  angular_accumulator_.accumulate(delta_angular);

  linear_x_ = linear_x_accumulator_.getRollingMean();
  linear_y_ = linear_y_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  // Compute odometric pose

  integrateExact(linear_x_, linear_y_, angular_);


  return true;
}

// bool Odometry::updateFromVelocity(double front_left_vel, double front_right_vel, double rear_left_vel, double rear_right_vel, const rclcpp::Time & time)
// {
//   const double delta_time = (time - timestamp_).seconds();
//   if (std::abs(delta_time) < 1.0e-4) return false;
//   timestamp_ = time;

//   const double front_left_wheel_delta = front_left_vel * wheel_radius_;
//   const double front_right_wheel_delta = front_right_vel * wheel_radius_;
//   const double rear_left_wheel_delta = rear_left_vel * wheel_radius_;
//   const double rear_right_wheel_delta = rear_right_vel * wheel_radius_;


//   double delta_linear_x = (front_left_wheel_delta + front_right_wheel_delta + rear_left_wheel_delta + rear_right_wheel_delta) / 4.0;
//   double delta_linear_y = (-front_left_wheel_delta + front_right_wheel_delta + rear_left_wheel_delta - rear_right_wheel_delta) / 4.0;
//   double delta_angular = (-front_left_wheel_delta +front_right_wheel_delta - rear_left_wheel_delta + rear_right_wheel_delta) / (4.0 * (wheel_separation_x_ + wheel_separation_y_));


//   delta_linear_x /= delta_time;
//   delta_linear_y /= delta_time;
//   delta_angular /= delta_time;

//   linear_x_accumulator_.accumulate(delta_linear_x);
//   linear_y_accumulator_.accumulate(delta_linear_y);
//   angular_accumulator_.accumulate(delta_angular);

//   linear_x_ = linear_x_accumulator_.getRollingMean();
//   linear_y_ = linear_y_accumulator_.getRollingMean();
//   angular_ = angular_accumulator_.getRollingMean();

//   integrateExact(linear_x_, linear_y_, angular_);

//   return true;
// }

void Odometry::updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time & time)
{
  linear_x_ = linear_x;
  linear_y_ = linear_y;
  angular_ = angular;

  const double delta_time = (time - timestamp_).seconds();
  timestamp_ = time;

  integrateExact(linear_x_, linear_y_, angular_);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  resetAccumulators();
}

void Odometry::setWheelParams(double wheel_separation_x, double wheel_separation_y, double wheel_radius, int encoder_ticks_per_revolution)
{
  wheel_separation_x_ = wheel_separation_x;
  wheel_separation_y_ = wheel_separation_y;
  wheel_radius_ = wheel_radius;
  encoder_ticks_per_revolution_ = encoder_ticks_per_revolution;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;
  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear_x, double linear_y, double angular)
{
  const double delta_heading = angular * 0.5;
  const double cos_heading = std::cos(heading_ + delta_heading);
  const double sin_heading = std::sin(heading_ + delta_heading);

  x_ += (linear_x * cos_heading - linear_y * sin_heading) * 0.5;
  y_ += (linear_x * sin_heading + linear_y * cos_heading) * 0.5;
  heading_ += angular;
}

void Odometry::integrateExact(double linear_x, double linear_y, double angular)
{
    if (fabs(angular) < 1e-6)
    {
        integrateRungeKutta2(linear_x, linear_y, angular);
        return;
    }
  const double cos_heading = std::cos(heading_);
  const double sin_heading = std::sin(heading_);

  x_ += (linear_x * cos_heading - linear_y * sin_heading);
  y_ += (linear_x * sin_heading + linear_y * cos_heading);
  heading_ += angular;
  while (heading_ < 0.0) heading_ += 2.0 * M_PI;
  while (heading_ > 2*M_PI) heading_ -= 2.0 * M_PI;
  
}

void Odometry::resetAccumulators()
{
  linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}
