#ifndef MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include <array>
#include <cmath>
#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace debict
{
    namespace mecanumbot
    {
        namespace hardware
        {

            class Odometry: rclcpp::Node
            {
            public:
            explicit Odometry(size_t velocity_rolling_window_size = 10);

            void init(const rclcpp::Time & time);
            bool update(double front_left_pos, double front_right_pos, double rear_left_pos, double rear_right_pos, const rclcpp::Time & time);
            // bool updateFromVelocity(double front_left_vel, double front_right_vel, double rear_left_vel, double rear_right_vel, const rclcpp::Time & time);
            void updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time & time);
            void resetOdometry();

            double getX() const { return x_; }
            double getY() const { return y_; }
            double getHeading() const { return heading_; }
            double getLinearX() const { return linear_x_; }
            double getLinearY() const { return linear_y_; }
            double getAngular() const { return angular_; }

            void setWheelParams(double wheel_separation_x, double wheel_separation_y, double wheel_radius, int encoder_ticks_per_revolution);
            void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

            private:
            using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

            void integrateRungeKutta2(double linear_x, double linear_y, double angular);
            void integrateExact(double linear_x, double linear_y, double angular);
            void resetAccumulators();
            void Odometry::publishOdometry();
            // Current timestamp:
            rclcpp::Time timestamp_;

            // Number of holes of the encoder:
            int encoder_ticks_per_revolution_;

            // Current pose:
            double x_;        //   [m]
            double y_;        //   [m]
            double heading_;  // [rad]

            // Current velocity:
            double linear_x_;   //   [m/s]
            double linear_y_;   //   [m/s]
            double angular_;  // [rad/s]

            // Wheel kinematic parameters [m]:
            double wheel_separation_x_;
            double wheel_separation_y_;
            double wheel_radius_;

            // Previous wheel position/state [rad]:
            double front_left_wheel_old_pos_;
            double front_right_wheel_old_pos_;
            double rear_left_wheel_old_pos_;
            double rear_right_wheel_old_pos_;

            // Publisher
            std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_publisher_ ;

            // Rolling mean accumulators for the linear and angular velocities:
            size_t velocity_rolling_window_size_;
            RollingMeanAccumulator linear_x_accumulator_;
            RollingMeanAccumulator linear_y_accumulator_;
            RollingMeanAccumulator angular_accumulator_;
            };

        }
    }
}

#endif  // MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_