#include "rclcpp/logging.hpp"
#include "rclcpp/rate.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class DistanceController : public rclcpp::Node {
public:
    DistanceController() : Node("distance_controller") {

        // Set up waypoints
        distance_goals = {1.0, 2.0, 3.0};
        current_goal_index = 0;

        // Publisher for velocity commands
        vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber for odometry data
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rosbot_xl_base_controller/odom", 10, std::bind(&DistanceController::odomCallback, this, std::placeholders::_1));

        // Make sure the loop execute within specific time
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&DistanceController::controlLoop, this));
    }

private:

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<float> distance_goals;
    int current_goal_index;

    float current_distance;
    float prev_error;
    float integral;

    float kp = 0.5, ki = 0.0125, kd = 0.0025;
    float max_linear_speed = 0.5;
    float goal_tolerance = 0.04; 

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_distance = msg->pose.pose.position.x;
    }

    void controlLoop() {
        
        // Calculate error
        float error = distance_goals[current_goal_index] - current_distance;

        // Update integral term
        integral += error;

        // Calculate derivative term
        float derivative = (error - prev_error) / 0.1;  // loop runs every 0.1 seconds

        // PID control
        float control_signal = kp * error + ki * integral + kd * derivative;

        // Create velocity command
        geometry_msgs::msg::Twist vel;
        vel.linear.x = control_signal;

        // vel.linear.x = std::min(control_signal, max_linear_speed);

        // Publish velocity command
        vel_pub->publish(vel);

        // Check if the goal is reached
        if (std::abs(error) < goal_tolerance) {
            RCLCPP_INFO(rclcpp::get_logger("Distance controller"), "Goal %i is completed" , current_goal_index);
            rclcpp::Rate r(1);
            for(int i=0; i<2;i++){
                vel.linear.x  = 0.0;
                vel_pub->publish(vel);
                r.sleep();
            }
            // Move to the next goal
            current_goal_index++;

            // Reset integral term
            integral = 0.0;

            // Stop the robot if all goals are reached
            if (current_goal_index >= distance_goals.size()) {
                vel.linear.x  = 0.0;
                vel_pub->publish(vel);
                timer_->cancel();
            }
        }

        // Update previous error
        prev_error = error;
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
