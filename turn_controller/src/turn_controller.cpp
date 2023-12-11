#include "rclcpp/logging.hpp"
#include "rclcpp/rate.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class TurnController : public rclcpp::Node {
public:
    TurnController() : Node("turn_controller") {

        // Set up waypoints
        points_goals = {
            {1.40, -0.81},
            {3.07, -0.2},
            {5.10, 1.04}  
        };
        current_goal_index = 0;

        // Publisher for velocity commands
        vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber for odometry data
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rosbot_xl_base_controller/odom", 10, std::bind(&TurnController::odomCallback, this, std::placeholders::_1));

        // Make sure the loop execute within specific time
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&TurnController::controlLoop, this));
    }

private:

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::pair<float, float>> points_goals;
    float rad_goals;

    int current_goal_index;

    float current_yaw_rad;
    float current_x_pos;
    float current_y_pos;

    float prev_error;
    float integral;

    float kp = 0.7, ki = 0.0175, kd = 0.0035;
    float goal_tolerance = 0.03; 

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_yaw_rad = euler_degree_transform(msg);
        current_x_pos = msg->pose.pose.position.x;
        current_y_pos = msg->pose.pose.position.y;
    }

    float euler_degree_transform(const nav_msgs::msg::Odometry::SharedPtr msg){
        
        float x = msg->pose.pose.orientation.x;
        float y = msg->pose.pose.orientation.y;
        float z = msg->pose.pose.orientation.z;
        float w = msg->pose.pose.orientation.w; 

        return atan2(2 * (w * z + x * y),1 - 2 * (y * y + z * z));
    }

    float cartesian_transfrom(float x_set, float y_set, float x, float y){
        
        float goad_rad;
        goad_rad = atan2( y_set - y, x_set - x);

        return goad_rad;
    }

    void controlLoop() {
        
        rad_goals = cartesian_transfrom(
            points_goals[current_goal_index].first,
            points_goals[current_goal_index].second , 
                    current_x_pos, current_y_pos);
        // Calculate error
        float error = rad_goals - current_yaw_rad;

        // Update integral term
        integral += error;

        // Calculate derivative term
        float derivative = (error - prev_error) / 0.1;  // loop runs every 0.1 seconds

        // PID control
        float control_signal = kp * error + ki * integral + kd * derivative;

        // Create velocity command
        geometry_msgs::msg::Twist vel;
        // velocity is equal to control signal
        vel.angular.z = control_signal;
        vel_pub->publish(vel);


        // Check if the goal is reached
        if (std::abs(error) < goal_tolerance) {
            RCLCPP_INFO(rclcpp::get_logger("Distance controller"), "Goal %i is completed" , current_goal_index);
            rclcpp::Rate r(1);
            for(int i=0; i<2;i++){
                vel.angular.z  = 0.0;
                vel_pub->publish(vel);
                r.sleep();
            }
            // Move to the next goal
            current_goal_index++;

            // Reset integral term
            integral = 0.0;

            // Stop the robot if all goals are reached
            if (current_goal_index >= points_goals.size()) {
                vel.angular.z = 0.0;
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
    auto node = std::make_shared<TurnController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

