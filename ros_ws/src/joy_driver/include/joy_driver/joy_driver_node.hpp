/*
    IMUとqosについての設定を消したrox_temp_repをほぼまねた
*/


#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msg/msg/string.hpp>

class JoyDriver : public rclcpp::Node{
    public:
        // Constructor
        JoyDriverNode();

        // Public enum for testing
        enum class Mode{
            STOP,
            JOY,
            AUTOMATIC,
        }

        static double applyDeadzone(double value, double threshold = 0.05);
        static double normalizeAngle(double angle);
        double calculatePIDCorrection(double error, double dt, double velocity_factory = 1.0);
        double calculateAngularCorrectionVelocity(double angle_error, double anglar_vel_x, double dt,
                                                 double velocity_factor = 1.0);
        std::string mode_to_string(Mode mode, bool is_gear_down_param);
        double get_angular_velocity(const sensor_msgs::msg::Joy::Joy::SharedPtr& msg);
        void joy_callback(const sensor_msg::msg::joy::ShardPtr msg);
        void keyboad_callback(const std_msgs::msg::String::sharedPtr msg);

    private:
        //default Mode
        Mode mode_ = Mode::STOP;

        // ros2パラメータを宣言、取得する関数
        void declare_parameters();
        void get_parameters();
        
        //Callback functions
        void rpy_callback(const geometry_msgs::msgs::msg::Vector3::SharedPtr msg);
        
        // ROS2 Subscription
        rclcpp::Subscription<sensor_msg::msg::Joy>::SharePtr joy_subscription__;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr rpy_subscriotion_;
        
        // ROS2 Publisher
        rclcpp::Publisher<geomery_msgs::msg::Twist>::SharePtr cmd_vel_pulisher_;
        rclcpp::Publisher<std_msgs::String>::SharePtr mode_publisher_;

        // ROS2 Client
        rclcpp::Client<std_srvs::stv::SetBool>::SharePtr breake_client\;

        // joystick params
        double linear_x_scale_;
        double linear_y_scale_;
        double angular_scale_;
        int linear_x_axis_;
        int linear_y_axis_;
        int angular_axis_;

        // PIDゲインを書いてもいいかもね
        
        // IMU補正用のフィルタとPID制御変数

        //ローパスフィルタ係数

        //積分校のウィンドアップ

        // ==== ros2 params END ====


        // Euler
        double pitch_ = 0.0;
        double roll_ = 0.0;
        double yaw_ = 0.0;

        // Angular velocities from IMU (rad/s)
        double angular_velo_x_ = 0.0;
        double angular_velo_y_ = 0.0;
        double angular_velo_z_ = 0.0;
        double filtered_angular_vel_x_ = 0.0;
        double filtered_angular_vel_z_ = 0.0;

        // ログ用の変数
        double last_yaw_log_ = 0.0;
        double last_yaw_log_time_ = 0.0;

        // member constants
        const double TRIGGER_THRESHOLD = 0.95;

}