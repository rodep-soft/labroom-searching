#pragma once

#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <map>
#include <mutex>

// ros2
#include <rclcpp/rclcpp.hpp>

// モーターフィードバック情報を保存する構造体
struct MotorFeedback {
    uint8_t motor_id;    // DATA[0]
    uint8_t mode;        // DATA[1]
    int16_t velocity;    // DATA[4], DATA[5]
    std::chrono::steady_clock::time_point timestamp;
    bool valid;
    
    MotorFeedback() : motor_id(0), mode(0), velocity(0), valid(false) {}
};

class MotorController
{
    public:
        MotorController(rclcpp::Logger logger);
        ~MotorController();

        bool init_port(const std::string & port_name, int baud_rate);
        
        // Motor control commands functions
        void send_velocity_command(uint8_t motor_id, int16_t rpm, bool brake = false);
        void send_velocity_commands_sequential(const std::vector<std::pair<uint8_t, int16_t>>& commands, bool brake = false);

        // Helper functions
        void clear_serial_buffer();
        void start_async_read();
        void parse_buffer();
        void process_feedback_packet(const std::vector<uint8_t>& packet);
        void wait_for_feedback_response(uint8_t motor_id, int timeout_ms = 20);
        bool wait_for_motor_response(uint8_t motor_id, int timeout_ms = 50);
        bool validate_motor_response(const std::vector<uint8_t>& response, uint8_t expected_id);
        
        // Feedback取得関数
        MotorFeedback get_feedback(uint8_t motor_id);
        std::map<uint8_t, MotorFeedback> get_all_feedbacks();
   

        private:
        std::string port_name_;
        int baud_rate_;
        boost::asio::io_context io_context_;
        boost::asio::serial_port serial_port_;
        rclcpp::Logger logger_;

        uint8_t calculate_crc8_maxim(const std::vector<uint8_t>& data);

        std::vector<uint8_t> buffer_;
        std::array<uint8_t, 64> read_buffer_;
        std::atomic<bool> reading_;
        std::thread io_thread_;

        std::chrono::steady_clock::time_point feedback_received_time_;
        std::atomic<uint8_t> last_motor_id_{0};
        
        // 全モーターのフィードバック情報を保存
        std::map<uint8_t, MotorFeedback> motor_feedbacks_;
        std::mutex feedback_mutex_;
    };




