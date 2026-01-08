#include "wheel_driver/motor_controller.hpp"

#include <bit>
#include <cmath>
#include <cstring>

uint8_t MotorController::calculate_crc8_maxim(const std::vector<uint8_t>& data){
    uint8_t crc = 0x00; // 初期値

    const uint8_t reflected_polynomial = 0x8C;

    // データのバイトを一つずつ処理
    for(size_t i = 0; i < data.size();i++){
        crc ^= data[i];

        // 各バイトの8ビットを処理
        for(uint8_t bit = 0;bit < 8;bit++){ //data[0] ~ data[8]までの合計9バイト
            if(crc & 0x01){
                crc = (crc >> 1) ^ reflected_polynomial;
            
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

MotorController::MotorController(rclcpp::Logger logger)
    : logger_(logger), serial_port_(io_context_), reading_(false){}

MotorController::~MotorController(){
    if(reading_){
        reading_ = false;
        io_context_.stop();
        if(io_thread_.joinable()){
            io_thread_.join();
        }
        if(serial_port_.is_open()){
            serial_port_.close();
        }
    }
}

bool MotorController::init_port(const std::string& port_name, int baud_rate){
    try {
        this->port_name_ = port_name;
        this->baud_rate_ = baud_rate;
        serial_port_.open(this->port_name_);
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(this->baud_rate_));
        serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_.set_option(boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none));
        serial_port_.set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));
        serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));

        // 非同期読み取り開始
        reading_ = true;
        start_async_read();

        // io_contextを別スレッドで実行
        io_thread_ = std::thread([this]() { io_context_.run(); });

        } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s", port_name.c_str(), e.what());
        return false;
    }
    return true;
}
// setting velocity loop mode

void MotorController::send_velocity_command(uint8_t motor_id, int16_t rpm, bool brake){
uint8_t crc_calculated = calculate_crc8_maxim(packet_data);
    /*
    DATA[0] : motor_id
    DATA[1] : 0x64
    DATA[2] : Velocity high 8 bits
    DATA[3] : Velocity low 8 bits
    DATA[4] : 0
    DATA[5] : 0
    DATA[6] : Acceleration  time
    DATA[7] : Brake
    DATA[8] : 0
    DATA[9] : CRC8-Maxim
    */
    std::vector<uint8_t> data;

    data.push_back(static_cast<uint8_t>(motor_id)); //DATA[0] : motor_id
    data.push_back(0x64); //DATA[1] : 0x64 (Velocity control mode)

    uint16_t rpm_u16 = static_cast<uint16_t>(rpm);
    data.push_back(static_cast<uint8_t>((rpm_u16 >> 8) & 0xFF)); // DATA[2] : Velocity high 8 bits
    data.push_back(static_cast<uint8_t>(rpm_u16 & 0xFF));        // DATA[3] : Velocity low 8 bits
    data.push_back(0x00); // DATA[4] : 0
    data.push_back(0x00); // DATA[5] : 0
    data.push_back(0x00); // DATA[6] : Acceleration time

    // DATA[7] : Brake
    if(brake) data.push_back(0xFF); 
    else data.push_back(0x00);

    data.push_back(0x00); // DATA[8] : 0
    data.push_back(calculate_crc8_maxim(data)); // DATA[9] : CRC8-Maxim

    try{
        boost::asio::write(serial_port_, boost::asio::buffer(data));

        // Feedback待ち
        wait_for_feedback_response(motor_id, 20);
    } catch(const std::exception& e){
        RCLCPP_ERROR(logger_, "Failed to send velocity command to motor %d: %s", motor_id, e.what());
    }
}

void MotorController::wait_for_feedback_response(uint8_t motor_id, int timeout_ms){
    auto start_time = std::chrono::steady_clock::now();

    while(true){
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
        
        // タイムアウトチェック
        if(elapsed.count() > timeout_ms){
            RCLCPP_WARN(logger_, "Timeout waiting for feedback from motor %d", motor_id);
            last_motor_id_ = 0;  // リセット
            return;
        }
        
        // フィードバック受信確認
        if(last_motor_id_ == motor_id){
            last_motor_id_ = 0;  // リセット
            return;
        }
        
        // 少し待つ
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}



void MotorController::send_velocity_commands_sequential(
    const std::vector<std::pair<uint8_t, int16_t>>& commands, bool brake){

        // 全モーターに順次送信
        for(const auto& [motor_id, rpm] : commands){
            send_velocity_command(motor_id, rpm, brake);
        }
}

void MotorController::start_async_read() {
  if (!reading_) return;
  serial_port_.async_read_some(
      boost::asio::buffer(read_buffer_), [this](boost::system::error_code ec, std::size_t length) {
        if (!ec && length > 0) {
          buffer_.insert(buffer_.end(), read_buffer_.begin(), read_buffer_.begin() + length);
          parse_buffer();
        } else {
          if (ec) {
            RCLCPP_DEBUG(logger_, "Read error: %s", ec.message().c_str());
          }
        }
        start_async_read();
      });
}

void MotorController::parse_buffer() {
  // 10バイトパケット単位でチェック
  while (buffer_.size() >= 10) {
    // パケット開始候補を探す(IDは1~4の範囲)
    size_t pos = 0;
    for (; pos <= buffer_.size() - 10; ++pos) {
      if (buffer_[pos] >= 1 && buffer_[pos] <= 4) {
        // CRC8チェック
        std::vector<uint8_t> packet_data(buffer_.begin() + pos, buffer_.begin() + pos + 9);
        uint8_t crc_calculated = calculate_crc8_maxim(packet_data);
        uint8_t crc_received = buffer_[pos + 9];
        if (crc_calculated == crc_received) {
          break;  // 正しいパケット発見
        }
      }
    }

    if (pos > 0) {
      // 不正な先頭バイトがあれば捨てる
      buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
    }

    if (buffer_.size() < 10) break;

    // 10バイトパケット取り出し
    std::vector<uint8_t> packet(buffer_.begin(), buffer_.begin() + 10);

    // パースして保存
    process_feedback_packet(packet);

    // パケット消費
    buffer_.erase(buffer_.begin(), buffer_.begin() + 10);
  }
}

void MotorController::process_feedback_packet(const std::vector<uint8_t>& packet){

    /* Feedback DATA (簡略版)
    DATA[0] : motor_id
    DATA[1] : Mode
    DATA[4] : Velocity high 8 bits
    DATA[5] : Velocity low 8 bits
    */
    if(packet.size() != 10){
        RCLCPP_WARN(logger_, "Invalid packet size: %zu", packet.size());
        return;
    }

    // フィードバック情報を構造体に格納
    MotorFeedback feedback;
    feedback.motor_id = packet[0];  // DATA[0]
    
    // ID確認: 1~4の範囲内かチェック
    if(feedback.motor_id < 1 || feedback.motor_id > 4){
        RCLCPP_WARN(logger_, "Invalid motor ID in feedback: %d (expected 1-4)", feedback.motor_id);
        return;
    }
    
    feedback.mode = packet[1];      // DATA[1]
    feedback.velocity = (static_cast<int16_t>(packet[4]) << 8) | packet[5];  // DATA[4], DATA[5]
    feedback.timestamp = std::chrono::steady_clock::now();
    feedback.valid = true;

    // マップに保存 (motor_idをキーに) - 4台すべて個別に保存
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        motor_feedbacks_[feedback.motor_id] = feedback;
    }

    // ログ出力
    RCLCPP_DEBUG(logger_, "Motor %d: mode=0x%02X, velocity=%d rpm", 
                 feedback.motor_id, feedback.mode, feedback.velocity);

    last_motor_id_ = feedback.motor_id;
    feedback_received_time_ = feedback.timestamp;
}

// 特定のモーターのフィードバックを取得
MotorFeedback MotorController::get_feedback(uint8_t motor_id){
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if(motor_feedbacks_.find(motor_id) != motor_feedbacks_.end()){
        return motor_feedbacks_[motor_id];
    }
    return MotorFeedback();  // 無効なフィードバックを返す
}

// 全モーターのフィードバックを取得
std::map<uint8_t, MotorFeedback> MotorController::get_all_feedbacks(){
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return motor_feedbacks_;
}

// シリアルバッファをクリア
void MotorController::clear_serial_buffer(){
    buffer_.clear();
}

