/*
 * Copyright (C) 2023 eSOL Co.,Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE FREEBSD PROJECT ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE FREEBSD PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// C++標準ライブラリ
#include <memory>

// ROSのC++クライアントライブラリ
#include "rclcpp/rclcpp.hpp"
//#include "urg_node2/urg_node2.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
// 文字列のメッセージ型
#include "std_msgs/msg/string.hpp"

// 省略表記
using std::placeholders::_1;

// rclcpp::Nodeを継承したクラスSimpleListenerの宣言
class SimpleListener : public rclcpp::Node
{
public:
  // コンストラクタ
  SimpleListener()
  : Node("simple_listener")   // ノード名をsimple_listenerで初期化
  {
    // subscriberの作成
    // 第一引数はトピック名
    // 第二引数はメッセージのバッファサイズ、
    // 第三引数は受信したときに呼ばれる関数
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::QoS(20), std::bind(&SimpleListener::scan_callback, this, _1)
    );
  }

private:
  // トピックを受信したときに呼ばれるコールバック関数
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
  {
    // ターミナルへの
    RCLCPP_INFO(this->get_logger(), "subscribe: %.2f", msg->ranges[0]);
  }

  // std_msgs::msg::String型のトピックを受信するsubscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // rcl(ros client library)の初期化
  rclcpp::init(argc, argv);

  // ノードを作成し、スピン（実行）
  rclcpp::spin(std::make_shared<SimpleListener>());

  // rclの終了
  rclcpp::shutdown();

  return 0;
}
