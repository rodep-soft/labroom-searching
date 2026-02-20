#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <vector>
#include <iomanip>
#include <sys/ioctl.h>

int main(){
  // ジョイスティックデバイスを開く
  const char* device = "/dev/input/js0";
  int joy_fd = open(device, O_RDONLY | O_NONBLOCK);
  if(joy_fd < 0){
    std::cerr << "デバイス認識失敗: " << device << std::endl;
    return 1;
  }

  // ボタンと軸の数を取得
  __u8 num_axes, num_buttons;
  if(ioctl(joy_fd, JSIOCGAXES, &num_axes) < 0){
    std::cerr << "軸数の取得失敗" << std::endl;
    close(joy_fd);
    return 1;
  }
  if(ioctl(joy_fd, JSIOCGBUTTONS, &num_buttons) < 0){
    std::cerr << "ボタン数の取得失敗" << std::endl;
    close(joy_fd);
    return 1;
  }
  
  // 状態・値を保存するベクトル
  std::vector<int> axes_values(num_axes, 0);
  std::vector<int> button_values(num_buttons, 0);

  struct js_event e;

  while(true){
    bool updated = false;

    while(read(joy_fd, &e, sizeof(e)) > 0){
      // 初期化イベントはスキップ
      if(e.type & JS_EVENT_INIT) continue;

      updated = true;

      // 値をベクトルに格納
      if(e.type & JS_EVENT_BUTTON){
        if(e.number < num_buttons){
          button_values[e.number] = e.value;
        } else {
          std::cerr << "警告: 不正なボタン番号 " << (int)e.number << std::endl;
        }
      } else if(e.type & JS_EVENT_AXIS) {
        if(e.number < num_axes){
          axes_values[e.number] = e.value;

          // int nomalized_value = static_cast<int>(
          //     (static_cast<double>(e.value) / 32767.0));
          // axes_values[e.number] = nomalized_value;
        } else {
          std::cerr << "警告: 不正な軸番号 " << (int)e.number << std::endl;
        }
      }
    }

    // 値を表示
    if(updated){
      //topic echo風に表示
      std::cout << "\033[H\033[J"; 
      std::cout << "--- Joystick State (ros2 topic echo style) ---" << std::endl;

      std::cout << "Axes:   [";
      for (int i = 0; i < num_axes; ++i) {
        std::cout << std::setw(6) << axes_values[i] << (i == num_axes - 1 ? "" : ", ");
      }
      std::cout << "]" << std::endl;

      std::cout << "Buttons:[";
      for (int i = 0; i < num_buttons; ++i) {
        std::cout << button_values[i] << (i == num_buttons - 1 ? "" : ", ");
      }
      std::cout << "]" << std::endl;
    }

    usleep(10000);
  }

  close(joy_fd);
  return 0;
}
