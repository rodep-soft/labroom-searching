#ROS WORK SPACE

それぞれのコードをこれ以下に書いて 


ros2 pkg create --build-type ament_cmake <パッケージ名>  
上のコマンドで
  <パッケージ名>/
├── CMakeLists.txt         # CMakeビルド設定ファイル
├── package.xml            # パッケージのメタ情報（依存関係など）
├── include/<パッケージ名>/  # ヘッダファイル用ディレクトリ（空）
├── src/                   # ソースコード用ディレクトリ（空）

ができるはず



https://docs.ros.org/en/rolling/Installation.html
英語だけど公式を参考に
