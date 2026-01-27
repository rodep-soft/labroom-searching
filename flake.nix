{
  description = "ROS 2 Jazzy environment (Cached)";

  # キャッシュ設定（ここはさっきのままでOK）
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJGTgbewUGQlTNhdxtaVF8s8jBuWZz/pM=" ];
  };

  inputs = {
    # ROS Overlayをマスターに
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";

    # 【最重要】nixpkgsをオーバーレイのものに強制同期させる
    # これにより "Stable 24.11" ではなく "Unstable" になりますが、
    # ROSのバイナリキャッシュを使うための必須条件です。
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, nix-ros-overlay, flake-utils, ... }:
    flake-utils.lib.eachSystem [ "aarch64-linux" "x86_64-linux" ] (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ 
            nix-ros-overlay.overlays.default
          ];
          config.allowUnfree = true;
        };

        rosDistro = pkgs.rosPackages.jazzy;

        myRosWorkspace = rosDistro.buildEnv {
          paths = [
            rosDistro.ros-base
            rosDistro.slam-toolbox
            rosDistro.rviz2
            rosDistro.joy
            rosDistro.demo-nodes-cpp
            
            rosDistro.sensor-msgs
            rosDistro.geometry-msgs
            rosDistro.nav-msgs
            rosDistro.std-msgs
            rosDistro.tf2
            rosDistro.tf2-ros
            rosDistro.tf2-geometry-msgs
            rosDistro.eigen3-cmake-module
            rosDistro.ament-cmake
          ];
        };

      in {
        devShells.default = pkgs.mkShell {
          name = "ros2-jazzy-env";

          buildInputs = [
            myRosWorkspace
            pkgs.colcon
            pkgs.cmake
            pkgs.gcc
            pkgs.ccache
            pkgs.git
            pkgs.vim
            pkgs.tmux
            pkgs.fzf
            (pkgs.python3.withPackages (ps: [ 
              ps.gpiozero 
              ps.pip
            ]))
          ];

          shellHook = ''
            export CCACHE_DIR=$HOME/.ccache
            export PATH="${pkgs.ccache}/bin:$PATH"
            if [ ! -f $HOME/.config/colcon/defaults.yaml ]; then
              mkdir -p $HOME/.config/colcon
              echo "build: {args: ['--symlink-install']}" > $HOME/.config/colcon/defaults.yaml
            fi
            echo "--------------------------------------------------------"
            echo "🤖 ROS 2 Jazzy Environment Loaded! (Cached Build)"
            echo "--------------------------------------------------------"
          '';
        };
      }
    );
}
