{
  description = "ROS 2 Jazzy environment (Cached + GUI Support)";

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJGTgbewUGQlTNhdxtaVF8s8jBuWZz/pM=" ];
  };

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
    nixgl.url = "github:nix-community/nixGL";
  };

  outputs = { self, nixpkgs, nix-ros-overlay, flake-utils, nixgl, ... }:
    flake-utils.lib.eachSystem [ "aarch64-linux" "x86_64-linux" ] (system:
      let
        # -------------------------------------------------------------
        # 1. ROS構築用 (混ぜ物なしのピュアな環境)
        # -------------------------------------------------------------
        pkgsRos = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
          config.allowUnfree = true;
        };

        # -------------------------------------------------------------
        # 2. シェル環境用 (nixGLなどの便利ツール入り)
        # -------------------------------------------------------------
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ 
            nix-ros-overlay.overlays.default
            nixgl.overlay 
          ];
          config.allowUnfree = true;
        };

        # ROSパッケージは 'pkgsRos' (ピュアな方) から作る！
        rosDistro = pkgsRos.rosPackages.jazzy;

        myRosWorkspace = rosDistro.buildEnv {
          paths = [
            # --- 基本セット ---
            rosDistro.ros-base
            rosDistro.slam-toolbox
            rosDistro.rviz2
            rosDistro.joy
            rosDistro.demo-nodes-cpp
            
            # --- 依存関係 ---
            rosDistro.sensor-msgs
            rosDistro.geometry-msgs
            rosDistro.nav-msgs
            rosDistro.std-msgs
            rosDistro.tf2
            rosDistro.tf2-ros
            rosDistro.tf2-geometry-msgs
            rosDistro.eigen3-cmake-module
            
            # --- ビルドシステム ---
            rosDistro.ament-cmake
            rosDistro.ament-cmake-core
            rosDistro.ament-cmake-ros
            rosDistro.ament-cmake-test
            rosDistro.ament-lint-auto
            rosDistro.ament-lint-common
            
            # --- 追加依存 ---
            rosDistro.rosidl-default-generators
            rosDistro.rosidl-default-runtime
            rosDistro.diagnostic-updater
            rosDistro.diagnostic-msgs
            rosDistro.laser-proc
            rosDistro.urg-c
          ];
        };

      in {
        # -------------------------------------------------------------
        # A. 普段の開発用 (nixGLあり / --impure 必須)
        # -------------------------------------------------------------
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
            
            pkgs.nixgl.auto.nixGLDefault # ここにGUI用が入っている

            (pkgs.python3.withPackages (ps: [ 
              ps.gpiozero 
              ps.pip
            ]))
          ];

          shellHook = ''
            export CCACHE_DIR=$HOME/.ccache
            export PATH="${pkgs.ccache}/bin:$PATH"
            
            alias rviz2="nixGL rviz2"
            alias rqt="nixGL rqt"

            if [ ! -f $HOME/.config/colcon/defaults.yaml ]; then
              mkdir -p $HOME/.config/colcon
              echo "build: {args: ['--symlink-install']}" > $HOME/.config/colcon/defaults.yaml
            fi
            echo "--------------------------------------------------------"
            echo "🤖 ROS 2 Jazzy Environment (With GUI Support)"
            echo "--------------------------------------------------------"
          '';
        };

        # -------------------------------------------------------------
        # B. CI用 (nixGLなし / 純粋な環境 / --impure 不要)
        # -------------------------------------------------------------
        # ↓↓↓ ここが今回追加される部分です！ ↓↓↓
        devShells.ci = pkgsRos.mkShell {
          name = "ros2-jazzy-ci";
          
          buildInputs = [
            myRosWorkspace
            pkgsRos.colcon
            pkgsRos.gcc
            pkgsRos.cmake
            pkgsRos.git
            # nixGLは入れない！
          ];
        };
      }
    );
}
