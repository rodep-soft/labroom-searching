{
  description = "ROS 2 Jazzy environment (Cached)";

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
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ 
            nix-ros-overlay.overlays.default
            nixgl.overlay
          ];
          config.allowUnfree = true;
        };

        rosDistro = pkgs.rosPackages.jazzy;

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
            
            rosDistro.ament-cmake
            rosDistro.ament-cmake-core  
            #rosDistro.ament-cmake-cpp
            rosDistro.ament-cmake-ros
            rosDistro.ament-cmake-test
            rosDistro.ament-lint-auto
            rosDistro.ament-lint-common
            
            rosDistro.rosidl-default-generators
            rosDistro.rosidl-default-runtime
            rosDistro.diagnostic-updater
            rosDistro.diagnostic-msgs
            rosDistro.laser-proc
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

            pkgs.nixgl.auto.nixGLDefault

            (pkgs.python3.withPackages (ps: [ 
              ps.gpiozero 
              ps.pip
            ]))
          ];

          shellHook = ''

            alias rviz2="nixGL rviz2"

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
