{
  description = "ROS 2 Jazzy environment (Multi-Arch: ARM64 & AMD64)";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nix-ros-overlay.inputs.nixpkgs.follows = "nixpkgs";
    
    #  multi archtecture
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, nix-ros-overlay, flake-utils, ... }:
    flake-utils.lib.eachSystem [ "aarch64-linux" "x86_64-linux" ] (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
          config.allowUnfree = true;
        };

        rosDistro = pkgs.rosPackages.jazzy;

        # ワークスペースの自動構築
        myRosWorkspace = rosDistro.buildEnv {
          paths = [
            # --- 共通で入れたいパッケージ ---
            rosDistro.ros-base
            rosDistro.slam-toolbox
            rosDistro.rviz2
            rosDistro.joy
            rosDistro.demo-nodes-cpp

            # --- 自動スキャン (rosdepの代わり) ---
            # package.xml を読んで依存関係を自動解決
            (rosDistro.buildWorkspace {
              name = "labroom-searching-workspace";
              src = ./ros_ws/src;
            })
          ];
        };

      in {
        devShells.default = pkgs.mkShell {
          name = "ros2-jazzy-multiarch";

          buildInputs = [
            # ROS環境
            myRosWorkspace

            # 共通ツール
            pkgs.colcon
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
            
            # colcon defaults
            if [ ! -f $HOME/.config/colcon/defaults.yaml ]; then
              mkdir -p $HOME/.config/colcon
              echo "build: {args: ['--symlink-install']}" > $HOME/.config/colcon/defaults.yaml
            fi

            # 起動メッセージ (アーキテクチャを表示)
            echo "--------------------------------------------------------"
            echo " ROS 2 Jazzy Environment Loaded!"
            echo " System Architecture: ${system}"
            echo "--------------------------------------------------------"
          '';
        };
      }
    );
}
