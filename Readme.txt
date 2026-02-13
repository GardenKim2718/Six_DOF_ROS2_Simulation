1. terminator 설치 (미설치 시 아래의 launch.sh 이용 불가)
a) $ sudo apt update
b) $ sudo apt install terminator

2. RViz2 설치 (미설치 시 시각화 불가능)
a) $ sudo apt install ros-humble-rviz2
b) $ sudo apt install ros-humble-rviz-2d-overlay-plugins
c) $ sudo apt install ros-humble-rviz-2d-overlay-msgs

3. Build
a) ROS2 workspace의 src 폴더에 압축 파일 해제
b) $ cd src/free_floating_simulation    (다수의 package로 구성하였기에 해당 폴더에서 작업 권장)
c) colcon build --symlink-install

4. Execution (!!! free_floating_simulation 폴더에서 터미널 실행 !!!)
a) $ source install/setup.bash 
b) $ bash launch.sh
   자동으로 모든 노드 실행

5. 파라미터 수정 (VS Code 활용 권장)
주) launch 파일 통해 연결되기에 파라미터 수정 후 Build 다시 할 필요 없음
a) 유도/제어 파라미터 수정
   /src/free_floating_simulation/src/fsw/launch/fsw.launch.xml 파일 열어서 수정
b) 시뮬레이션 파라미터 수정
   /src/free_floating_simulation/src/simulator/launch/simulator.launch.xml 파일 열어서 수정