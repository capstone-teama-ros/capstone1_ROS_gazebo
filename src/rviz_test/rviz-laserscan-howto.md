## How to view LaserScan data using RViz

RViz에서 데이터를 보려면 ROS의 `tf` 패키지에서 제공하는 frame 정보가 필요합니다.

지금은 우리가 직접 frame을 생성하기 곤란하므로, 고정된 frame을 제공하는 서비스를 실행하겠습니다. 충분합니다.

### `base_scan` frame 만들기

LIDAR 데이터는 `base_scan`이라는 frame을 사용합니다. BAG 파일의 데이터를 재생할 경우, `base_scan` frame이 없으니 우리가 대신 만들어줘야 합니다.

아래 명령을 실행하면 base_scan이라는 frame을 publish하는 background process을 시작합니다.

```
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map base_scan 10 &
```

나중에 종료하려면 `fg` 명령을 사용해 해당 프로세스를 foreground로 불러온 뒤 <kbd>Ctrl+C</kbd>로 끄면 됩니다.

### BAG 파일의 데이터를 재생하려 할 경우

RViz를 실행하기 전에 ROS 파라미터로 `use_sim_time` 값을 설정해야 합니다.

```
rosparam set /use_sim_time true
```

이후 BAG 파일을 아래와 같이 실행합니다.

```
rosbag play --clock <bag_file>
```

이 때 `--loop` 등의 다른 옵션도 함께 사용할 수 있습니다.

### RViz 실행

```
rosrun rviz rviz
```

그냥 `rviz`를 사용해 켜는 방법도 있는데, 뭐가 더 나은지는 잘 모르겠습니다.

