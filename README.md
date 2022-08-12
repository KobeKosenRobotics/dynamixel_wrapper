# dynamixel_wrapper
Dynamixel SDK wrapper library for ROS

## 1. Setup
```bash
cd ~/catkin_ws/src
git clone https://github.com/KobeKosenRobotics/dynamixel_wrapper
cd ..
rosdep install -i --from-paths src
catkin build
```

## 2. include pathの設定
自作のROS Packageから利用するための設定    

* CMakeList.txtのfind_packageにdynamixel_wrapperを追記
```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  dynamixel_wrapper #これを追記
)
```

* package.xmlに以下の3行を追記
```xml
<build_depend>dynamixel_wrapper</build_depend>
<build_export_depend>dynamixel_wrapper</build_export_depend>
<exec_depend>dynamixel_wrapper</exec_depend>
```

## 3. sample code
```cpp
#include <ros/ros.h>
#include <dynamixel_wrapper/dynamixel_wrapper.h>

int main(int argc, char **argv){
    
    ros::init(argc, argv, "sample_node");
    ros::NodeHandle n;
    double rate=10.0;
    //制御周期10Hz
    ros::Rate loop_rate(rate);

    //param setting
    ros::NodeHandle pn("~");
    
    std::string port_name("/dev/ttyUSB0");
    int baudrate=1000000;

    dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name,baudrate);
    dynamixel_wrapper::dynamixel_wrapper motor0(0,dxl_base,dynamixel_wrapper::XM430,40.0);
    

    motor0.setTorqueEnable(true);
    

    while (n.ok())  {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
```