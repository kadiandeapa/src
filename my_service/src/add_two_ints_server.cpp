#include "ros/ros.h"
#include "my_service/AddTwoInts.h"  // 自动生成的头文件，包含服务类型

// 服务回调函数，接收请求和响应
bool add(my_service::AddTwoInts::Request &req,
         my_service::AddTwoInts::Response &res) {
    res.sum = req.a + req.b;  // 计算并返回和
    ROS_INFO("Request: a=%ld, b=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("Sending back response: [%ld]", (long int)res.sum);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle n;

    // 创建服务
    ros::ServiceServer service = n.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to add two ints.");
    ros::spin();

    return 0;
}
