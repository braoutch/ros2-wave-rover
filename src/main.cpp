#include <QCoreApplication>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <RobotController.hpp>


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    std::cout << "Hello, Gros Pote !" << std::endl;
    rclcpp::init(argc, argv);
    RobotController robot;
    return a.exec();
}
