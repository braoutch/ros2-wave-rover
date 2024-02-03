#include <QCoreApplication>
#include <iostream>

#include <RobotController.hpp>


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    std::cout << "Hello, Gros Pote !" << std::endl;

    RobotController robot;
    // return a.exec();
}
