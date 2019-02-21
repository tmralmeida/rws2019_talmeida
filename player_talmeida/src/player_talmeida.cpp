#include <iostream>
#include <ros/ros.h>

int doSomething()
{
    return 0;
}

int main(int argc, char *argv[])
{
    // std::cout<< "Hello world"<< std::endl;
    ros::init(argc, argv, "player_talmeida");

    ros::NoteHandle n;
    ros::Rate r(1);
    for (int i = 0; i < 10; i++)
    {
        std::cout << i << std::endl;
    }

    return 0;
}
