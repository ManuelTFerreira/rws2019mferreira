#include <iostream>

#include <ros/ros.h>


main(int argc, char *argv[])
{
    ros::init(argc, argv, "player_mferreira");

    ros::Rate r(100);

    for (int i = 0; i < 10; i++)
    {
        std::cout << i << std::endl;

        // r.sleep();
    }

    return 0;
}