#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "talker_gugudan");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Int32>("gugudan", 50);
    
    ros::Rate loop_rate(1);  

    while (ros::ok()) 
    {
        int dan = 1; 
        while (dan <= 9) 
        {
            for (int i = 1; i <= 9; ++i) 
            {
                std_msgs::Int32 msg;
                msg.data = dan * i;
                ROS_INFO("multiply %d * %d = ", dan, i, msg.data);
                pub.publish(msg);
                ros::spinOnce();
                loop_rate.sleep();
            }
            
            ++dan;
            loop_rate.sleep(); 
        }
    }

    return 0;
}
