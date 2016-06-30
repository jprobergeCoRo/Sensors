/****************************************************************************************
//DataListener V1.1
//Author: Jean-Philippe Roberge Ing, M.Sc.A.
//Date: June 9th 2015
//Affiliations: Laboratoire de commande et de robotique (École de technologie supérieure)
//
//Description:  DataListener.cpp - A example script that listens for any kind of data
//              coming from any sensor(s). This starts by subscribing to the topics
//              "/TactileSensor/DynamicData" and "/TactileSensor/DynamicData". It will
//              display any data it receives.
//
//______________________________________________________________________________________
//Version: 1.0 : April 2nd 2015
//Version: 1.1 : June 9th 2015
//
//Last Modified: April 2nd 2015 - Initial release
//               June 9th 2015  - Modified to include acquisition of multiple sensors at
//                                the same time --> e.g. by adding option -sensor 1:10
//                                on the command line
****************************************************************************************/



#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64.h"
#include "tactilesensors/StaticData.h"
#include "tactilesensors/DynamicData.h"

void staticdataCallback(const tactilesensors::StaticData::ConstPtr &msg)
{
    for(int i=0;i<msg->Data.thedata.size();i++)
    {
        ROS_INFO("We have received static data: [%i] from sensor [%i] taxel [%i]", msg->Data.thedata[i],msg->SensorID,i );
    }
}

void dynamicdataCallback(const tactilesensors::DynamicData::ConstPtr &msg)
{
    ROS_INFO("We have received dynamic data: [%f] from sensor [%i]", msg->Data,msg->SensorID);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("TactileSensor/StaticData", 1000, staticdataCallback);
    ros::Subscriber sub2 = n.subscribe("TactileSensor/DynamicData", 1000, dynamicdataCallback);

    ros::spin();
    return 0;
}
