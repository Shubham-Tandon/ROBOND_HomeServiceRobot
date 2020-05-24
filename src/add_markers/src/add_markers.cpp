#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

float pickup[3]    = {2.0, 3.0, 1.0};
float dropoff[3]   = {1.0,-4.0,-1.0};
float threshold[2] = {0.4, 0.4};

bool atPickup = false;
bool finishPickup = false;
bool atDropOff = false;
bool finishDropOff = false;


void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) 
{

    float posX = msg->pose.pose.position.x;
    float posY = msg->pose.pose.position.y;

    if (std::abs(posX - pickup[0]) < threshold[0] && std::abs(posY - pickup[1]) < threshold[1]) 
    {
        // pause 5s to simulate pickup
        ros::Duration(5).sleep();
        // hide marker and set it to new coordinates
        atPickup = true;
		ROS_INFO("At pickup location...");
    } 
    else
    {
        atPickup = false;
    }
    
    if (std::abs(posX - dropoff[0]) < threshold[0] && std::abs(posY - dropoff[1]) < threshold[1]) 
    {
        atDropOff = true;
	    ROS_INFO("At dropoff location...");
    }
    else
    {
        atDropOff = false;
    }

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub  = n.subscribe("/odom", 1000, odomCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;



    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pickup[0];
    marker.pose.position.y = pickup[1];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = pickup[2];

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
    
   while (ros::ok())
   {

        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
          if (!ros::ok())
          {
            return 0;
          }
          ROS_WARN_ONCE("Please create a subscriber to the marker");
          sleep(1);
        }


        while(!atPickup)
        {
            ros::spinOnce();
        }

        if(atPickup && !finishPickup)
        {
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker);
            ROS_INFO("Pick-up marker removed");
            finishPickup = true;
        }  

        //Wait for Drop-Off
        while(!atDropOff)
        {
            ros::spinOnce();
        }

        if(atDropOff && !finishDropOff)
        {
            marker.pose.position.x = dropoff[0];
            marker.pose.position.y = dropoff[1];
            marker.pose.orientation.w = dropoff[2];
            marker.action = visualization_msgs::Marker::ADD;
            marker_pub.publish(marker);
            ROS_INFO("Drop-off marker displayed");
            finishDropOff = true;
            ros::Duration(10.0).sleep();
        }  
    
  }
    
  return 0;

}

