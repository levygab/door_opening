#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_publisher");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("rviz_publication_marker_test", 10);

  // Set the frame ID and timestamp
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and ID for this marker
  marker.ns = "basic_shapes";
  marker.id = 10;

  // Set the marker type to cube list
  marker.type = visualization_msgs::Marker::CUBE_LIST;

  // Set the marker scale
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set the marker color
  // marker.color.r = 1.0f;
  // marker.color.g = 0.0f;
  // marker.color.b = 0.0f;
  // marker.color.a = 1.0;

  // Set the pose of the marker
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the lifetime of the marker
  marker.lifetime = ros::Duration();

  // Set the cube list
  for (float i = 0; i < 5; i++)
  {
    geometry_msgs::Point p;
    p.x = i+1;
    p.y = i+1;
    p.z = i+1;
    
    std_msgs::ColorRGBA color;
    color.r = i/5.0;  // Red component varies from 0 to 1
    color.a = 1.0;  // Alpha (transparency) is fixed at 1
    marker.points.push_back(p);
    marker.colors.push_back(color);

  }
    

  // Publish the marker
  while (ros::ok())
  {
    
    marker_pub.publish(marker);
    ros::spinOnce();
  }

  return 0;
}
