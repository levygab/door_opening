#include "ros/ros.h"
//SIMULATION
#include "opening_door/door_info.h"
//REALITY
// #include "robot_smach_states/door_info.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "cb_base_navigation_msgs/LocalPlannerActionResult.h"
#include <cmath>

//tf2 librairy
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//message library
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/PointCloud2.h"

//pcl librairy
#include "pcl_ros/transforms.h"
#include "pcl/common/common.h"
#include <pcl_ros/point_cloud.h>
//pcl filter
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
//pcl segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//visualization
#include <visualization_msgs/Marker.h>


//size 
#define BOUNDING_BOX_SIZE 0.5

geometry_msgs::Twist twist_message(float a, float b, float c, float d, float e, float f){
    geometry_msgs::Twist pub;
    pub.angular.x = a;
    pub.angular.y = b;
    pub.angular.z = c;
    pub.linear.x = d;
    pub.linear.y = e;
    pub.linear.z = f;
    return pub;
}

geometry_msgs::PoseWithCovarianceStamped pose_message(float a, float b, float c, float d, float e, float f, float g){
    geometry_msgs::PoseWithCovarianceStamped pub ;
    pub.pose.pose.position.x = a;
    pub.pose.pose.position.y = b;
    pub.pose.pose.position.z = c;

    pub.pose.pose.orientation.x = d;
    pub.pose.pose.orientation.y = e;
    pub.pose.pose.orientation.z = f;
    pub.pose.pose.orientation.w = g;
    return pub;
}

geometry_msgs::PoseStamped poseStamped_message(float a, float b, float c, float d, float e, float f, float g) {
    geometry_msgs::PoseStamped pub;
    pub.pose.position.x = a;
    pub.pose.position.y = b;
    pub.pose.position.z = c;

    pub.pose.orientation.x = d;
    pub.pose.orientation.y = e;
    pub.pose.orientation.z = f;
    pub.pose.orientation.w = g;

    return pub;
}


class doorOpener {

    public:
        //ros variables
        ros::NodeHandle* nh;
        ros::ServiceServer service;
        ros::Publisher chatter_pose;
        ros::Publisher chatter_twist;
        ros::Publisher chatter_planner;
        ros::Subscriber sub;
        ros::Publisher marker_pub;

        boost::shared_ptr<sensor_msgs::LaserScan const> sharedLaserMessage;
        sensor_msgs::LaserScan laserMessage;

        boost::shared_ptr<cb_base_navigation_msgs::LocalPlannerActionResult const> sharedPlannerMessage;
        cb_base_navigation_msgs::LocalPlannerActionResult PlannerMessage;

        //other variables
        bool find_end;
        bool arrive_at_destination;

        //constructor
        doorOpener(ros::NodeHandle* nh_ptr): nh(nh_ptr) {
            //initialise service and publisher
            this -> service  = nh -> advertiseService("door_info", &doorOpener::doorInfo_callback, this);
            chatter_pose = nh -> advertise<geometry_msgs::PoseWithCovarianceStamped>("/hero/initialpose",1);
            chatter_twist = nh -> advertise<geometry_msgs::Twist>("/hero/base/references",1);
            chatter_planner = nh -> advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
            marker_pub = nh -> advertise<visualization_msgs::Marker>("visualization_marker", 10);
            }

        void position_achieve() {
           
            //wait for result of action_server message
            sharedPlannerMessage = ros::topic::waitForMessage<cb_base_navigation_msgs::LocalPlannerActionResult>("/hero/local_planner/action_server/result",*nh);
            
            if (sharedPlannerMessage != NULL) {
                PlannerMessage = *sharedPlannerMessage;
                arrive_at_destination = true;
            }
            else {
                ROS_INFO("problem in position_achieve");
                arrive_at_destination = true;
            }
        }

        void go_treshold(float limite){
            while (ros::ok() && !(this -> find_end)) {

                    //wait for scan message
                    sharedLaserMessage = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hero/base_laser/scan",*nh);
                    if (sharedLaserMessage != NULL) laserMessage = *sharedLaserMessage;
                    else ROS_INFO("problem in go_treshold");

                    int position_angle_zero = int(-(sharedLaserMessage->angle_min)/sharedLaserMessage->angle_increment);
                    float distance = sharedLaserMessage->ranges[position_angle_zero];

                    if (distance <= limite) {
                        ROS_INFO("idtance = %f", distance);
                        this -> find_end = true;
                    }

                    else {
                        geometry_msgs::Twist pub = twist_message(0, 0, 0, 0.05, -0.01, 0);
                        chatter_twist.publish(pub);
                    }

            }
            ROS_INFO("we can grab the handle");
        }

        bool isDoorOpen(){
            //call once the sensor data in order to know if we are in front of the door
            boost::shared_ptr<sensor_msgs::LaserScan const> sharedLaserMessage;
            sharedLaserMessage = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hero/base_laser/scan",*nh);

            if(sharedLaserMessage == NULL) {
                ROS_INFO("error in isDooropen");
            return 15;
            }

            int position_angle_zero = int(-(sharedLaserMessage->angle_min)/sharedLaserMessage->angle_increment);
            float distance = sharedLaserMessage->ranges[position_angle_zero];
            ROS_INFO("distance = %f", distance);
            return false;
            // if (distance < 0.630) {
            //     return false;
            // }
            // else return true;
        }

        int getDoorState(float rot_y) {
            float size_to_check = 0.3; //the door is around 0.92m.
            //call once the sensor data in order to know if we are in front of the door
            boost::shared_ptr<sensor_msgs::LaserScan const> sharedLaserMessage;
            sharedLaserMessage = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hero/base_laser/scan",*nh);

            if(sharedLaserMessage == NULL) {
                ROS_INFO("error in getDoorState");
                return 15;
            }

            float position_angle_zero_according_robot = -(sharedLaserMessage->angle_min)/sharedLaserMessage->angle_increment;
            //ROS_INFO("before update : %f, angle increment = %f, rot_y = %f", position_angle_zero_according_robot, sharedLaserMessage->angle_increment, rot_y);

            int position_angle_zero = int(position_angle_zero_according_robot - (rot_y/sharedLaserMessage->angle_increment)); //to take into account the rotation of the robot
            //ROS_INFO("after update : %d",position_angle_zero);
            float distance = sharedLaserMessage->ranges[position_angle_zero];
            //ROS_INFO("distance = %f", distance); //get the distance to the door
            float angle_max = atan(size_to_check/distance); //get the angle we have to ckeck the value

            int number_of_value_from_position_angle_zero = int(angle_max/sharedLaserMessage->angle_increment); //get the difference from angle 0
            //ROS_INFO("number_of_value = %d", number_of_value_from_position_angle_zero);
            //get the the position of the 2 value to check
            int value_to_check_min = position_angle_zero - number_of_value_from_position_angle_zero;
            int value_to_check_max = position_angle_zero + number_of_value_from_position_angle_zero;

            float d_angle_min = sharedLaserMessage->ranges[value_to_check_min];
            float d_angle_max = sharedLaserMessage->ranges[value_to_check_max];

            //ROS_INFO("d_angle_min = %f, d_angle_max = %f", d_angle_min, d_angle_max); //get the distance to the door

            float d_value = (pow(size_to_check,2) + pow(distance,2));
            d_value = pow(d_value,0.5);

            //ROS_INFO("d_value = %f", d_value);

            //calcul on the value we have
            if (distance > 1.4)  {
                ROS_INFO("door state : door is totally open");
                return 1;
            }

            if (abs(d_angle_max-d_angle_min)>0.08){
                ROS_INFO("door state : door is open, but not totally");
                return 2;
            }

            else {
                ROS_INFO("door state : door is close");
                return 3;
            }
        }

        visualization_msgs::Marker publish_marker() {
            //get the mesage from depth_registered
            boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedPointCloudMessage;
            sensor_msgs::PointCloud2 PointCloudMessage;
            //transform variable
            tf2_ros::Buffer tf_buffer(ros::Duration(20));
            tf2_ros::TransformListener tfListener(tf_buffer);

            //create a marker and a publisher for him
            //publisher 

            // Set the frame ID and timestamp
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();

            // Set the namespace and ID for this marker
            marker.ns = "clustersMarker";
            marker.id = 10;
            
            // Set the marker type to cube list
            marker.type = visualization_msgs::Marker::CUBE_LIST;

            // Set the marker scale
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
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
            marker.lifetime = ros::Duration();

            //get the message
            sharedPointCloudMessage = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/hero/head_rgbd_sensor/depth_registered/rectified_points", *nh);
            if (sharedPointCloudMessage != NULL) {
                PointCloudMessage = *sharedPointCloudMessage;
            }
            else{
                ROS_INFO("No point cloud message received");
                return marker;
            }

            ROS_INFO("height = %d , width = %d", PointCloudMessage.height, PointCloudMessage.width);
            
            geometry_msgs::PointStamped handle_location_frame_map;
            handle_location_frame_map.header.frame_id = "map";
            handle_location_frame_map.point.x = 7.475;
            handle_location_frame_map.point.y = 0.135;
            handle_location_frame_map.point.z = 1.06;
            geometry_msgs::PointStamped handle_location_frame_sensor;


            //test transform
            geometry_msgs::TransformStamped transformStamped;

            if (tf_buffer.canTransform( PointCloudMessage.header.frame_id, "map", ros::Time(0), ros::Duration(1.0))) {
                try {
                transformStamped = tf_buffer.lookupTransform(PointCloudMessage.header.frame_id, "map", ros::Time(0), ros::Duration(1.0));
                ROS_INFO("can transform");
                tf2::doTransform(handle_location_frame_map, handle_location_frame_sensor, transformStamped);        
                }
                catch (tf2::TransformException& ex) {
                    ROS_WARN("%s", ex.what());
                    return marker;
                }
            }
            else {
                ROS_INFO("can not transform");
            }
            ROS_INFO("after transform into frame sensor");
            
            ROS_INFO("Received PointStamped message:");
            ROS_INFO("Header: frame_id: %s", handle_location_frame_sensor.header.frame_id.c_str());
            ROS_INFO("Point: \n- x: %f \n- y: %f \n- z: %f", handle_location_frame_sensor.point.x, handle_location_frame_sensor.point.y, handle_location_frame_sensor.point.z);

            // This crops the pointcloud to a bounding box of 25 cm around the original handle location
            double min_x = handle_location_frame_sensor.point.x - (BOUNDING_BOX_SIZE/2.0);
            double min_y = handle_location_frame_sensor.point.y - (BOUNDING_BOX_SIZE/2.0);
            double min_z = handle_location_frame_sensor.point.z - (BOUNDING_BOX_SIZE/2.0);

            double max_x = handle_location_frame_sensor.point.x + (BOUNDING_BOX_SIZE/2.0);
            double max_z = handle_location_frame_sensor.point.z + (BOUNDING_BOX_SIZE/2.0);
            double max_y = handle_location_frame_sensor.point.y + (BOUNDING_BOX_SIZE/2.0);

            //create the pointcloud that will receive the output after cropping the data
            pcl::PointCloud<pcl::PointXYZ>::Ptr PC_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::fromROSMsg(PointCloudMessage, *PC_ptr);

            pcl::PointCloud<pcl::PointXYZ>::Ptr PC_cropped_frame_sensor_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); // pcl::PointXYZ is the type of point , ocl::PointCloud create a pointcloud from this type of point
            
            //create the crop variable 
            pcl::PassThrough<pcl::PointXYZ> crop_filter;
            crop_filter.setInputCloud(PC_ptr);
            //we are going to crop according to x, y  and z.
            crop_filter.setFilterFieldName("x");
            crop_filter.setFilterLimits(min_x, max_x);
            crop_filter.setFilterFieldName("y");
            crop_filter.setFilterLimits(min_y, max_y);
            crop_filter.setFilterFieldName("z");
            crop_filter.setFilterLimits(min_z, max_z);
            crop_filter.filter(*PC_cropped_frame_sensor_ptr);

            ROS_INFO("pass through filter done, here is some info about the header, the height, the width and the fields of cropped pointcloud:");
            ROS_INFO("Header: frame_id: %s", PC_cropped_frame_sensor_ptr -> header.frame_id.c_str());
            ROS_INFO("height = %d , width = %d", PC_cropped_frame_sensor_ptr -> height,  PC_cropped_frame_sensor_ptr -> width);
            //ROS_INFO("fields = %s", (PC_cropped_frame_sensor_ptr -> fields)[0]);

            // Create the segmentation object for the planar model and set all the parameters
            pcl::SACSegmentation<pcl::PointXYZ> seg; //segmentation object
            pcl::PointIndices::Ptr inliers = pcl::make_shared<pcl::PointIndices>(); //inliers points of the shape
            pcl::ModelCoefficients::Ptr coefficients = pcl::make_shared<pcl::ModelCoefficients>(); //
            pcl::PCDWriter writer;

            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(100);
            seg.setDistanceThreshold(0.01);

            pcl::PointCloud<pcl::PointXYZ>::Ptr PC_plane_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); //pointcloud for the output
            pcl::PointCloud<pcl::PointXYZ>::Ptr PC_plane_intermediate_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); //pointcloud for the intermediate output

            uint32_t nb_point = PC_cropped_frame_sensor_ptr -> points.size();

            //this part is going to remove points that are plane from the cloud
            while (PC_cropped_frame_sensor_ptr -> points.size() > 0.8 * nb_point){
                //print the number of points 
                ROS_INFO("number of points in the pointcloud = %ld", PC_cropped_frame_sensor_ptr -> points.size());
                //segment the largest planar of the cloud
                seg.setInputCloud(PC_cropped_frame_sensor_ptr);
                seg.segment(*inliers, *coefficients);
                if (inliers -> indices.size() == 0) {
                    ROS_INFO("Could not estimate a planar model for the given dataset.");
                    return marker;
                }

                //extract the planar inliers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(PC_cropped_frame_sensor_ptr);
                extract.setIndices(inliers);
                extract.setNegative(false);

                //get the points associated with the planar surface
                extract.filter(*PC_plane_intermediate_ptr);

                //remove the planar inliers, extract the rest
                extract.setNegative(true);
                extract.filter(*PC_plane_intermediate_ptr);

                *PC_cropped_frame_sensor_ptr = *PC_plane_intermediate_ptr;
            }
            
            // Creating the KdTree object for the search method of the extraction
            // a cluster is a group of points that are close to each other

            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
            tree->setInputCloud(PC_cropped_frame_sensor_ptr);
            
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.01); // 1cm
            ec.setMinClusterSize(100);
            ec.setMaxClusterSize(25000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(PC_cropped_frame_sensor_ptr);
            ec.extract(cluster_indices);

            ROS_INFO("there are %ld clusters", cluster_indices.size());

            //check the position of every cluster to know which one is the closest the handle
            double i = 0; //count for marker color
            double min_error = 1; //to get the point that will be use to grab the handle
             pcl::PointCloud<pcl::PointXYZ>::Ptr handle_cluster = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); //cluster that will represent the handle
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
                //create a new pointcloud for the cluster
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                
                //fill the pointcloud with the points of the cluster
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) cloud_cluster->points.push_back(PC_cropped_frame_sensor_ptr->points[*pit]);
                //fill the other info
                cloud_cluster->width = cloud_cluster->points.size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                //compute the center of the cluster
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cloud_cluster, centroid);

                //compute the difference between the center of the cluster and the handle
                double error_x = std::abs(centroid(0) - handle_location_frame_sensor.point.x);
                double error_y = std::abs(centroid(1) - handle_location_frame_sensor.point.y);
                double error_z = std::abs(centroid(2) - handle_location_frame_sensor.point.z);
                double measured_error = error_x + error_y + error_z;

                //print the info
                ROS_INFO("Cluster: size=%ld, centroid=(%f,%f,%f), Total error of the cluster = %f", cloud_cluster->size(), centroid[0], centroid[1], centroid[2], measured_error);

                


                geometry_msgs::Point p;
                p.x = centroid[0];
                p.y = centroid[1];
                p.z = centroid[2];

                std_msgs::ColorRGBA color;
                color.r = 1.0;  
                color.a = 1.0;  
                marker.points.push_back(p);
                marker.colors.push_back(color);
                i++;

                if (measured_error < min_error){
                    min_error = measured_error;
                    *handle_cluster = *cloud_cluster;
                    ROS_INFO("this is cluster number %f", i);
                }
                
            }       
            marker.header.frame_id = PointCloudMessage.header.frame_id;

            //making the handle cluster cube bigger than the other
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*handle_cluster, centroid);
            geometry_msgs::Point p;
            p.x = centroid[0];
            p.y = centroid[1];
            p.z = centroid[2];

            std_msgs::ColorRGBA color;
            color.r = 0;  
            color.a = 1.0;  
            color.g = 1.0;
            marker.points.push_back(p);
            marker.colors.push_back(color);

            return marker;
        // while (ros::ok())
        //     {
        //         marker_pub.publish(marker);
        //         ros::spinOnce();
        //     }
        // }
        }
        //SIMULATION vs REALITY
        bool doorInfo_callback(opening_door::door_info::Request &msg_rqst, opening_door::door_info::Response &msg_rsps) {
            ros::Rate sleeping_time(0.5);

            if(msg_rqst.input_string == "goIFOhandle") {
                ROS_INFO("go to the handle");
                geometry_msgs::PoseStamped pub1 = poseStamped_message(6.55, 0.381, 0, 0, 0, 0.001, 0.99);
                chatter_planner.publish(pub1);
                
                //Set arrive at destination on false. When it becomes tue, it means we arrive at destination
                this -> arrive_at_destination = false;

                // geometry_msgs::PoseWithCovarianceStamped pub1 = pose_message(6.55, 0.381, 0, 0, 0, 0.003, 0.99);
                // chatter_pose.publish(pub1);
                return true;
            }

            else if(msg_rqst.input_string == "goIFOhandle2") {
                ROS_INFO("go to the handle");
                geometry_msgs::PoseWithCovarianceStamped pub3 = pose_message(6.5, 0.381, 0, 0, 0, 0.003, 0.99);
                chatter_pose.publish(pub3);
                return true;
            }

            else if(msg_rqst.input_string == "goBehindDoor") {
                ROS_INFO("go behind the handle");
                geometry_msgs::PoseWithCovarianceStamped pub3 = pose_message(8.1, 0.37, 0, 0, 0, -0.99, 0.022);
                chatter_pose.publish(pub3);
                return true;
            }

            else if(msg_rqst.input_string == "goIFOdoor") {
                ROS_INFO("go IFO door");
                geometry_msgs::PoseStamped pub3 = poseStamped_message(6, 0.450, 0, 0, 0, 0.01, 0.99);
                chatter_planner.publish(pub3);
                
                //Set arrive at destination on false. When it becomes tue, it means we arrive at destination
                this -> arrive_at_destination = false;

                // geometry_msgs::PoseWithCovarianceStamped pub3 = pose_message(6, 0.450, 0, 0, 0, 0.01, 0.99);
                // chatter_pose.publish(pub3);
                return true;
            }

            else if (msg_rqst.input_string == "go_treshold") {
                ROS_INFO("going forward until threshold");
                //start the subscription to laser
                this -> find_end = false;
                //REALITY
                //this -> go_treshold(0.40);
                //SIMULATION
                this -> go_treshold(0.59);
                return true;
            }

            else if (msg_rqst.input_string == "go_treshold_behind") {
                ROS_INFO("going forward until threshold");
                //start the subscription to laser
                this -> find_end = false;
                this -> go_treshold(0.42);
                return true;
            }

            else if (msg_rqst.input_string == "is_door_open") {
                ROS_INFO("check of door is open or not");
                bool result = this -> isDoorOpen();
                return true;
            }

            else if (msg_rqst.input_string == "push_door") {
                ROS_INFO("go forward to push the door");
                geometry_msgs::Twist pub2 = twist_message(0, 0, 0, 1, 0, 0);
                chatter_twist.publish(pub2);
                sleeping_time.sleep();
                return true;
            }

            else if (msg_rqst.input_string == "go_forward") {
                ROS_INFO("go forward");
                geometry_msgs::Twist pub2 = twist_message(0, 0, 0, 0.18, 0, 0);
                chatter_twist.publish(pub2);
                sleeping_time.sleep();
                chatter_twist.publish(pub2);
                sleeping_time.sleep();
                chatter_twist.publish(pub2);
                sleeping_time.sleep();
                return true;
            }

            else if (msg_rqst.input_string == "door_state") {
                //ROS_INFO("get door state");
                //start the subscription to laser
                int result = this -> getDoorState(msg_rqst.input_int);
                msg_rsps.output_int = result;
                return true;
            }

            else if (msg_rqst.input_string == "go_other_side") {
                ROS_INFO("go other side of the door");
                geometry_msgs::PoseStamped pub3 = poseStamped_message(8.1, 0.37, 0, 0, 0, -0.01, 0.99);
                chatter_planner.publish(pub3);
                return true;
            }

            else if (msg_rqst.input_string == "arrive_at_destination"){
                //wait to acheve the goal
                while(ros::ok() && !this -> arrive_at_destination){
                    this -> position_achieve();
                }
                msg_rsps.output_int = 1;  
                return true;
            }

            else if (msg_rqst.input_string == "publish_marker"){
                visualization_msgs::Marker marker = this -> publish_marker();
                while (ros::ok()) {
                    marker_pub.publish(marker);
                    ros::spinOnce();
                }
                return true;
            }

            else {
                ROS_INFO("you sent a commande that does not exist");
                return false;
            }


        }

};


int main(int argc, char**argv) {
    ros::init(argc, argv, "door_informations");

    ros::NodeHandle nh;

    doorOpener opener = doorOpener(&nh);

    ros::spin();

    return 15;
}


