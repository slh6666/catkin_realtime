#include "ebotController.h"
#include "PidControl.h"
#include <tf/transform_datatypes.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace ebot_controller{

ebotControllerNode::ebotControllerNode(ros::NodeHandle &n){

    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf_listener_ = new tf::TransformListener();
    n.param("/ebotController/P", P, 0.5);
    n.param("/ebotController/I", I, 0.4);
    target_point_index = 0;
    n.param("/ebotController/D", D, 0.0);
    n.param("/ebotController/set_distance",set_distance,0.8);
    target_point_index = 0;
   

    sub_waypoints = n.subscribe("/local_trajectory",1,&ebotControllerNode::waypoints_cb,this);
    
    sub_rear_pose_test = n.subscribe("/amcl_pose",1,&ebotControllerNode::pose_cb_test,this);

  
    pub_twist_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    marker_pub =n.advertise<visualization_msgs::Marker>("/visualization_marker",1);

    control_timer_ = n.createTimer(ros::Duration(0.02), &ebotControllerNode::ControlCallback,this);
}

double ebotControllerNode::getdistance(geometry_msgs::Point A,geometry_msgs::Point B)
{
    double d = sqrt((A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y));
    return d;
}
double ebotControllerNode::point_distance(double x1,double y1,double x2, double y2)
{
    double d =sqrt(pow((x2-x1),2)+pow((y2-y1),2));
    return d;
}

void ebotControllerNode::pose_cb_test(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{

    my_odom.pose.pose.position.x = msg->pose.pose.position.x;
    my_odom.pose.pose.position.y = msg->pose.pose.position.y;
                


}
void ebotControllerNode::waypoints_cb(const trajectory_msgs::JointTrajectory::ConstPtr &recieved_curve) 
{   
    final_waypoints.header.frame_id = recieved_curve->header.frame_id;
    final_waypoints.header.stamp = ros::Time::now();
    final_waypoints.poses.resize(recieved_curve->points.size());
   
    for (size_t i = 0;i < recieved_curve->points.size(); i++)
    {
        
        final_waypoints.poses[i].pose.position.x = recieved_curve->points[i].positions[0];
        final_waypoints.poses[i].pose.position.y = recieved_curve->points[i].positions[1];
        final_waypoints.poses[i].pose.position.z = recieved_curve->points[i].positions[2];/
        if (final_waypoints.poses[i].pose.position.z < 0)
        {
            final_waypoints.poses[i].pose.position.z = final_waypoints.poses[i].pose.position.z + 2*3.1415926;
        }
        
        
    }
    velocity.linear.x = cos(final_waypoints.poses[target_point_index].pose.position.z)*recieved_curve->points[target_point_index].velocities[0] 
                        + sin(final_waypoints.poses[target_point_index].pose.position.z)*recieved_curve->points[target_point_index].positions[1];
    
    if (velocity.linear.x > 0.5)
    {
        velocity.linear.x = 0.5;
    }
    if (velocity.linear.x < -0.5)
    {
        velocity.linear.x = -0.5;
    }
    velocity.linear.x = velocity.linear.x; 
    std::cout<< "velocity.linear.x:" << velocity.linear.x << std::endl;

}


void ebotControllerNode::nearest_point_index(const nav_msgs::Path &msg) 
{

  

   
    currentX = my_odom.pose.pose.position.x;
    currentY = my_odom.pose.pose.position.y;
    geometry_msgs::Point current_pose_;
    current_pose_.x = currentX;
    current_pose_.y = currentY;

    
    len = final_waypoints.poses.size();

    if (final_waypoints.poses.size()>0)
    {

        double min_distance = sqrt(pow((currentX - final_waypoints.poses[0].pose.position.x),2)+pow((currentY-final_waypoints.poses[0].pose.position.y),2));
        
        
        for (int num = 0; num<len-1; num++) 
        {
            geometry_msgs::Point select_pose;
            select_pose.x = final_waypoints.poses[num].pose.position.x;
            select_pose.y = final_waypoints.poses[num].pose.position.y;
            double current_dis = getdistance(current_pose_,select_pose);   
            if(current_dis <  min_distance)
            {
                min_distance = current_dis;

                nearest_point = num;
               
     
            }    
            
             
        }
        target_point_index = nearest_point;
        set_distance =1;
        lad = 0;
        for ( target_point_index;target_point_index < len-1;target_point_index++)
        {

            add_distance = point_distance(final_waypoints.poses[target_point_index].pose.position.x,final_waypoints.poses[target_point_index].pose.position.y,
                                            final_waypoints.poses[target_point_index+1].pose.position.x,final_waypoints.poses[target_point_index+1].pose.position.y );
            lad +=add_distance;
            if (lad > set_distance)
            {
                target_point_index++;
                break; 
            }
            if(target_point_index >= len-1)
            {
                target_point_index = len -1;
            }
        }
        
        std::cout<< "target_point_index:" << target_point_index <<":"<< final_waypoints.poses[target_point_index].pose.position.x << ","<< final_waypoints.poses[target_point_index].pose.position.y << std::endl;
        std::cout<< "nearest_point_index:" << nearest_point <<":"<< final_waypoints.poses[nearest_point].pose.position.x << ","<< final_waypoints.poses[nearest_point].pose.position.y << std::endl;


    }

}



void ebotControllerNode::ControlCallback(const ros::TimerEvent& event){
    if (final_waypoints.poses.size()>0)
    {
        control_period_ = 0.02;
        max_steering_theta = 35;
        min_steering_theta =-35;
        PidControl PID;
        
        double control_period = control_period_;
        double steering_theta = steering_theta_;
        
        PID.setParams(P,I,D,min_steering_theta,max_steering_theta);
        nearest_point_index(final_waypoints);

        
        if (final_waypoints.poses.size()>0)
        {
            targetX = final_waypoints.poses[target_point_index].pose.position.x;
            targetY = final_waypoints.poses[target_point_index].pose.position.y;
        }

      
        tf::TransformListener listener;
        geometry_msgs::PointStamped base_link_point;
        base_link_point.header.frame_id = "/map";
        base_link_point.header.stamp = ros::Time();
    
        base_link_point.point.x = targetX;
        base_link_point.point.y = targetY;
        base_link_point.point.z = 0.0;
        geometry_msgs::PointStamped vehicle_frame_point;
        try{
            
            listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(3.0));
            listener.transformPoint("/base_link", base_link_point, vehicle_frame_point);
            intercept = fabs(vehicle_frame_point.point.y);
            ROS_INFO("map: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                base_link_point.point.x, base_link_point.point.y, base_link_point.point.z,
                vehicle_frame_point.point.x, vehicle_frame_point.point.y, vehicle_frame_point.point.z, vehicle_frame_point.header.stamp.toSec());
            std::cout<<"vehicle_frame_point.point.y:"<< vehicle_frame_point.point.y << std::endl;
            
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
        }


        double theta = 0;
        theta = PID.step(intercept,control_period);
        steering_theta = theta;
        
        if (vehicle_frame_point.point.y > 0)
        {
            steering_theta = steering_theta;
        
        }
        else
        {
            steering_theta = -steering_theta; 
        }


        if (steering_theta >100)
        {
                steering_theta =100;
        }
            
        else if(steering_theta < -100)
        {
                steering_theta = -100;
        }


        std::cout<<"steering_theta:"<<steering_theta<<std::endl;
        
        visualization_msgs::Marker point;
        point.header.frame_id = "/map";
        point.header.stamp = ros::Time();
        point.pose.position.x = targetX;
        point.pose.position.y = targetY;
        point.color.g =1.0f;
        point.color.a =1.0;
        point.scale.x = 0.2;
        point.scale.y = 0.2;
        point.scale.z = 0.2;
        marker_pub.publish(point);

        geometry_msgs::Twist cmd_twist;
        
        cmd_twist.linear.x = 0.5;
        /
        
        double set_stop_distance = 0.4;
        double stop_distance = point_distance(currentX,currentY,final_waypoints.poses.back().pose.position.x,final_waypoints.poses.back().pose.position.y);
        if (stop_distance < set_stop_distance)
        {
            cmd_twist.linear.x = 0.5*cmd_twist.linear.x; 

        }
        if (stop_distance  <= 0.2)
        {
            cmd_twist.linear.x = 0;
        }
        cmd_twist.angular.z = steering_theta;
        pub_twist_cmd.publish(cmd_twist);
    }
    
}


}






























