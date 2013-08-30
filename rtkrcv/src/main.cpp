#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "rtkrcv/satellite.h"
#include "rtkrcv/satellitedata.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <math.h>
#include <iomanip>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

#define PI 3.142

using namespace std;

int start_rtkrcv(int argc, char **argv);
void stop_rtkrcv();
char get_solution(double &lat, double &lon, double &height);
char get_satellite_data(rtkrcv::satellitedata &sat_msg);

int main(int argc, char** argv)
{
  std::string frame_id;
  ros::init(argc, argv, "rtklib");

  ros::NodeHandle private_nh("~");
  private_nh.param("frame_id", frame_id, std::string("antenna"));

  visualization_msgs::Marker mark;
  visualization_msgs::Marker text_marker;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::MarkerArray text_marker_array;

  ros::NodeHandle nh;

  ros::Publisher pub_navsatfix = nh.advertise<sensor_msgs::NavSatFix>("fix", 5);
  ros::Publisher pub_satdata = nh.advertise<rtkrcv::satellitedata>("sat_data", 100);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("sat_topic", 100); 
  ros::Publisher text_marker_arr_pub= nh.advertise<visualization_msgs::MarkerArray>("text_topic", 100);
  
  if (!start_rtkrcv(argc, argv)){ 
      return(-1);
  }

  ros::Rate r(1); // 1 Hz

  while (ros::ok()) 
  { 
    rtkrcv::satellitedata sat_msg;
    rtkrcv::satellite data;   
    get_satellite_data(sat_msg); 
    pub_satdata.publish(sat_msg);

    
    sensor_msgs::NavSatFix fix;
    fix.header.stamp = ros::Time::now();
    fix.header.frame_id = frame_id;
    fix.status.service = fix.status.SERVICE_GPS | fix.status.SERVICE_GALILEO;
    fix.status.status = get_solution(fix.latitude, fix.longitude, fix.altitude);
    fix.position_covariance_type = fix.COVARIANCE_TYPE_UNKNOWN;
    
    pub_navsatfix.publish(fix);
    marker_array.markers.clear();

    for (const rtkrcv::satellite &sat: sat_msg.sat)
    {     
      mark.header.frame_id = "sat_frame";
      mark.header.stamp = ros::Time::now();
      mark.ns = "satellites";

      mark.id = sat.id;
      mark.type = visualization_msgs::Marker::CUBE;
      mark.action = visualization_msgs::Marker::ADD;
      
      mark.color.a = 1.0;
      mark.color.r = 0.0f;
      mark.color.b = 1.0f;
      mark.color.g = 1.0f;

      mark.pose.position.x = 10 *(sin(sat.elevation) * cos(sat.asimuth));
      mark.pose.position.y = 10 *(sin(sat.elevation) * sin(sat.asimuth));
      mark.pose.position.z = 10 *cos(sat.elevation);
      
      mark.pose.orientation.x = 0.0;
      mark.pose.orientation.y = 0.0;
      mark.pose.orientation.z = 0.0;
      mark.pose.orientation.w = 1.0;
      
      mark.scale.x = 0.4;
      mark.scale.y = 0.4;
      mark.scale.z = 0.4;
      
      mark.color.a = 1.0;
      mark.color.r = 1.0f;
      mark.color.b = 0.0f;
      mark.color.g = 0.0f;

      //Converting id from int to string
      string sat_id;
      ostringstream convert;
      convert << sat.id;
      text_marker.text= convert.str();                 

      text_marker.header.frame_id= "sat_frame";
      text_marker.header.stamp= ros::Time::now();
      text_marker.ns="satellites";

      text_marker.id = sat.id;
      text_marker.type= visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::Marker::ADD;
                     
      text_marker.pose.position.x = 10 *(sin(sat.elevation) * cos(sat.asimuth));
      text_marker.pose.position.y = 10 *(sin(sat.elevation) * sin(sat.asimuth));
      text_marker.pose.position.z = 10 *cos(sat.elevation)+ 0.5;
      
      text_marker.pose.orientation.x = 0.0;
      text_marker.pose.orientation.y = 0.0;
      text_marker.pose.orientation.z = 0.0;
      text_marker.pose.orientation.w = 1.0;

      text_marker.scale.x = 0.4;
      text_marker.scale.y = 0.4;
      text_marker.scale.z = 0.4;

      text_marker.color.a = 1.0;
      text_marker.color.r = 1.0f;
      text_marker.color.b = 1.0f;
      text_marker.color.g = 1.0f;

      marker_array.markers.push_back(mark); 
      text_marker_array.markers.push_back(text_marker);                                                     
    }
    
    marker_pub.publish(marker_array);
    text_marker_arr_pub.publish(text_marker_array);

    ros::spinOnce();

    // TODO krz: Instead of sleeping here read on monitor TCP socket until there is the data.
    // When there is data, a new solution was computed and the rtklib data struct
    // should be ready with the updated data (@see rtksvr.c:429 -> rtksvr.c:72ff).

    r.sleep();  
  }
  
  stop_rtkrcv();

  return(0);
}






