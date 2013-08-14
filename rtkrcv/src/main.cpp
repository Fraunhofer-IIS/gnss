#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

int start_rtkrcv(int argc, char **argv);
void stop_rtkrcv();
char get_solution(double &lat, double &lon, double &height);

int
main(int argc, char** argv)
{
  std::string frame_id;
  ros::init(argc, argv, "rtklib");

  ros::NodeHandle private_nh("~");
  private_nh.param("frame_id", frame_id, std::string("antenna"));

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::NavSatFix>("fix", 5);

  if (!start_rtkrcv(argc, argv)) return(-1);

  ros::Rate r(1); // 1 Hz
  while (ros::ok()) {
    sensor_msgs::NavSatFix fix;
    fix.header.stamp = ros::Time::now();
    fix.header.frame_id = frame_id;
    fix.status.service = fix.status.SERVICE_GPS | fix.status.SERVICE_GALILEO;
    fix.status.status = get_solution(fix.latitude, fix.longitude, fix.altitude);
    fix.position_covariance_type = fix.COVARIANCE_TYPE_UNKNOWN;
    pub.publish(fix);

    ros::spinOnce();
    r.sleep();
  }

  stop_rtkrcv();

  return(0);
}

