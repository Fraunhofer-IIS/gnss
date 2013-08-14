#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <rtklib/rtklib.h>

int loadrosopts(opt_t *opts)
{
  ros::NodeHandle private_nh("~");
  for (opt_t *opt = opts; opt->var != NULL; opt++) {
    std::string value;
    std::string key(opt->name);
    std::replace(key.begin(), key.end(), '-', '_');
    private_nh.param(key, value, value);
    if (value.empty()) continue;
    if (!str2opt(opt, value.c_str()))
      trace(1,"loadrosopts: invalid option value '%s' (%s), %d\n", value.c_str(), opt->name, value.length());
  }
  return 1;
}
