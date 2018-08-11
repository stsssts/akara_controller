#include <ros/ros.h>

#include "modbus_interface.cpp"

using namespace modbus_interface;

void configureThrusters(ros::NodeHandle& nh, ModbusWorker* mw)
{
  if (!nh.hasParam("thrusters"))
  {
    ROS_WARN("Doesn't find thrusters parameters, they won't be configured");
    return;
  }
  ros::Rate r(2);

  XmlRpc::XmlRpcValue params;
  nh.getParam("thrusters", params);
  for (auto p : params)
  {
    if (p.second.hasMember("slave") && p.second.hasMember("timChId"))
    {
      int slave = p.second["slave"];
      int timChId = p.second["timChId"];

//      Thruster t(mw, slave, 0);
//      int device_id = t.initialize(timChId);

//      ROS_INFO("Configured thruster %s. id: %i, slave: %i", p.first.c_str(), device_id, slave);
    }
  }
}

void configureLeds(ros::NodeHandle& nh, ModbusWorker* mw)
{
  if (!nh.hasParam("leds"))
  {
    ROS_WARN("Doesn't find leds parameters, they won't be configured");
    return;
  }
  ros::Rate r(2);

  XmlRpc::XmlRpcValue params;
  nh.getParam("leds", params);
  for (auto p : params)
  {
    if (p.second.hasMember("slave") && p.second.hasMember("timChId"))
    {
      int slave = p.second["slave"];
      int timChId = p.second["timChId"];

      LED l(mw, slave, 0);
      int device_id = l.initialize(timChId);

      ROS_INFO("Configured led %s. id: %i, slave: %i", p.first.c_str(), device_id, slave);
      ROS_INFO("Testing...");

      l.setBrightness(0.5);
      r.sleep();
      l.setBrightness(0);
    }
  }
}

void configureBCS(ros::NodeHandle& nh, ModbusWorker* mw)
{
  if (!nh.hasParam("bcs"))
  {
    ROS_WARN("Doesn't find bcs parameters, they won't be configured");
    return;
  }
  ros::Rate r(2);

  XmlRpc::XmlRpcValue params;
  nh.getParam("bcs", params);
  for (auto p : params)
  {
    if (p.second.hasMember("slave") && p.second.hasMember("timChId") &&
        p.second.hasMember("dirGPIOChId") && p.second.hasMember("disGPIOChId") &&
        p.second.hasMember("termGPIOChId"))
    {
      int slave = p.second["slave"];
      int timChId = p.second["timChId"];
      int dirGPIOChId = p.second["dirGPIOChId"];
      int disGPIOChId = p.second["disGPIOChId"];
      int termGPIOChId = p.second["termGPIOChId"];

      BCS b(mw, slave, 0);
//      int device_id = b.initialize(timChId, dirGPIOChId, disGPIOChId, termGPIOChId);

//      ROS_INFO("Configured bcs %s. id: %i, slave: %i", p.first.c_str(), device_id, slave);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "device_configure");
  ros::NodeHandle nh("~");

  std::string device;
  int baud;
  bool debug;
  nh.param("device", device, std::string("/dev/ttyUSB0"));
  nh.param("baudrate", baud, 115200);
  nh.param("debug", debug, false);

  ModbusWorker mw;

  if (!mw.initialize(device.c_str(), baud, debug))
  {
    ROS_FATAL("Can't open the device %s", device.c_str());
    nh.shutdown();
    return EXIT_FAILURE;
  }

  configureThrusters(nh, &mw);
  configureLeds(nh, &mw);
  configureBCS(nh, &mw);

  ROS_INFO("Configuration done");
  return EXIT_SUCCESS;
}
