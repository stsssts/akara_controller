#include <ros/ros.h>

#include "modbus_interface.cpp"

using namespace modbus_interface;

class DeviceConfigure
{
public:
  DeviceConfigure() : pnh_("~")
  {
  }

  bool initialize()
  {
    std::string device;
    int baud;
    bool debug;
    pnh_.param("device", device, std::string("/dev/ttyUSB0"));
    pnh_.param("baudrate", baud, 115200);
    pnh_.param("debug", debug, false);

    if (!mw_.initialize(device.c_str(), baud, debug))
    {
      ROS_FATAL("Can't open the device %s", device.c_str());
      return false;
    }
    return true;
  }

  bool configure()
  {
    if (!pnh_.hasParam("slaves"))
      return false;

    XmlRpc::XmlRpcValue slaves;
    pnh_.getParam("slaves", slaves);
    for (auto slave : slaves)
    {
      ROS_INFO("Configure slave %s", slave.first.c_str());
      configureSlave_(slave.second);
    }
    ROS_INFO("Configuration done");
    return true;
  }

private:
  void configureSlave_(XmlRpc::XmlRpcValue& slave)
  {
    if (!slave.hasMember("id"))
    {
      ROS_ERROR("Can't find slave id parameter");
      return;
    }

    int id = slave["id"];
    if (!mw_.ping(id))
    {
      ROS_ERROR("Can't ping slave: %i", id);
      return;
    }

    for (auto devices : slave)
    {
      if (devices.first == "thrusters")
        configureThrusters_(id, devices.second);
      else if (devices.first == "leds")
        configureLeds_(id, devices.second);
      else if (devices.first == "bcs")
        configureBCS_(id, devices.second);
    }
    mw_.saveConfig(id);
  }

  void configureThrusters_(int slave, XmlRpc::XmlRpcValue& params)
  {
    for (auto t_params : params)
    {
      if (!t_params.second.hasMember("timChId"))
      {
        ROS_ERROR("Thruster %s doesn't have timChId param", t_params.first.c_str());
        continue;
      }

      int timChId = t_params.second["timChId"];

      Thruster thruster(&mw_, slave, 0);
      if (!thruster.initialize(timChId))
      {
        ROS_INFO("Can't add thruster at channel %i", timChId);
        continue;
      }
      ROS_INFO("Adding thruster %s with id %i", t_params.first.c_str(), thruster.getId());

      ROS_WARN("Testing...");
      ros::Rate r(2);
      r.sleep();
      thruster.setSpeed(0.2);
      r.sleep();
    }
  }

  void configureLeds_(int slave, XmlRpc::XmlRpcValue& params)
  {
    for (auto l_params : params)
    {
      if (!l_params.second.hasMember("timChId"))
      {
        ROS_ERROR("Led %s doesn't have timChId param", l_params.first.c_str());
        continue;
      }

      int timChId = l_params.second["timChId"];
      LED led(&mw_, slave, 0);
      if (!led.initialize(timChId))
      {
        ROS_INFO("Can't add led at channel %i", timChId);
        continue;
      }
      ROS_INFO("Adding led %s with id %i", l_params.first.c_str(), led.getId());

      ROS_WARN("Testing...");
      ros::Rate r(2);
      r.sleep();
      led.setBrightness(0.2);
      r.sleep();
    }
  }

  void configureBCS_(int slave, XmlRpc::XmlRpcValue& params)
  {
    for (auto b_params : params)
    {
      if (!b_params.second.hasMember("timChId"))
      {
        ROS_ERROR("BCS %s doesn't have timChId param", b_params.first.c_str());
        continue;
      }
      if (!b_params.second.hasMember("dirGPIOChId"))
      {
        ROS_ERROR("BCS %s doesn't have dirGPIOChId param", b_params.first.c_str());
        continue;
      }
      if (!b_params.second.hasMember("disGPIOChId"))
      {
        ROS_ERROR("BCS %s doesn't have disGPIOChId param", b_params.first.c_str());
        continue;
      }
      if (!b_params.second.hasMember("termGPIOChId"))
      {
        ROS_ERROR("BCS %s doesn't have disGPIOChId param", b_params.first.c_str());
        continue;
      }

      int timChId = b_params.second["timChId"];
      int dirGPIOChId = b_params.second["dirGPIOChId"];
      int disGPIOChId = b_params.second["disGPIOChId"];
      int termGPIOChId = b_params.second["termGPIOChId"];

      BCS bcs(&mw_, slave, 0);
      if (!bcs.initialize(timChId, dirGPIOChId, disGPIOChId, termGPIOChId))
      {
        ROS_INFO("Can't add bcs at channel %i", timChId);
        continue;
      }
      ROS_INFO("Adding bcs %s with id %i", b_params.first.c_str(), bcs.getId());

      ROS_WARN("Testing...");
      ros::Rate r(2);
      r.sleep();
      bcs.setSpeed(0.2);
      r.sleep();
      bcs.stop();
    }
  }

  ros::NodeHandle pnh_;
  ModbusWorker mw_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "device_configure");

  DeviceConfigure dc;
  if (dc.initialize())
    dc.configure();
  return EXIT_SUCCESS;
}
