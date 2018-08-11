#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <akara_msgs/Thruster.h>

#include "modbus_interface.cpp"


using namespace modbus_interface;

class AkaraController
{
public:
  AkaraController() : _node("~")
  {
    std::string device;
    int baud;
    bool debug;

    _node.param("device", device, std::string("/dev/ttyUSB0"));
    _node.param("baudrate", baud, 115200);
    _node.param("debug", debug, false);
    _node.param("rate", _loop_rate, 1);

    if (!_mb.initialize(device.c_str(), baud, debug))
    {
      ROS_FATAL("Can't open the device %s", device.c_str());
      _node.shutdown();
      return;
    }

    _loadDeviceParameters("thrusters", _thrusters);
    _loadDeviceParameters("leds", _leds);
    _loadDeviceParameters("bcs", _bcs);
    _loadMS5837Parameters();

    ros::NodeHandle root_nh;
    _thruster_subscriber = root_nh.subscribe<akara_msgs::Thruster>(
          "thrusters", 1, boost::bind(&AkaraController::thrustersCallback, this, _1));

    _led_subscriber = root_nh.subscribe<std_msgs::Float64>(
          "led", 1, boost::bind(&AkaraController::ledCallback, this, _1));

    // advertise buoyancy service
    // create ms5837 publisher
  }

  void thrustersCallback(const akara_msgs::ThrusterConstPtr& msg)
  {
    if (msg->power.size() != _thrusters.size())
    {
      ROS_ERROR("Thrusters message should contain %lu elements", _thrusters.size());
      return;
    }

    for (int i = 0; i < msg->power.size(); ++i)
      _thrusters[i].setSpeed(msg->power[i]);
  }

  void ledCallback(const std_msgs::Float64ConstPtr& msg)
  {
    for (auto led : _leds)
      led.setBrightness(msg->data);
  }

  void spin()
  {
    ros::Rate loop_rate(_loop_rate);
    while (_node.ok())
    {
      float temp, press;
      if (_ms5837.readTempAndPress(&temp, &press))
        ROS_INFO("Temp: %f, Press: %f", temp, press);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

private:
  template<typename DeviceType>
  void _loadDeviceParameters(const char* name, std::vector<DeviceType>& devices)
  {
    if (!_node.hasParam(name))
    {
      ROS_WARN("Doesn't find %s parameters, they won't be loaded", name);
      return;
    }

    XmlRpc::XmlRpcValue params;
    _node.getParam(name, params);
    for (auto p : params)
    {
      if (p.second.hasMember("slave") && p.second.hasMember("id"))
      {
        int slave = p.second["slave"];
        int id = p.second["id"];
        if (_mb.ping(slave))
        {
          devices.push_back(DeviceType(&_mb, slave, id));
          ROS_INFO("Added %s %s - slave: %i, id: %i", name, p.first.c_str(), slave, id);
        }
        else
          ROS_ERROR("Slave %i not availible, %s %s won't be added", slave, name, p.first.c_str());
      }
    }
  }

  void _loadMS5837Parameters()
  {
    if (!_node.hasParam("ms5837/slave"))
    {
      ROS_WARN("Doesn't find MS5837 parameters, it won't be loaded");
      return;
    }

    int slave;
    _node.getParam("ms5837/slave", slave);
    _ms5837.initialize(&_mb, slave);
    if (_ms5837.checkConnection() )
      ROS_INFO("Added MS5837 on slave %i", slave);
    else
      ROS_WARN("Failed to add MS5837 on slave %i", slave);
  }

  ros::NodeHandle _node;
  int _loop_rate;

  ModbusWorker _mb;
  std::vector<Thruster> _thrusters;
  std::vector<LED> _leds;
  std::vector<BCS> _bcs;
  MS5837 _ms5837;

  ros::Subscriber _thruster_subscriber;
  ros::Subscriber _led_subscriber;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "akara_controller_node");
  AkaraController ac;
  ac.spin();
  return EXIT_SUCCESS;
}
