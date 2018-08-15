#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/FluidPressure.h>
#include <akara_msgs/Thruster.h>
#include <akara_msgs/BCS.h>

#include "modbus_interface.cpp"


using namespace modbus_interface;

class AkaraController
{
public:
  AkaraController() : node_("~")
  {
    std::string device;
    int baud;
    bool debug;

    node_.param("device", device, std::string("/dev/ttyUSB0"));
    node_.param("baudrate", baud, 115200);
    node_.param("debug", debug, false);
    node_.param("rate", loop_rate_, 1);

    if (!modbus_.initialize(device.c_str(), baud, debug))
    {
      ROS_FATAL("Can't open the device %s", device.c_str());
      node_.shutdown();
      return;
    }

    _loadDeviceParameters("thrusters", thrusters_);
    _loadDeviceParameters("leds", leds_);
    _loadDeviceParameters("bcs", bcs_);
    _loadMS5837Parameters();

    thrusters_[0].initialize(0);

    ros::NodeHandle nh;
    if (!thrusters_.empty())
      thruster_subscriber_ = nh.subscribe<akara_msgs::Thruster>(
            "thrusters", 1, &AkaraController::thrustersCallback, this);

    if (!leds_.empty())
      led_subscriber_ = nh.subscribe<std_msgs::Float32>(
            "led", 1, &AkaraController::ledCallback, this);

    if (!bcs_.empty())
      buoyancy_service_ = nh.advertiseService(
            "bcs_service", &AkaraController::bcsCallback, this);

    press_publisher_ = nh.advertise<sensor_msgs::FluidPressure>("pressure", 1);


  }

  void thrustersCallback(const akara_msgs::ThrusterConstPtr& msg)
  {
    if (msg->power.size() != thrusters_.size())
    {
      ROS_ERROR("Thrusters message should contain %lu elements", thrusters_.size());
      return;
    }

    for (int i = 0; i < msg->power.size(); ++i)
      thrusters_[i].setSpeed(msg->power[i]);
  }

  void ledCallback(const std_msgs::Float32ConstPtr& msg)
  {
    for (auto& led : leds_)
      led.setBrightness(msg->data);
  }

  bool bcsCallback(akara_msgs::BCSRequest& req, akara_msgs::BCSResponse& res)
  {
    if (req.buoyancy == "positive")
    {
      for (auto& bcs : bcs_)
        bcs.moveToEnd(0);
      return true;
    }

    if (req.buoyancy == "negative")
    {
      for (auto& bcs : bcs_)
        bcs.moveToEnd(1);
      return true;
    }

    if (req.buoyancy == "neutral")
    {
      for (auto& bcs : bcs_)
        bcs.setNeutral();
      return true;
    }

    return false;
  }

  void spin()
  {
    ros::Rate loop_rate(loop_rate_);
    while (node_.ok())
    {
      if (ms5837_.checkConnection())
      {
        float press;
        ms5837_.readPress(&press);
        press_msg_.fluid_pressure = press;
        press_publisher_.publish(press_msg_);
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

private:
  template<typename DeviceType>
  void _loadDeviceParameters(const char* name, std::vector<DeviceType>& devices)
  {
    if (!node_.hasParam(name))
    {
      ROS_WARN("Doesn't find %s parameters, they won't be loaded", name);
      return;
    }

    XmlRpc::XmlRpcValue params;
    node_.getParam(name, params);
    for (auto p : params)
    {
      if (!p.second.hasMember("slave"))
      {
        ROS_ERROR("Doesn't find slave parameter for %s", name);
        continue;
      }
      if (!p.second.hasMember("id"))
      {
        ROS_ERROR("Doesn't find id parameter for %s", name);
        continue;
      }

      int slave = p.second["slave"];
      int id = p.second["id"];
      if (modbus_.ping(slave))
      {
        devices.push_back(DeviceType(&modbus_, slave, id));
        ROS_INFO("Added %s %s - slave: %i, id: %i", name, p.first.c_str(), slave, id);
      }
      else
        ROS_ERROR("Slave %i not availible, skipping %s %s", slave, name, p.first.c_str());
    }
  }

  void _loadMS5837Parameters()
  {
    if (!node_.hasParam("ms5837/slave"))
    {
      ROS_WARN("Doesn't find MS5837 parameters, it won't be loaded");
      return;
    }

    int slave;
    node_.getParam("ms5837/slave", slave);
    ms5837_.initialize(&modbus_, slave);
    if (ms5837_.checkConnection() )
      ROS_INFO("Added MS5837 on slave %i", slave);
    else
      ROS_WARN("Failed to add MS5837 on slave %i", slave);
  }

  ros::NodeHandle node_;
  int loop_rate_;

  ModbusWorker modbus_;
  std::vector<Thruster> thrusters_;
  std::vector<LED> leds_;
  std::vector<BCS> bcs_;
  MS5837 ms5837_;

  ros::Subscriber thruster_subscriber_;
  ros::Subscriber led_subscriber_;

  ros::Publisher temp_publisher_;
  ros::Publisher press_publisher_;

  ros::ServiceServer buoyancy_service_;

  sensor_msgs::FluidPressure press_msg_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "akara_controller_node");
  AkaraController ac;
  ac.spin();
  return EXIT_SUCCESS;
}
