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
  AkaraController(ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh("~");

    std::string device;
    int baud;
    int rate;
    bool debug;

    pnh.param("device", device, std::string("/dev/ttyUSB0"));
    pnh.param("baudrate", baud, 115200);
    pnh.param("debug", debug, false);
    pnh.param("rate", rate, 1);

    if (!modbus_.initialize(device.c_str(), baud, debug))
    {
      ROS_FATAL("Can't open the device %s", device.c_str());
      ros::shutdown();
      return;
    }

    XmlRpc::XmlRpcValue slaves;
    pnh.getParam("slaves", slaves);

    for (auto slave : slaves)
    {
      ROS_INFO("Configuring slave %s", slave.first.c_str());
      configureSlave_(slave.second);
    }

    if (!thrusters_.empty())
      thruster_subscriber_ = nh.subscribe<akara_msgs::Thruster>(
            "thrusters", 1, &AkaraController::thrustersCallback, this);

    if (!leds_.empty())
      led_subscriber_ = nh.subscribe<std_msgs::Float32>(
            "led", 1, &AkaraController::ledCallback, this);

    if (!bcs_.empty())
    {
      buoyancy_subscriber_ = nh.subscribe<akara_msgs::Thruster>(
            "buoyancy", 1, &AkaraController::bcsMoveCallback, this);

      buoyancy_service_ = nh.advertiseService(
            "bcs_service", &AkaraController::bcsPositionCallback, this);
    }

    if (!ms5837_.empty())
    {
      press_publisher_ = nh.advertise<sensor_msgs::FluidPressure>("pressure", 1);
      timer_ = nh.createTimer(ros::Rate(rate), &AkaraController::publishPressure, this);
    }

    if (hydroacoustics_.ok())
    {
      // TODO: create publisher
      // TODO: create timer
    }
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
    {
      if (!led.setBrightness(msg->data))
        ROS_WARN("Failed to set brightness");
    }
  }

  void bcsMoveCallback(const akara_msgs::ThrusterConstPtr& msg)
  {
    if (msg->power.size() != bcs_.size())
    {
      ROS_ERROR("Thrusters message should contain %lu elements", thrusters_.size());
      return;
    }

    for (int i = 0; i < msg->power.size(); ++i)
      bcs_[i].move(msg->power[i]);
  }

  bool bcsPositionCallback(akara_msgs::BCSRequest& req, akara_msgs::BCSResponse& res)
  {
    try // if we can convert argument to int assume it is the speed
    {
      int speed = std::stoi(req.buoyancy);
      for (auto& bcs : bcs_)
        bcs.setSpeed(speed);
      return true;
    }
    catch (std::exception& e)
    {
      if (req.buoyancy == "set_zero")
      {
        for (auto& bcs : bcs_)
          bcs.setZero(); // sets current position to zero
        return true;
      }

      if (req.buoyancy == "positive")
      {
        for (auto& bcs : bcs_)
          bcs.setPositiveBuoyancy();
        return true;
      }

      if (req.buoyancy == "negative")
      {
        for (auto& bcs : bcs_)
          bcs.setNegativeBuoyancy();
        return true;
      }

      if (req.buoyancy == "neutral")
      {
        for (auto& bcs : bcs_)
          bcs.setNeutralBuoyancy();
        return true;
      }

      if (req.buoyancy == "stop")
      {
        for (auto& bcs : bcs_)
          bcs.stop();
        return true;
      }
    }

    ROS_ERROR("Use one of the following commands: set_zero, positive, negative, neutral, stop");
    return false;
  }

  void publishPressure(const ros::TimerEvent& event)
  {
    float pressure = 0;
    for (auto& m : ms5837_)
    {
      float p;
      if (!m.readPress(&p))
      {
        ROS_ERROR("slomalsya datchik!");
        return;
      }
      pressure += p;
    }

    press_msg_.fluid_pressure = pressure / ms5837_.size();
    press_msg_.header.stamp = ros::Time().now();
    press_publisher_.publish(press_msg_);
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
    if (!modbus_.ping(id))
    {
      ROS_ERROR("Can't ping slave: %i", id);
      return;
    }

//     akkuratno!!!
//    if (id == 2)
//      modbus_.changeSlaveAddress(id, 7);

    ROS_INFO("Configure slave %i", id);
    for (auto devices : slave)
    {
      if (devices.first == "thrusters")
        loadThrustersParameters_(id, devices.second);
      else if (devices.first == "leds")
        loadLedParameters_(id, devices.second);
      else if (devices.first == "bcs")
        loadBCSParameters_(id, devices.second);
      else if (devices.first == "hydroacoustics")
        loadHydroAcousticsParameters_(id, devices.second);
      else if (devices.first == "ms5837")
      {
        bool load = devices.second;
        if (load)
          loadMS5837Parameters_(id);
      }
    }
  }

  void loadThrustersParameters_(int slave, XmlRpc::XmlRpcValue& params)
  {
    for (auto p : params)
    {
      if (!p.second.hasMember("id"))
      {
        ROS_ERROR("Doesn't find id parameter for thrusters");
        continue;
      }
      if (!p.second.hasMember("deadzone"))
      {
        ROS_ERROR("Doesn't find deadzone parameter for thrusters");
        continue;
      }

      int id = p.second["id"];
      int deadzone = p.second["deadzone"];
      thrusters_.push_back(Thruster(&modbus_, slave, id, deadzone));
      ROS_INFO("Added thruster %s - slave: %i, id: %i", p.first.c_str(), slave, id);
    }
  }

  void loadBCSParameters_(int slave, XmlRpc::XmlRpcValue& params)
  {
    for (auto p : params)
    {
      if (!p.second.hasMember("id"))
      {
        ROS_ERROR("Doesn't find id parameter for bcs");
        continue;
      }
      if (!p.second.hasMember("speed"))
      {
        ROS_ERROR("Doesn't find speed parameter for bcs");
        continue;
      }
      if (!p.second.hasMember("neutral_position"))
      {
        ROS_ERROR("Doesn't find neutral position parameter for bcs");
        continue;
      }
      if (!p.second.hasMember("maximum_position"))
      {
        ROS_ERROR("Doesn't find neutral position parameter for bcs");
        continue;
      }
      if (!p.second.hasMember("positive_direction"))
      {
        ROS_ERROR("Doesn't find neutral position parameter for bcs");
        continue;
      }

      int id = p.second["id"];
      int speed = p.second["speed"];
      int neutral_pos = p.second["neutral_position"];
      int max_pos = p.second["maximum_position"];
      int positive_dir= p.second["positive_direction"];

      bcs_.push_back(BCS(&modbus_, slave, id, speed, neutral_pos, max_pos, positive_dir));
      ROS_INFO("Added bcs %s - slave: %i, id: %i", p.first.c_str(), slave, id);
      if (p.first != std::string("manipulator"))
      {
        bcs_.back().setNegativeBuoyancy();
        ROS_WARN("Moving bcs %s to zero position...", p.first.c_str());
      }
    }
  }

  void loadLedParameters_(int slave, XmlRpc::XmlRpcValue& params)
  {
    for (auto p : params)
    {
      if (!p.second.hasMember("id"))
      {
        ROS_ERROR("Doesn't find id parameter for led %s", p.first.c_str());
        continue;
      }
      if (!p.second.hasMember("smart"))
      {
        ROS_ERROR("Doesn't find smart parameter for led %s", p.first.c_str());
        continue;
      }

      int id = p.second["id"];
      bool smart = p.second["smart"];

      leds_.push_back(LED(&modbus_, slave, id, smart));
      ROS_INFO("Added led %s - slave: %i, id: %i", p.first.c_str(), slave, id);
    }
  }

  void loadMS5837Parameters_(int slave)
  {
    MS5837 m;
    m.initialize(&modbus_, slave);

    if (m.checkConnection())
    {
      ms5837_.push_back(m);
      ROS_INFO("Added MS5837 on slave %i", slave);
    }
    else
      ROS_WARN("Failed to add MS5837 on slave %i", slave);
  }

  void loadHydroAcousticsParameters_(int slave, XmlRpc::XmlRpcValue& params)
  {
    if (!params.hasMember("distance"))
    {
      ROS_ERROR("Doesn't find distance parameter for hydroacoustics");
      return;
    }
    if (!params.hasMember("disc_freq"))
    {
      ROS_ERROR("Doesn't find disc_freq parameter for hydroacoustics");
      return;
    }
    if (!params.hasMember("c"))
    {
      ROS_ERROR("Doesn't find c parameter for hydroacoustics");
      return;
    }

    int fd = params["disc_freq"];
    double d = params["distance"];
    double c = params["c"];

    hydroacoustics_.initialize(&modbus_, slave, fd, d, c);
  }

  ModbusWorker modbus_;

  std::vector<Thruster> thrusters_;
  std::vector<LED> leds_;
  std::vector<BCS> bcs_;
  std::vector<MS5837> ms5837_;
  HydroAcoustics hydroacoustics_;

  ros::Timer timer_;

  ros::Subscriber thruster_subscriber_;
  ros::Subscriber buoyancy_subscriber_;
  ros::Subscriber led_subscriber_;

  ros::Publisher temp_publisher_;
  ros::Publisher press_publisher_;

  ros::ServiceServer buoyancy_service_;

  sensor_msgs::FluidPressure press_msg_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "akara_controller_node");
  ros::NodeHandle nh;
  AkaraController ac(nh);
  ros::spin();
  return EXIT_SUCCESS;
}
