#include <math.h>
#include <modbus/modbus.h>

#include "akara_controller/modbus_data_types.h"

namespace modbus_interface {


class ModbusWorker
{
public:
  ModbusWorker()
  {
    context_ = nullptr;
  }

  bool initialize(const char *device, int baud, bool debug)
  {
    context_ = modbus_new_rtu(device, baud, 'N', 8, 1);

    modbus_set_debug(context_, debug);
//    modbus_set_response_timeout(context_, 1, 0);
    if (modbus_connect(context_) == -1)
    {
      modbus_free(context_);
      context_ = nullptr;
      return false;
    }
    return true;
  }

  ~ModbusWorker()
  {
    if (context_)
    {
      modbus_close(context_);
      modbus_free(context_);
    }
  }

  bool get(int slave, int cmd, int nb, uint16_t *data)
  {
    modbus_set_slave(context_, slave);
    if (modbus_read_registers(context_, cmd, nb, data) == -1)
    {
      ROS_ERROR("Failed to execute command %x on slave %i", cmd, slave);
      return false;
    }
    else
      return true;
  }

  bool set(int slave, int cmd, int nb, uint16_t *data)
  {
    modbus_set_slave(context_, slave);
    if (modbus_write_registers(context_, cmd, nb, data) == -1)
    {
      ROS_ERROR("Failed to execute command %x on slave %i", cmd, slave);
      return false;
    }
    else
      return true;
  }

  bool readBuffer(int slave, int nb, uint16_t *data)
  {
    modbus_set_slave(context_, slave);
    return modbus_read_registers(context_, READ_BUFFER, nb, data) != -1;
  }

  int ping(uint8_t slave)
  {
    return get(slave, PING, 0, NULL);
  }

  bool changeSlaveAddress(int slave, int new_slave)
  {
    uint16_t d = (uint8_t)new_slave << 8;
    modbus_set_slave(context_, slave);
    return set(slave, CH_SLAVE, 1, &d);
  }

  bool saveConfig(uint8_t slave)
  {
    modbus_set_slave(context_, slave);
    return set(slave, SAVE_CONFIG, 0, NULL);
  }

  bool resetConfig(uint8_t slave)
  {
    return set(slave, RESET_CONFIG, 0, NULL);
  }

  void reset(uint8_t slave)
  {
    set(slave, RESET, 0, NULL);
  }

private:
  enum Command
  {
    PING         = 0x00,
    CH_SLAVE     = 0x01,
    SAVE_CONFIG  = 0x02,
    RESET_CONFIG = 0x03,
    RESET        = 0x04,
    READ_BUFFER  = 0x06,
  };

  modbus_t *context_;
};

class Thruster
{
public:
  Thruster(ModbusWorker* modbus, uint8_t slave, uint8_t thrId, uint8_t deadzone)
    : mw_(modbus), slave_(slave), deadzone_(deadzone)
  {
    sp_req_.data.device_id = thrId;
  }

  ~Thruster()
  {
    setSpeed(0);
  }

  bool initialize(uint8_t timChId)
  {
    uint16_t d = timChId << 8;
    if (!mw_->set(slave_, INIT, 1, &d))
      return false;
    mw_->readBuffer(slave_, 1, &d);
    sp_req_.data.device_id = d >> 8;
    return true;
  }

  bool setSpeed(double speed)
  {
    sp_req_.data.power = _calculateSpeed(speed);
    return mw_->set(slave_, SET_SPEED, sp_req_.BINARY_DATA_SIZE, sp_req_.binary);
  }

  int getId()
  {
    return sp_req_.data.device_id;
  }

private:
  int8_t _calculateSpeed(double sp)
  {
    if (sp == 0)
      return 0;

    int8_t s = sp*127 + deadzone_;

    if (abs(s) > 127)
      return copysign(127, s);
    else
      return s;
  }

  enum Command : int
  {
    INIT      = 0x10,
    SET_SPEED = 0x11
  };

  ModbusWorker* mw_;
  const uint8_t slave_;
  power_data_t sp_req_;
  const uint8_t deadzone_;
};

class LED
{
public:
  LED(ModbusWorker *modbus, uint8_t slave, uint8_t ledId, bool smart=true)
    : mw_(modbus), slave_(slave), smart_(smart)
  {
    br_req_.data.device_id = ledId;
  }

  ~LED()
  {
    setBrightness(0);
  }

  bool initialize(uint8_t timChId)
  {
    int cmd = smart_ ? INIT_S : INIT;
    uint16_t d = timChId << 8;
    if (!mw_->set(slave_, cmd, 1, &d))
      return false;
    mw_->readBuffer(slave_, 1, &d);
    br_req_.data.device_id = d >> 8;
    return true;
  }

  bool setBrightness(float power)
  {
    int cmd = smart_ ? SET_BRIGHTNESS_S : SET_BRIGHTNESS;
    br_req_.data.power = _calculatePower(power);
    return mw_->set(slave_, cmd, br_req_.BINARY_DATA_SIZE, br_req_.binary);
  }

  int getId()
  {
    return br_req_.data.device_id;
  }

private:
  enum Command
  {
    INIT = 0x20,
    SET_BRIGHTNESS = 0x21,
    INIT_S = 0x50,
    SET_BRIGHTNESS_S = 0x51
  };

  uint8_t _calculatePower(double pow)
  {
    if (pow > 1.0)
      return 0xFF;
    else if (pow < 0.0)
      return 0x00;
    return pow * 0xFF;
  }

  ModbusWorker *mw_;
  const uint8_t slave_;
  const bool smart_;
  power_data_t br_req_;
};

class BCS
{
public:
  BCS(ModbusWorker *modbus, uint8_t slave, uint8_t bcsId, uint8_t speed,
      uint16_t neutral_position, uint16_t maximum_position, uint8_t positive_direction)
    : mw_(modbus),
      slave_(slave),
      device_id_(bcsId),
      neutral_pos_(neutral_position),
      max_pos_(maximum_position),
      positive_direction_(positive_direction)
  {
    setSpeed(speed);
  }

  bool initialize(uint8_t timChID, uint8_t dirGPIOChId,
                 uint8_t disGPIOChId, uint8_t termGPIOChId)
  {
    init_bcs_data_t data;
    data.data.timChID = timChID;
    data.data.dirGPIOChId = dirGPIOChId;
    data.data.disGPIOChId = disGPIOChId;
    data.data.termGPIOChId = termGPIOChId;
    if (!mw_->set(slave_, INIT, data.BINARY_DATA_SIZE, data.binary))
      return false;
    uint16_t d;
    mw_->readBuffer(slave_, 1, &d);
    device_id_ = d >> 8;
    return true;
  }

  bool setSpeed(uint8_t speed)
  {
    set_speed_bcs_data_t sp_req;
    sp_req.data.device_id = device_id_;
    sp_req.data.power = speed;
    return mw_->set(slave_, SET_SPEED, sp_req.BINARY_DATA_SIZE, sp_req.binary);
  }

  void move(int steps)
  {
    int new_position = current_pos_ + steps;

    if (new_position < 0 || new_position > max_pos_)
      return;

    move_bcs_data_t move_req;
    move_req.data.device_id = device_id_;
    move_req.data.steps = SWAP_BYTES(static_cast<uint16_t>(abs(steps)));

    if (steps < 0) // k konceviku
      move_req.data.direction = 1 - positive_direction_;
    else // ot koncevika
      move_req.data.direction = positive_direction_;


    current_pos_ = new_position;
    ROS_INFO("moving, steps: %i, dir: %u, current position: %i", steps, move_req.data.direction, current_pos_);

    mw_->set(slave_, MOVE, move_req.BINARY_DATA_SIZE, move_req.binary);
  }

  void setPositiveBuoyancy()
  {
    move(max_pos_ - current_pos_);
  }

  void setNeutralBuoyancy()
  {
    move(neutral_pos_ - current_pos_);
  }

  void setNegativeBuoyancy()
  {
    current_pos_ = 0;
    if (checkTerminal_()) // uzhe vozle koncevika
      return;

    move_to_end_bcs_data_t req;
    req.data.device_id = device_id_;
    req.data.direction = 1 - positive_direction_;
    mw_->set(slave_, MOVE_TO_END, req.BINARY_DATA_SIZE, req.binary);
  }

  bool stop()
  {
    uint16_t id = device_id_ << 8;
    return mw_->set(slave_, STOP, 1, &id);
  }

  void setZero()
  {
    current_pos_ = 0;
  }

  int getId()
  {
    return device_id_;
  }

  int getPosition()
  {
    return current_pos_;
  }

private:
  bool checkTerminal_()
  {
    uint16_t d;
    mw_->get(slave_, READ_TERMINAL, 1, &d);
    return SWAP_BYTES(d) == 0; // v koncevike
  }

  enum BCSCommand
  {
    INIT          = 0x30,
    SET_SPEED     = 0x31,
    MOVE          = 0x32,
    MOVE_TO_END   = 0x33,
    STOP          = 0x34,
    READ_TERMINAL = 0x35
  };

  ModbusWorker *mw_;
  const uint8_t slave_;
  uint8_t device_id_;

  int current_pos_;
  int neutral_pos_;
  int max_pos_;
  uint8_t positive_direction_; // ot koncevika
};

class MS5837
{
public:
  MS5837()
  {
    mw_ = NULL;
  }

  void initialize(ModbusWorker *modbus, uint8_t slave)
  {
    mw_ = modbus;
    slave_ = slave;
  }

  bool checkConnection()
  {
    if (!mw_)
      return false;

    uint16_t data[1];
    if (mw_->get(slave_, CHECK_CONNECTION, 1, data))
      return !data[0];

    return false;
  }

  bool readTemp(float *temp)
  {
    temp_data_t temp_data;
    if (!mw_->get(slave_, READ_TEMP, temp_data.BINARY_DATA_SIZE, temp_data.binary))
      return false;

    temp_data.binary[0] = SWAP_BYTES(temp_data.binary[0]);
    temp_data.binary[1] = SWAP_BYTES(temp_data.binary[1]);
    *temp = temp_data.temp;

    return true;
  }

  bool readPress(float *press)
  {
    temp_data_t press_data;
    if (!mw_->get(slave_, READ_PRESS, press_data.BINARY_DATA_SIZE, press_data.binary))
      return false;

    press_data.binary[0] = SWAP_BYTES(press_data.binary[0]);
    press_data.binary[1] = SWAP_BYTES(press_data.binary[1]);
    *press = press_data.temp;

    return true;
  }

  bool readTempAndPress(float *temp, float *press)
  {
    temp_and_press_data_t data;
    if (!mw_->get(slave_, READ_TEMP_AND_PRESS, data.BINARY_DATA_SIZE, data.binary))
      return false;

    data.binary[0] = SWAP_BYTES(data.binary[0]);
    data.binary[1] = SWAP_BYTES(data.binary[1]);
    data.binary[2] = SWAP_BYTES(data.binary[2]);
    data.binary[3] = SWAP_BYTES(data.binary[3]);

    *temp = data.data.temp;
    *press = data.data.press;
    return true;
  }

  bool setParams(uint8_t d1osr, uint8_t d2osr)
  {
    uint16_t d1 = d1osr << 8;
    uint16_t d2 = d2osr << 8;
    if (mw_->set(slave_, CH_D1OSR, 1, &d1) &&
        mw_->set(slave_, CH_D2OSR, 1, &d2))
      return true;
    return false;
  }

private:
  enum MS5837Command
  {
    CHECK_CONNECTION    = 0x40,
    READ_TEMP           = 0x41,
    READ_TEMP_AND_PRESS = 0x42,
    READ_PRESS          = 0x43,
    CH_D1OSR            = 0x44,
    CH_D2OSR            = 0x45
  };

  ModbusWorker *mw_;
  uint8_t slave_;
};

class HydroAcoustics
{
public:
  HydroAcoustics()
  {
    mw_ = NULL;
  }

  void initialize(ModbusWorker *modbus, uint8_t slave)
  {
    mw_ = modbus;
    slave_ = slave;
  }

  bool readAngles(float *phi, float *psi)
  {
    hydroacustics_data_t data;
    if (!mw_->get(slave_, READ, data.BINARY_DATA_SIZE, data.binary))
      return false;

    ROS_INFO("detect: %u, k12: %i, level1: %i, k13: %i, level2: %i, k23: %i",
             data.data.detect, data.data.k12, data.data.level1, data.data.k13, data.data.level2, data.data.k23);

    return true;
  }

private:
  enum HydroAcousticsCommand
  {
    READ = 0x00
  };

  ModbusWorker *mw_;
  uint8_t slave_;
};

}
