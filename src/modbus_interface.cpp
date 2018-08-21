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
    return modbus_read_registers(context_, cmd, nb, data) != -1;
  }

  bool set(int slave, int cmd, int nb, uint16_t *data)
  {
    modbus_set_slave(context_, slave);
    return modbus_write_registers(context_, cmd, nb, data) != -1;
  }

  bool readBuffer(int slave, int nb, uint16_t *data)
  {
    modbus_set_slave(context_, slave);
    return modbus_read_registers(context_, READ_BUFFER, nb, data);
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

  bool deinitializeConfig(uint8_t slave)
  {
    return set(slave, DEINITIALIZE_CONFIG, 0, NULL);
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
    DEINITIALIZE_CONFIG = 0x07
  };

  modbus_t *context_;
};

class Thruster
{
public:
  Thruster(ModbusWorker* modbus, uint8_t slave, uint8_t thrId)
    : mw_(modbus), slave_(slave)
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
    if (abs(sp) > 1.0)
      sp = copysign(1.0, sp);
    return sp * 127;
  }

  enum Command : int
  {
    INIT      = 0x10,
    SET_SPEED = 0x11
  };

  ModbusWorker* mw_;
  const uint8_t slave_;
  power_data_t sp_req_;
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
    int cmd = smart_ ? INIT_S : INIT_S;
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
  BCS(ModbusWorker *modbus, uint8_t slave, uint8_t bcsId, uint8_t speed)
    : mw_(modbus), slave_(slave), device_id_(bcsId)
  {
    pos_ = 0;
    neutral_pos_ = 200; // eto odin oborot

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

  bool move(uint8_t direction, uint16_t steps)
  {
    move_bcs_data_t move_req;
    move_req.data.device_id = device_id_;
    move_req.data.direction = direction;
    move_req.data.steps = SWAP_BYTES(steps);
    return mw_->set(slave_, MOVE, move_req.BINARY_DATA_SIZE, move_req.binary);
  }

  bool moveToEnd(uint8_t direction)
  {
    pos_ = (direction == 0) ? -1 : 1;
    move_to_end_bcs_data_t move_req;
    move_req.data.device_id = device_id_;
    move_req.data.direction = direction;
    return mw_->set(slave_, MOVE_TO_END, move_req.BINARY_DATA_SIZE, move_req.binary);
  }

  bool stop()
  {
    stop_bcs_data_t stop_req;
    stop_req.data.device_id = SWAP_BYTES((uint16_t)device_id_);
    return mw_->set(slave_, STOP, stop_req.BINARY_DATA_SIZE, stop_req.binary);
  }

  void setNeutral()
  {
    pos_ = 0;
    int steps = neutral_pos_- pos_;
    pos_ = neutral_pos_;
    if (steps > 0)
      move(1, steps);
    else
      move(0, steps);
  }

  int getId()
  {
    return device_id_;
  }

private:
  enum BCSCommand
  {
    INIT        = 0x30,
    SET_SPEED   = 0x31,
    MOVE        = 0x32,
    MOVE_TO_END = 0x33,
    STOP        = 0x34
  };

  ModbusWorker *mw_;
  const uint8_t slave_;
  uint8_t device_id_;
  uint8_t pos_;
  uint16_t neutral_pos_;
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
    {
      enabled_ = false;
      return false;
    }

    uint16_t data[1];
    if (mw_->get(slave_, CHECK_CONNECTION, 1, data))
      return enabled_ = !data[0];

    return false;
  }

  bool ok()
  {
    return enabled_;
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
  bool enabled_;
};

}
