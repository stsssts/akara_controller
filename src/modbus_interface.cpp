#include <modbus/modbus.h>
#include "akara_controller/modbus_data_types.h"

namespace modbus_interface {


class ModbusWorker
{
public:
  ModbusWorker()
  {
    _context = nullptr;
  }

  bool initialize(const char *device, int baud, bool debug)
  {
    _context = modbus_new_rtu(device, baud, 'N', 8, 1);

    modbus_set_debug(_context, debug);
    if (modbus_connect(_context) == -1)
    {
      modbus_free(_context);
      _context = nullptr;
      return false;
    }
    return true;
  }

  ~ModbusWorker()
  {
    if (_context)
    {
      modbus_close(_context);
      modbus_free(_context);
    }
  }

  bool get(int slave, int cmd, int nb, uint16_t *data)
  {
    modbus_set_slave(_context, slave);
    return modbus_read_registers(_context, cmd, nb, data) != -1;
  }

  bool set(int slave, int cmd, int nb, uint16_t *data)
  {
    modbus_set_slave(_context, slave);
    return modbus_write_registers(_context, cmd, nb, data) != -1;
  }

  bool readBuffer(int slave, int nb, uint16_t *data)
  {
    modbus_set_slave(_context, slave);
    return modbus_read_registers(_context, READ_BUFFER, nb, data);
  }

  int ping(uint8_t slave)
  {
    return get(slave, PING, 0, NULL);
  }

  bool changeSlaveAddress(int slave, int new_slave)
  {
    uint16_t d = (uint8_t)new_slave << 8;
    modbus_set_slave(_context, slave);
    return set(slave, CH_SLAVE, 1, &d);
  }

  bool saveConfig(uint8_t slave)
  {
    modbus_set_slave(_context, slave);
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
    READ_BUFFER  = 0x06
  };

  modbus_t *_context;
};

class Thruster
{
public:
  Thruster(ModbusWorker* modbus, uint8_t slave, uint8_t thrId)
    : _mw(modbus), _slave(slave)
  {
    _sp_req.data.device_id = thrId;
  }

  ~Thruster()
  {
    setSpeed(0);
  }

  bool initialize(uint8_t timChId)
  {
    uint16_t d = timChId << 8;
    return _mw->set(_slave, INIT, 1, &d);
  }

  bool setSpeed(int8_t speed)
  {
    _sp_req.data.power = speed;
    return _mw->set(_slave, SET_SPEED, _sp_req.BINARY_DATA_SIZE, _sp_req.binary);
  }

private:
  enum Command : int
  {
    INIT      = 0x10,
    SET_SPEED = 0x11
  };

  ModbusWorker* _mw;
  const uint8_t _slave;
  power_data_t _sp_req;
};

class LED
{
public:
  LED(ModbusWorker *modbus, uint8_t slave, uint8_t ledId)
    : _mw(modbus), _slave(slave)
  {
    _br_req.data.device_id = ledId;
  }

  ~LED()
  {
    setBrightness(0);
  }

  bool initialize(uint8_t timChId)
  {
    uint16_t d = timChId << 8;
    return _mw->set(_slave, INIT, 1, &d);
  }

  bool setBrightness(uint8_t power)
  {
    _br_req.data.power = power;
    return _mw->set(_slave, SET_BRIGHTNESS, _br_req.BINARY_DATA_SIZE, _br_req.binary);
  }

private:
  enum Command
  {
      INIT = 0x20,
      SET_BRIGHTNESS = 0x21
  };

  ModbusWorker *_mw;
  const uint8_t _slave;
  power_data_t _br_req;
};

class BCS
{
public:
  BCS(ModbusWorker *modbus, uint8_t slave, uint8_t bcsId)
    : _mw(modbus), _slave(slave), _device_id(bcsId)
  {
  }

  bool initialize(uint8_t timChID, uint8_t dirGPIOChId,
                 uint8_t disGPIOChId, uint8_t termGPIOChId)
  {
    init_bcs_data_t data;
    data.data.timChID = timChID;
    data.data.dirGPIOChId = dirGPIOChId;
    data.data.disGPIOChId = disGPIOChId;
    data.data.termGPIOChId = termGPIOChId;
    return _mw->set(_slave, INIT, data.BINARY_DATA_SIZE, data.binary);
  }

  bool setSpeed(int8_t speed)
  {
    power_data_t sp_req;
    sp_req.data.device_id = _device_id;
    sp_req.data.power = speed;
    return _mw->set(_slave, SET_SPEED, sp_req.BINARY_DATA_SIZE, sp_req.binary);
  }

  bool move(uint8_t direction, uint16_t steps)
  {
    move_bcs_data_t move_req;
    move_req.data.device_id = _device_id;
    move_req.data.direction = direction;
    move_req.data.steps = SWAP_BYTES(steps);
    return _mw->set(_slave, MOVE, move_req.BINARY_DATA_SIZE, move_req.binary);
  }

  bool moveToEnd(uint8_t direction)
  {
    move_to_end_bcs_data_t move_req;
    move_req.data.device_id = _device_id;
    move_req.data.direction = direction;
    return _mw->set(_slave, MOVE_TO_END, move_req.BINARY_DATA_SIZE, move_req.binary);
  }

  bool stop()
  {
    stop_bcs_data_t stop_req;
    stop_req.data.device_id = SWAP_BYTES((uint16_t)_device_id);
    return _mw->set(_slave, STOP, stop_req.BINARY_DATA_SIZE, stop_req.binary);
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

  ModbusWorker *_mw;
  const uint8_t _slave;
  const uint8_t _device_id;
};

class MS5837
{
public:
  MS5837()
  {  }

  void initialize(ModbusWorker *modbus, uint8_t slave)
  {
    _mw = modbus;
    _slave = slave;
  }

  bool checkConnection()
  {
    uint16_t data[1];
    if (_mw->get(_slave, CHECK_CONNECTION, 1, data))
      return !data[0];
    return false;
  }

  bool readTemp(float *temp)
  {
    temp_data_t temp_data;
    if (!_mw->get(_slave, READ_TEMP, temp_data.BINARY_DATA_SIZE, temp_data.binary))
      return false;

    temp_data.binary[0] = SWAP_BYTES(temp_data.binary[0]);
    temp_data.binary[1] = SWAP_BYTES(temp_data.binary[1]);
    *temp = temp_data.temp;

    return true;
  }

  bool readTempAndPress(float *temp, float *press)
  {
    temp_and_press_data_t data;
    if (!_mw->get(_slave, READ_TEMP_AND_PRESS, data.BINARY_DATA_SIZE, data.binary))
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
    if (_mw->set(_slave, CH_D1OSR, 1, &d1) &&
        _mw->set(_slave, CH_D2OSR, 1, &d2))
      return true;
    return false;
  }

private:
  enum MS5837Command
  {
    CHECK_CONNECTION    = 0x40,
    READ_TEMP           = 0x41,
    READ_TEMP_AND_PRESS = 0x42,
    CH_D1OSR            = 0x44,
    CH_D2OSR            = 0x45
  };

  ModbusWorker *_mw;
  uint8_t _slave;
};

}
