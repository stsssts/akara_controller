#ifndef MODBUS_DATA_TYPES_H
#define MODBUS_DATA_TYPES_H

namespace modbus_interface {

#define SWAP_BYTES(A) ((A << 8) | (A >> 8))

#pragma pack (push, 1)
union settings_data_t
{
  enum
  {
    BINARY_DATA_SIZE = 4
  };
  struct
  {
    uint8_t  slave;     // 1
    uint8_t  cmd_type;  // 2
    uint16_t cmd;       // 4
  } data;

  uint8_t binary[BINARY_DATA_SIZE];
};
#pragma pack (pop)


#pragma pack (push, 1)
union init_data_t
{
  enum
  {
    BINARY_DATA_SIZE = 5
  };
  struct
  {
    uint8_t  slave;     // 1
    uint8_t  cmd_type;  // 2
    uint16_t cmd;       // 4
    uint8_t  device_id; // 5
  } data;

  uint8_t binary[BINARY_DATA_SIZE];
};
#pragma pack (pop)


#pragma pack (push, 1)
union power_data_t
{
  enum
  {
    BINARY_DATA_SIZE = 1
  };
  struct
  {
    int8_t   power;     // 0.5
    uint8_t  device_id; // 0.5
  } data;

  uint16_t binary[BINARY_DATA_SIZE];
};
#pragma pack (pop)


#pragma pack (push, 1)
union init_bcs_data_t
{
  enum
  {
    BINARY_DATA_SIZE = 2
  };
  struct
  {
    uint8_t  dirGPIOChId;  // 3
    uint8_t  timChID;      // 4
    uint8_t  termGPIOChId; // 1
    uint8_t  disGPIOChId;  // 2
  } data;

  uint16_t binary[BINARY_DATA_SIZE];
};
#pragma pack (pop)


#pragma pack (push, 1)
union set_speed_bcs_data_t
{
  enum
  {
    BINARY_DATA_SIZE = 1
  };
  struct
  {
    uint8_t  power;     // 0.5
    uint8_t  device_id; // 0.5
  } data;

  uint16_t binary[BINARY_DATA_SIZE];
};
#pragma pack (pop)


#pragma pack (push, 1)
union move_bcs_data_t
{
  enum
  {
    BINARY_DATA_SIZE = 2
  };
  struct
  {
    uint8_t  direction; // 0.5
    uint8_t  device_id; // 0.5
    uint16_t steps;     // 1
  } data;

  uint16_t binary[BINARY_DATA_SIZE];
};
#pragma pack (pop)


#pragma pack (push, 1)
union move_to_end_bcs_data_t
{
  enum
  {
    BINARY_DATA_SIZE = 1
  };
  struct
  {
    uint8_t  direction; // 0.5
    uint8_t  device_id; // 0.5
  } data;
  uint16_t binary[BINARY_DATA_SIZE];
};
#pragma pack (pop)


#pragma pack (push, 1)
union temp_data_t
{
  enum
  {
    BINARY_DATA_SIZE = 2
  };

  float  temp; // 2

  uint16_t binary[BINARY_DATA_SIZE];
};
#pragma pack (pop)


#pragma pack (push, 1)
union temp_and_press_data_t
{
  enum
  {
    BINARY_DATA_SIZE = 4
  };
  struct
  {
    float temp;  // 2
    float press; // 2
  } data;

  uint16_t binary[BINARY_DATA_SIZE];
};
#pragma pack (pop)


#pragma pack (push, 1)
union hydroacustics_data_t
{
  enum
  {
    BINARY_DATA_SIZE = 3
  };
  struct
  {
    uint8_t detect;
    int8_t k12;
    uint8_t level1;
    int8_t k13;
    uint8_t level2;
    int8_t k23;
  } data;

  uint16_t binary[BINARY_DATA_SIZE];
};
#pragma pack (pop)

} // namespace modbus_interface

#endif // MODBUS_DATA_TYPES_H
