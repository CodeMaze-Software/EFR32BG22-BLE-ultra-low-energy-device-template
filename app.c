/***************************************************************************//**
 * Dariusz Adamczyk
 *
 * Compatible soc's:
 *
 * EFR32BG22 - IM40
 * EFR32BG22 - GM32
 *
 * @file app.c
 * @brief Application code
 *******************************************************************************
 *******************************************************************************
 ******************************************************************************/

#include <stdbool.h>
#include <stdio.h>
#include "em_common.h"
#include "sl_status.h"
#include "sl_bluetooth.h"
#include "app.h"
#include "gatt_db.h"
#include "sl_sleeptimer.h"
#include "em_gpio.h"
#include "em_usart.h"

#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT

#ifdef SL_CATALOG_APP_LOG_PRESENT
#include "app_log.h"
#endif // SL_CATALOG_APP_LOG_PRESENT

#ifdef SL_CATALOG_BTMESH_WSTK_LCD_PRESENT
#include "sl_btmesh_wstk_lcd.h"
#endif // SL_CATALOG_BTMESH_WSTK_LCD_PRESENT

#include "cJSON.h"

/**************************************************************************//**
 * Definitions
 *****************************************************************************/
#define UINT16_TO_BYTES(n)            ((uint8_t) (n)), ((uint8_t)((n) >> 8))
#define UINT16_TO_BYTE0(n)            ((uint8_t) (n))
#define UINT16_TO_BYTE1(n)            ((uint8_t) ((n) >> 8))

#define BUFSIZE                       128
#define SPP_BUFF_SIZE                 128
#define USART_NEWLINE_TERMINATOR      '\n'

#define SERIAL_READY_PORT             gpioPortB
#define SERIAL_REQUEST_PIN            1
#define SERIAL_READY_PIN              2

#define SERIAL_TEMPERATURE_CMD        "temperature"
#define SERIAL_HUMIDITY_CMD           "humidity"
#define SERIAL_BATT_LVL_CMD           "batt"

/**************************************************************************//**
 * Declarations
 *****************************************************************************/
static sl_status_t
read_current_timestamp_characteristic (void);

static sl_status_t
update_current_timestamp_characteristic (sl_sleeptimer_timestamp_t timestamp);

static sl_status_t
update_current_temperature_characteristic (uint8_t current_temperature);

static sl_status_t
update_current_humidity_characteristic (uint8_t current_humidity);

static sl_status_t
update_battery_level_characteristic (uint16_t battery_level);

void
convert_overdue_data_to_byte_array (uint32_t *in_data, uint8_t *out_data);

static sl_status_t
update_agregate_characteristic (uint32_t *agregate_data_array);

static sl_status_t
update_spp_characteristic (uint8_t *data);

static sl_status_t
read_spp_characteristic (uint8_t *data);

bool
blocking_serial_is_available (void);

bool
blocking_serial_write (uint8_t *buff, uint32_t buff_sz);

size_t
blocking_serial_read (uint8_t *data, uint8_t terminator);

void
blocking_serial_hw_control (bool connected);

bool
check_if_command_is_valid (char *inJson, size_t len, char *command,
                           uint8_t *value);
bool
parse_incoming_data_str (char *rxPacket);

float
convert_array_to_float(uint8_t *array);

/**************************************************************************//**
 * Handlers
 *****************************************************************************/
sl_sleeptimer_timer_handle_t app_timer_handler, led_timer_handler;
sl_sleeptimer_date_t datetime;
sl_sleeptimer_timestamp_t timestamp;

/**************************************************************************//**
 * Application global variables
 *****************************************************************************/
// TEST VARs
uint8_t dummy_h, dummy_t, dummy_i;
uint32_t dummy_agregate_buff[4];
uint8_t i;

// <<<<<<<<<

struct
{
  uint8_t flags_len;     // Length of the Flags field.
  uint8_t flags_type;    // Type of the Flags field.
  uint8_t flags;         // Flags field.
  uint8_t mandata_len;   // Length of the Manufacturer Data field.
  uint8_t mandata_type;  // Type of the Manufacturer Data field.
  uint8_t comp_id[2];    // Company ID field.
  uint8_t beac_type[2];  // Beacon Type field.
  uint8_t uuid[16]; // 128-bit Universally Unique Identifier (UUID). The UUID is an identifier for the company using the beacon.
  uint8_t maj_num[2];    // Beacon major number. Used to group related beacons.
  uint8_t min_num[2]; // Beacon minor number. Used to specify individual beacons within a group.
  uint8_t tx_power; // The Beacon's measured RSSI at 1 meter distance in dBm. See the iBeacon specification for measurement guidelines.
} bcn_beacon_adv_data =
  {
  // Flag bits - See Bluetooth 4.0 Core Specification , Volume 3, Appendix C, 18.1 for more details on flags.
      2,// Length of field.
      0x01,         // Type of field.
      0x04 | 0x02,  // Flags: LE General Discoverable Mode, BR/EDR is disabled.

      // Manufacturer specific data.
      26,// Length of field.
      0xFF, // Type of field.

      // The first two data octets shall contain a company identifier code from
      // the Assigned Numbers - Company Identifiers document.
        { UINT16_TO_BYTES(0x03aa) },

      // Beacon type.
      // 0x0215 is iBeacon.
        { UINT16_TO_BYTE1(0x0215), UINT16_TO_BYTE0(0x0215) },

      // 128 bit / 16 byte UUID
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00 },

      // Beacon major number.
      // Set to 34987 and converted to correct format.
        { UINT16_TO_BYTE1(34987), UINT16_TO_BYTE0(34987) },

      // Beacon minor number.
      // Set as 1025 and converted to correct format.
        { UINT16_TO_BYTE1(1025), UINT16_TO_BYTE0(1025) },

      // The Beacon's measured RSSI at 1 meter distance in dBm.
      // 0xD7 is -41dBm
      0xD7 };

enum
{
  DISABLED = 0, ENABLED, NONE
};

typedef union
{
  uint8_t bytes[4];
  float number;

}FLOATUNION_t;

typedef union
{
  uint8_t bytes[2];
  uint16_t number;

}UINT16UNION_t;

static uint8_t input_uart_buffer[SPP_BUFF_SIZE];
static uint8_t temp_buffer[SPP_BUFF_SIZE];

// Application ready flags
static bool agregate_notify_flag, is_client_connected = false;

static uint8_t ble_adv_state;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

static void
test_fun(void)
{
  if(++dummy_t >= 50)
    dummy_t = 0;

  if(++dummy_h >= 50)
    dummy_h = 0;

  update_current_temperature_characteristic(dummy_t);
  update_current_humidity_characteristic(dummy_h);

}

/**************************************************************************//**
 * Application Sleeptimer callback
 *****************************************************************************/
void
periodic_timer_callback (sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void) handle;
  (void) data;

  //### DUMMY TEST: auto characteristic update##
  //test_fun();//###############################

  // Update timestamp value
  update_current_timestamp_characteristic (sl_sleeptimer_get_time ());

  // Update overdue values
  if (agregate_notify_flag)
    {
      dummy_agregate_buff[0] = i++; // DUMMY TEST
      update_agregate_characteristic (dummy_agregate_buff);
    }
}

/***************************************************************************//**
 * Application Init.
 ******************************************************************************/
SL_WEAK void
app_init (void)
{
  sl_status_t sc;

  USART_InitAsync_TypeDef initAsync = USART_INITASYNC_DEFAULT;
  USART_InitAsync (USART1, &initAsync);

  // Disable overdue data notifying
  agregate_notify_flag = false;

  ble_adv_state = DISABLED;

  // Initialize datetime with WALL_CLOCK enabled in RTC
  sc = sl_sleeptimer_init ();

  // Initialize date-time wiht default values
  sl_sleeptimer_build_datetime (&datetime, 2022, MONTH_JANUARY, 1, 12, 00, 00,
                                0);
  // Set current datetime
  sc = sl_sleeptimer_set_datetime (&datetime);

  // Convert current date to UNX epoch format
  sc = sl_sleeptimer_convert_date_to_time (&datetime, &timestamp);

  // Set current date in UNIX epoch format
  sc = sl_sleeptimer_set_time (timestamp);

  // Enable periodic application timer
  sl_sleeptimer_start_periodic_timer (&app_timer_handler, 30000,
                                      periodic_timer_callback, (void*) NULL, 0,
                                      0);

  // DUMMY TEST <<<!>>>
  dummy_agregate_buff[1] = sl_sleeptimer_get_time ();
  dummy_agregate_buff[2] = 0xAAAA;
  dummy_agregate_buff[3] = 5;

  // Initialize connection indicator pin
  GPIO_PinModeSet (SERIAL_READY_PORT, SERIAL_READY_PIN, gpioModePushPull, 1);
  GPIO_PinOutClear (SERIAL_READY_PORT, SERIAL_READY_PIN);

  // Initialize wake-ip pin (USART)
  GPIO_PinModeSet (SERIAL_READY_PORT, SERIAL_REQUEST_PIN, gpioModeInputPull, 1);

  // Sends invitation string via USART
  uint8_t uart_hello_str[] =
    { "Hello EFR32BG22 - BLE Template - ver 1.0\n" };
  blocking_serial_write (uart_hello_str, strlen ((char*) uart_hello_str));
}

/***************************************************************************//**
 * Application Process Action.
 ******************************************************************************/
SL_WEAK void
app_process_action (void)
{
  // Checks hardware pins to allows serial communication
  blocking_serial_hw_control (is_client_connected);

  // Check if USART register contains valid data,
  // if not, go to the EM2 sleep mode
  if (blocking_serial_is_available ())
    {
      //Reads data from USART
      if (blocking_serial_read (temp_buffer, USART_NEWLINE_TERMINATOR) == true)
        {
          // Update SPP characteristic value
          update_spp_characteristic (temp_buffer);

          // Parse incoming data
          if (!parse_incoming_data_str ((char*) input_uart_buffer))
            blocking_serial_write ((uint8_t*) "JSON error\n",
                                   strlen ("JSON error\n"));
          else
            blocking_serial_write ((uint8_t*) "OK\n",
                                               strlen ("OK\n"));

          memset (temp_buffer, 0, SPP_BUFF_SIZE);
        }
    }
  else if (GPIO_PinInGet (SERIAL_READY_PORT, SERIAL_REQUEST_PIN))
    {
      // Let the CPU go to sleep if the system allows it.
      sl_power_manager_sleep ();
    }
}

/**************************************************************************//**
 * Controls the pins that allow communication through the serial port.
 *
 *****************************************************************************/
void
blocking_serial_hw_control (bool connected)
{
  switch (connected)
    {
    case true:
      GPIO_PinOutSet (SERIAL_READY_PORT, SERIAL_READY_PIN);
      break;

    case false:
      GPIO_PinOutClear (SERIAL_READY_PORT, SERIAL_READY_PIN);
      break;
    }
}

/**************************************************************************//**
 * Check if serial has valid data
 *
 *****************************************************************************/
bool
blocking_serial_is_available (void)
{
  return USART_StatusGet (USART1) & USART_STATUS_RXDATAV;
}

/**************************************************************************//**
 * Blocking USART write
 *
 *****************************************************************************/
bool
blocking_serial_write (uint8_t *buff, uint32_t buff_sz)
{
  uint8_t c;

  if (!buff_sz || buff == NULL)
    return false;

  while (buff_sz)
    {
      c = *buff++;
      USART_Tx (USART1, c);
      buff_sz--;
    }

  while (!(USART_StatusGet (USART1) & USART_STATUS_TXC))
    ;

  return true;
}

/**************************************************************************//**
 * Blocking USART read
 *
 *****************************************************************************/
size_t
blocking_serial_read (uint8_t *data, uint8_t terminator)
{
  int8_t c = 0;
  static uint8_t index = 0;

  /* Retrieve characters, print local echo and full line back */
  c = USART_Rx (USART1);
  if (c > 0)
    {
      if (c == terminator)
        {
          input_uart_buffer[index] = '\0';
          memcpy (data, input_uart_buffer, index);
          index = 0;

          return true;
        }
      else
        {
          if (index < BUFSIZE - 1)
            {
              input_uart_buffer[index] = c;
              index++;
            }
        }
    }

  return false;
}

/**************************************************************************//**
 * Setup and enable iBeacon advertising
 * (scannable & connectable)
 *
 *****************************************************************************/
static void
bcn_setup_adv_beaconing (void)
{
  sl_status_t sc;
  int16_t ret_power_min, ret_power_max;

  // Set 0 dBm maximum Transmit Power.
  sc = sl_bt_system_set_tx_power (SL_BT_CONFIG_MIN_TX_POWER, 0, &ret_power_min,
                                  &ret_power_max);

  // Create an advertising set.
  sc = sl_bt_advertiser_create_set (&advertising_set_handle);

  // Set custom advertising data.
  sc = sl_bt_advertiser_set_data (advertising_set_handle, 0,
                                  sizeof(bcn_beacon_adv_data),
                                  (uint8_t*) (&bcn_beacon_adv_data));

  // Set advertising parameters. 100ms advertisement interval.
  sc = sl_bt_advertiser_set_timing (advertising_set_handle, 1000, // min. adv. interval (milliseconds * 1.6)
                                    1000, // max. adv. interval (milliseconds * 1.6)
                                    0,       // adv. duration
                                    0);      // max. num. adv. events

  // Start advertising in user mode and disable connections.
  sc = sl_bt_advertiser_start (advertising_set_handle,
                               sl_bt_advertiser_user_data,
                               sl_bt_advertiser_connectable_scannable);

}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void
sl_bt_on_event (sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];
  uint8_t spp_recv_data[128];

  memset (spp_recv_data, 0, sizeof(spp_recv_data));

  switch (SL_BT_MSG_ID(evt->header))
    {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address (&address, &address_type);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value (gattdb_system_id, 0,
                                                    sizeof(system_id),
                                                    system_id);

      bcn_setup_adv_beaconing ();

      // Initialize SPP characteristic
      memset (input_uart_buffer, 0, SPP_BUFF_SIZE);

      // Update SPP characteristic value
      update_spp_characteristic ((uint8_t*) input_uart_buffer);

      break;

      // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:

      // Change flag status after connecting
      is_client_connected = true;
      break;

      // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:

      // Change flag status after connecting
      is_client_connected = false;

      // Start advertising in user mode and disable connections.
      sc = sl_bt_advertiser_start (advertising_set_handle,
                                   sl_bt_advertiser_user_data,
                                   sl_bt_advertiser_connectable_scannable);
      break;

      // -------------------------------
      // This event indicates that the value of an attribute in the local GATT
      // database was changed by a remote GATT client.
    case sl_bt_evt_gatt_server_attribute_value_id:

      // The value of the gattdb_spp characteristic was changed.
      if (gattdb_spp
          == evt->data.evt_gatt_server_characteristic_status.characteristic)
        {
          // Reading data from SPP characteristic and puts to the USART
          sc = read_spp_characteristic (spp_recv_data);
          blocking_serial_write (spp_recv_data, strlen ((char*) spp_recv_data));
        }

      // The value of the gattdb_timestamp characteristic was changed.
      if (gattdb_timestamp
          == evt->data.evt_gatt_server_characteristic_status.characteristic)
        {
          sc = read_current_timestamp_characteristic ();
        }

      break;

      // -------------------------------
      // This event occurs when the remote device enabled or disabled the
      // notification or read request.
    case sl_bt_evt_gatt_server_characteristic_status_id:

      if (gattdb_aggregate
          == evt->data.evt_gatt_server_characteristic_status.characteristic)
        {
          if (evt->data.evt_gatt_server_characteristic_status.client_config_flags
              & sl_bt_gatt_notification)
            {
              agregate_notify_flag = true;
            }
          else
            {
              agregate_notify_flag = false;
            }
        }

      break;

      //<<

    case sl_bt_evt_advertiser_timeout_id:
      // todo advertising timeout  (do nothing?!)
      break;

      // -------------------------------
      // Default event handler.
    default:
      break;
    }

}

/***************************************************************************//**
 * Reads current timestamp
 ******************************************************************************/
static sl_status_t
read_current_timestamp_characteristic (void)
{
  sl_status_t sc;
  sl_sleeptimer_timestamp_t timestamp = 0;
  uint8_t time_buff[4];
  size_t data_recv_len;

  // Read characteristic value.
  sl_bt_gatt_server_read_attribute_value (gattdb_timestamp, 0,
                                          sizeof(time_buff), &data_recv_len,
                                          time_buff);

  // Convert uint8 array to uint32
  timestamp = time_buff[0] | (time_buff[1] << 8) | (time_buff[2] << 16)
      | (time_buff[3] << 24);

  // Update time in RTC
  sc = sl_sleeptimer_set_time (timestamp);

  return sc;
}

/******************************************************************************
 * Updates the current timestamp characteristic (write).
 ******************************************************************************/
static sl_status_t
update_current_timestamp_characteristic (sl_sleeptimer_timestamp_t timestamp)
{
  sl_status_t sc;
  uint8_t time_buff[4];

  time_buff[0] = timestamp >> 24;
  time_buff[1] = timestamp >> 16;
  time_buff[2] = timestamp >> 8;
  time_buff[3] = timestamp;

  // Write attribute in the local GATT database.
  sc = sl_bt_gatt_server_write_attribute_value (gattdb_timestamp, 0,
                                                sizeof(time_buff), time_buff);
  return sc;
}

/******************************************************************************
 * Updates the current temperature characteristic (write).
 ******************************************************************************/
static sl_status_t
update_current_temperature_characteristic (uint8_t current_temperature)
{
  sl_status_t sc;

  // Write attribute in the local GATT database.
  sc = sl_bt_gatt_server_write_attribute_value (gattdb_temperature, 0,
                                                sizeof(current_temperature),
                                                &current_temperature);
  return sc;
}

/******************************************************************************
 * Updates the current humidity characteristic (write).
 ******************************************************************************/
static sl_status_t
update_current_humidity_characteristic (uint8_t current_humidity)
{
  sl_status_t sc;

  // Write attribute in the local GATT database.
  sc = sl_bt_gatt_server_write_attribute_value (gattdb_humidity, 0,
                                                sizeof(current_humidity),
                                                &current_humidity);
  return sc;
}

/******************************************************************************
 * Updates the battery level characteristic (write).
 ******************************************************************************/
static sl_status_t
update_battery_level_characteristic (uint16_t battery_level)
{
  sl_status_t sc;
  uint8_t batt_arr[2];

  UINT16_TO_BYTE1(1);

  batt_arr[0] = UINT16_TO_BYTE1(battery_level);
  batt_arr[1] = UINT16_TO_BYTE0(battery_level);

  // Write attribute in the local GATT database.
  sc = sl_bt_gatt_server_write_attribute_value (gattdb_battery_level, 0,
                                                sizeof(batt_arr), batt_arr);

  return sc;
}

/******************************************************************************
 * Convert data to byte array in format : [2B-ID][4B-TS][2B-Type][4B-Data]
 ******************************************************************************/
void
convert_overdue_data_to_byte_array (uint32_t *in_data, uint8_t *out_data)
{
  out_data[0] = in_data[0] >> 24;
  out_data[1] = in_data[0] >> 16;
  out_data[2] = in_data[0] >> 8;
  out_data[3] = in_data[0];

  out_data[4] = in_data[1] >> 24;
  out_data[5] = in_data[1] >> 16;
  out_data[6] = in_data[1] >> 8;
  out_data[7] = in_data[1];

  out_data[8] = in_data[2] >> 8;
  out_data[9] = in_data[2];

  out_data[10] = in_data[3] >> 24;
  out_data[11] = in_data[3] >> 16;
  out_data[12] = in_data[3] >> 8;
  out_data[13] = in_data[3];
}

/******************************************************************************
 * Updates the overdue records (notification).
 ******************************************************************************/
static sl_status_t
update_agregate_characteristic (uint32_t *agregate_data_array)
{
  sl_status_t sc;
  uint8_t temp_buff[14];

  // Convert data to byte array
  convert_overdue_data_to_byte_array (agregate_data_array, temp_buff);

  // Write attribute in the local GATT database.
  sc = sl_bt_gatt_server_notify_all (gattdb_aggregate, sizeof(temp_buff),
                                     temp_buff);
  return sc;
}

/******************************************************************************
 * Writes to the SPP characteristic.
 ******************************************************************************/
static sl_status_t
update_spp_characteristic (uint8_t *data)
{
  sl_status_t sc;

  // Write attribute in the local GATT database.
  sc = sl_bt_gatt_server_write_attribute_value (gattdb_spp, 0, SPP_BUFF_SIZE,
                                                data);
  return sc;
}

/******************************************************************************
 * Reads from the SPP characteristic.
 ******************************************************************************/
static sl_status_t
read_spp_characteristic (uint8_t *data)
{
  size_t data_recv_len;

  // Read characteristic value.
  return sl_bt_gatt_server_read_attribute_value (gattdb_spp, 0,
  SPP_BUFF_SIZE,
                                                 &data_recv_len, data);
}

/******************************************************************************
 * Check if incoming data string is valid.
 ******************************************************************************/
bool
check_if_command_is_valid (char *inJson, size_t len, char *command,
                           uint8_t *value)
{
  bool retValue = false;
  const cJSON *hash = NULL;

  cJSON *jsonObj = cJSON_ParseWithLength (inJson, len);
  if (jsonObj == NULL)
    {
      return retValue;
    }

  hash = cJSON_GetObjectItemCaseSensitive (jsonObj, command);

  if (cJSON_IsString (hash) && (hash->valuestring != NULL))
    {
      memcpy (value, hash->valuestring, strlen (hash->valuestring));
      retValue = true;
    }
  else
    {
      value = NULL;
      retValue = false;
    }

  cJSON_Delete (jsonObj);

  return retValue;
}

/******************************************************************************
 * Parse incoming data string
 ******************************************************************************/
bool
parse_incoming_data_str (char *rxPacket)
{
  uint8_t value_array[32];

  if (check_if_command_is_valid (rxPacket, strlen (rxPacket), SERIAL_TEMPERATURE_CMD,
                                 value_array))
    {
      update_current_temperature_characteristic (
          (uint8_t) atoi ((char*) value_array));
      //blocking_serial_write (value_array, strlen ((char*) value_array));
      return true;
    }

  if (check_if_command_is_valid (rxPacket, strlen (rxPacket), SERIAL_HUMIDITY_CMD,
                                 value_array))
    {
      update_current_humidity_characteristic (
          (uint8_t) atoi ((char*) value_array));
      //blocking_serial_write (value_array, strlen ((char*) value_array));
      return true;
    }

  if (check_if_command_is_valid (rxPacket, strlen (rxPacket), SERIAL_BATT_LVL_CMD,
                                 value_array))
    {
      update_battery_level_characteristic (atoi((char *)value_array));
      //blocking_serial_write (value_array, strlen ((char*) value_array));
      return true;
    }

  // ...
  return false;
}

/******************************************************************************
 * Convert from unsigned int8 array to float
 ******************************************************************************/
float
convert_array_to_float(uint8_t *array)
{
  FLOATUNION_t converter;
  memcpy(converter.bytes, array, sizeof(float));

  return converter.number;
}
