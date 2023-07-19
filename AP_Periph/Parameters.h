#pragma once

#include <AP_Common/AP_Common.h>
#include "MAVLink.h"

// Global parameter class.
//
class Parameters {
public:
    static const uint16_t k_format_version = 2;

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_gps,
        k_param_compass,
        k_param_can_node,
        k_param_can_baudrate0,
        k_param_baro,
        k_param_buzz_volume,
        k_param_led_brightness,
        k_param_airspeed,
        k_param_rangefinder,
        k_param_flash_bootloader,
        k_param_rangefinder_baud,
        k_param_adsb_baudrate,
        k_param_hardpoint_id,
        k_param_hardpoint_rate,
        k_param_baro_enable,
        k_param_battery,
        k_param_debug,
        k_param_serial_number,
        k_param_adsb_port,
        k_param_servo_channels,
        k_param_rangefinder_port,
        k_param_notify,
        k_param_logger,
        k_param_log_bitmask,
        k_param_can_baudrate1,
        k_param_can_baudrate2,
        k_param_can_protocol0,
        k_param_can_protocol1,
        k_param_can_protocol2,
        k_param_sysid_this_mav,
        k_param_serial_manager,
        k_param_gps_mb_only_can_port,
        k_param_scripting,
        k_param_can_fdmode,
        k_param_can_fdbaudrate0,
        k_param_can_fdbaudrate1,
        k_param_serial_i2c_mode,
        k_param_can_terminator0,
        k_param_can_terminator1,
        k_param_can_node_stats,
        k_param_cubeid_fw_update_enabled,
        k_param_gps_base,
    };

    AP_Int16 format_version;
    AP_Int16 can_node;
    
    AP_Int32 can_baudrate[HAL_NUM_CAN_IFACES];

#ifdef HAL_PERIPH_ENABLE_BARO
    AP_Int8 baro_enable;
#endif
#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)
    AP_Int8 flash_bootloader;
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    AP_Int32 rangefinder_baud;
    AP_Int8 rangefinder_port;
#endif

#ifdef HAL_PERIPH_ENABLE_ADSB
    AP_Int32 adsb_baudrate;
    AP_Int8 adsb_port;
#endif

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    AP_Int16 hardpoint_id;
    AP_Int8 hardpoint_rate;
#endif

    AP_Int8 debug;

    AP_Int32 serial_number;

    AP_Int16 sysid_this_mav;

    AP_Int8 can_fdmode;
    AP_Int32 can_fdbaudrate[HAL_NUM_CAN_IFACES];
    AP_Int8 serial_i2c_mode;
    AP_Int8 can_terminator[HAL_NUM_CAN_IFACES];
    AP_Int8 node_stats;
    AP_Int8 cubeid_fw_update_enabled;

    Parameters() {}
};

extern const AP_Param::Info var_info[];
