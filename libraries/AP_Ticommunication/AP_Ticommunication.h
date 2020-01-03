/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Sonin Aero seasoning
 
#pragma once


#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_HAL/AP_HAL.h>

// Time in milliseconds before we declare the Ticommunication to be "unhealthy"
#Ticommunicationne HEALTHY_LAST_RECEIVED 3000


/*
*This library is for communication with the Texas Insttruments microcontroller, connected via Serial
* 
*
 */

class AP_Ticommunication {
public:

    // Initializes backend  
    void init(void);          // chkd

    // Requests backend to update the frontend. Should be called at 10Hz.
    void communicate_Ti();   // main loop function Sonin Aero ************************************************
	
	char input_buffer[10];   // used to store incoming values
	
	//  check if message is valid
	
	bool check_message();
    
    // Returns the State of charge
    int get_soc() ; 
	
	// Returns remaining Flight time
	
	int get_remaining_flight_time () ;
	
	// Send heartbeat from Pixhawk to Ti
	
	void send_heartbeat();

    // returns enabled state of Ticommunication
    bool enabled() const { return type != Ticommunication_COMMUNICATION_TYPE_NONE; }

    bool is_healthy() const;

protected:


    Ticommunication_State state;

private:
    // port
    AP_HAL::UARTDriver *port;
    // Front End Parameters
    AP_Int8 type;

    // Tracking backends
    AP_Ticommunication_Backend *backend;
 
};



class AP_Ticommunication_Backend {
public:    
    // Constructor with initialization
    AP_Ticommunication_Backend(AP_Ticommunication &_frontend);

    // Virtual destructor that Ticommunication backends can override 
    virtual ~AP_Ticommunication_Backend(void) {}

    // Update the state structure
    virtual void update() = 0;

protected:
    // Copies internal state to the frontend state
    void copy_to_frontend();

    // Semaphore for access to shared frontend data
    HAL_Semaphore sem;

    // Internal state for this driver (before copying to frontend)
    Ticommunication_State internal_state;

    int8_t get_uavcan_node_id(void) const;
    float get_coef1(void) const;
    float get_coef2(void) const;

private:
    AP_Ticommunication &frontend;
};


class AP_Ticommunication_Serial: public AP_Ticommunication_Backend {
    
public:
    // Constructor with initialization
    AP_Ticommunication_Serial(AP_Ticommunication &_frontend);

    // Update the state structure
    void update() override;

private:
    AP_HAL::UARTDriver *port;
    void parse_realtime_data();
    bool read_incoming_realtime_data();
    void send_request(uint8_t table, uint16_t first_offset, uint16_t last_offset);
    uint8_t read_byte_CRC32();
    uint32_t CRC32_compute_byte(uint32_t inCrc32, uint8_t data);
    float f_to_k(float temp_f) { return (temp_f + 459.67f) * 0.55556f; };
    
    // Serial protocol Variables
    uint32_t checksum;
    uint8_t step;
    uint8_t response_flag;
    uint16_t message_counter;
    uint32_t last_response_ms;

    // confirmed that last command was ok
    bool last_command_confirmed;

    // Command Response Codes
  /*  enum response_codes {
        RESPONSE_WRITE_OK =0x00,
        RESPONSE_REALTIME_DATA,
        RESPONSE_PAGE_DATA,
        RESPONSE_CONFIG_ERROR,
        RESPONSE_PAGE10_OK,
        RESPONSE_CAN_DATA,
        // Error Responses
        ERR_UNDERRUN = 0X80,
        ERR_OVERRUN,
        ERR_CRC_FAILURE,
        ERR_UNRECOGNIZED_COMMAND,
        ERR_OUT_OF_RANGE,
        ERR_SERIAL_BUSY,
        ERR_FLASH_LOCKED,
        ERR_SEQ_FAIL_1,
        ERR_SEQ_FAIL_2,
        ERR_CAN_QUEUE_FULL,
        ERR_CAN_TIMEOUT,
        ERR_CAN_FAILURE,
        ERR_PARITY,
        ERR_FRAMING,
        ERR_SERIAL_NOISE,
        ERR_TXMODE_RANGE,
        ERR_UNKNOWN
    };*/
    
    // Realtime Data Table Locations
  /*  enum realtime_data {
        PW1B = 2,
        PW1_LSB,
        RPMB = 6,
        RPM_LSB,
        ADVANCEB,
        ADVANCE_LSB,
        ENGINE_BM = 11,
        BAROMETERB = 16,
        BAROMETER_LSB,
        MAPB,
        MAP_LSB,
        MATB,
        MAT_LSB,
        CHTB,
        CHT_LSB,
        TPSB,
        TPS_LSB,
        AFR1B = 28,
        AFR1_LSB,
        AFR2B,
        AFR2_LSB,
        DWELLB = 62,
        DWELL_LSB,
        LOAD = 66,
        FUEL_PRESSUREB = 128,
        FUEL_PRESSURE_LSB, 
        // Helpers used when sending request
        RT_FIRST_OFFSET = PW1B,
        RT_LAST_OFFSET = FUEL_PRESSURE_LSB
    };*/
};


/***************
 *
 * Status enums
 *
 ***************/


// Stores the current state read by the Ticommunication system
// All backends are required to fill in this state structure
struct Ticommunication_State {
    // When this structure was last updated (milliseconds)
    uint32_t last_updated_ms;

    // Current overall engine state
    Engine_State engine_state;
 
    // If there is an error that does not fit other error types
    bool general_error;

    // Error/status fields 
    Crankshaft_Sensor_Status crankshaft_sensor_status;
    Temperature_Status temperature_status;
    Fuel_Pressure_Status fuel_pressure_status;
    Oil_Pressure_Status oil_pressure_status;
    Detonation_Status detonation_status;
    Misfire_Status misfire_status;
    Debris_Status debris_status;

    // Engine load (percent)
    uint8_t engine_load_percent;
    
    // Engine speed (revolutions per minute)
    uint32_t engine_speed_rpm;

    // Spark dwell time (milliseconds)
    float spark_dwell_time_ms;

    // Atmospheric (barometric) pressure (kilopascal)
    float atmospheric_pressure_kpa;

    // Engine intake manifold pressure (kilopascal)
    float intake_manifold_pressure_kpa;

    // Engine intake manifold temperature (kelvin)
    float intake_manifold_temperature;

    // Engine coolant temperature (kelvin)
    float coolant_temperature;
    
    // Oil pressure (kilopascal)
    float oil_pressure;

    // Oil temperature (kelvin)
    float oil_temperature;

    // Fuel pressure (kilopascal)
    float fuel_pressure;

    // Instant fuel consumption estimate, which 
    // should be low-pass filtered in order to prevent aliasing effects.
    // (centimeter^3)/minute.
    float fuel_consumption_rate_cm3pm;

    // Estimate of the consumed fuel since the start of the engine (centimeter^3)
    // This variable is reset when the engine is stopped.
    float estimated_consumed_fuel_volume_cm3;

    // Throttle position (percent)
    uint8_t throttle_position_percent;

    // The index of the publishing ECU.
    uint8_t ecu_index;

    // Spark plug activity report.
    // Can be used during pre-flight tests of the spark subsystem.
    // Use case is that usually on double spark plug engines, the 
    // engine switch has the positions OFF-LEFT-RIGHT-BOTH-START.
    // Gives pilots the possibility to test both spark plugs on 
    // ground before takeoff.
    Spark_Plug_Usage spark_plug_usage;

    // Status for each cylinder in the engine
    Cylinder_Status cylinder_status[ENGINE_MAX_CYLINDERS];

};


namespace AP {
    AP_Ticommunication *Ticommunication();
};

 
 
 //**********************************************************************DELETE******************************************************************
 
 enum class Engine_State : uint8_t {
    STOPPED  = 0,
    STARTING = 1,
    RUNNING  = 2,
    FAULT    = 3
};

enum class Crankshaft_Sensor_Status : uint8_t {
    NOT_SUPPORTED = 0,
    OK            = 1,
    ERROR         = 2
};

enum class Temperature_Status : uint8_t {
    NOT_SUPPORTED       = 0,
    OK                  = 1,
    BELOW_NOMINAL       = 2,
    ABOVE_NOMINAL       = 3,
    OVERHEATING         = 4,
    EGT_ABOVE_NOMINAL   = 5
};

enum class Fuel_Pressure_Status : uint8_t {
    NOT_SUPPORTED        = 0,
    OK                   = 1,
    BELOW_NOMINAL        = 2,
    ABOVE_NOMINAL        = 3
};

enum class Oil_Pressure_Status : uint8_t {
    OIL_PRESSURE_STATUS_NOT_SUPPORTED = 0,
    OIL_PRESSURE_OK                   = 1,
    OIL_PRESSURE_BELOW_NOMINAL        = 2,
    OIL_PRESSURE_ABOVE_NOMINAL        = 3
};

enum class Detonation_Status : uint8_t {
    NOT_SUPPORTED  = 0,
    NOT_OBSERVED   = 1,
    OBSERVED       = 2
};

enum class Misfire_Status : uint8_t {
    NOT_SUPPORTED = 0,
    NOT_OBSERVED  = 1,
    OBSERVED      = 2
};

enum class Debris_Status : uint8_t {
    NOT_SUPPORTED = 0,
    NOT_DETECTED  = 1,
    DETECTED      = 2
};

enum class Spark_Plug_Usage : uint8_t {
    SINGLE        = 0,
    FIRST_ACTIVE  = 1,
    SECOND_ACTIVE = 2,
    BOTH_ACTIVE   = 3
};


/***************
 * Status structs.
 * Ticommunications may not provide all data in the message, therefore, the following guidelines should be followed.
 * All integer fields are required unless stated otherwise.
 * All floating point fields are optional unless stated otherwise; unknown/unapplicable fields will be NaN.
 ***************/


// Per-cylinder status struct
struct Cylinder_Status {
    // Cylinder ignition timing (angular degrees of the crankshaft)
    float ignition_timing_deg;

    // Fuel injection time (millisecond)
    float injection_time_ms;

    // Cylinder head temperature (CHT) (kelvin)
    float cylinder_head_temperature;

    // Exhaust gas temperature (EGT) (kelvin)
    // If this cylinder is not equipped with an EGT sensor - will be NaN
    // If there is a single shared EGT sensor, will be the same value for all cylinders
    float exhaust_gas_temperature;

    // Estimated lambda coefficient (dimensionless ratio)
    // Useful for monitoring and tuning purposes.
    float lambda_coefficient;
};


