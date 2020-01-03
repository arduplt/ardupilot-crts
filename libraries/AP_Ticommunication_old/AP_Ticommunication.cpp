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
 
#include "AP_Ticommunication.h"
#include "AP_Ticommunication_analog.h"
#include "AP_Ticommunication_PulsedLightLRF.h"

#include "AP_Ticommunication_MaxsonarSerialLV.h"


#include "AP_Ticommunication_LightWareSerial.h"

#include "AP_Ticommunication_MAVLink.h"


#include "AP_Ticommunication_NMEA.h"


#include "AP_Ticommunication_PWM.h"
#include "AP_Ticommunication_BLPing.h"
#include "AP_Ticommunication_UAVCAN.h"


#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo Ticommunication::var_info[] = {

	// @Group: 1_
	// @Path: AP_Ticommunication_Params.cpp
	AP_SUBGROUPINFO(params[0], "1_", 25, Ticommunication, AP_Ticommunication_Params),

    // @Group: 1_
    // @Path: AP_Ticommunication_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  57, Ticommunication, backend_var_info[0]),

#if Ticommunication_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_Ticommunication_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, Ticommunication, AP_Ticommunication_Params),

    // @Group: 2_
    // @Path: AP_Ticommunication_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  58, Ticommunication, backend_var_info[1]),
#endif
    
    AP_GROUPEND
};


/*
  initialise the Ticommunication class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  Ticommunications.
 */
void Ticommunication::init(enum Rotation orientation_default)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }

    convert_params();

    // set orientation defaults
    for (uint8_t i=0; i<Ticommunication_MAX_INSTANCES; i++) {
        params[i].orientation.set_default(orientation_default);
    }

    for (uint8_t i=0, serial_instance = 0; i<Ticommunication_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }

        // initialise status
        state[i].status = Status::NotConnected;
        state[i].range_valid_count = 0;
    }
}

/*
  update Ticommunication state for all instances. This should be called at
  around 10Hz by main loop
 */
void Ticommunication::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if ((Type)params[i].type.get() == Type::NONE) {
                // allow user to disable a Ticommunication at runtime
                state[i].status = Status::NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            drivers[i]->update();
        }
    }
#ifndef HAL_BUILD_AP_PERIPH
    Log_RFND();
#endif
}

bool Ticommunication::_add_backend(AP_Ticommunication_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_instances == Ticommunication_MAX_INSTANCES) {
        AP_HAL::panic("Too many RANGERS backends");
    }

    drivers[num_instances++] = backend;
    return true;
}

/*
  detect if an instance of a Ticommunication is connected. 
 */
void Ticommunication::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
    const Type _type = (Type)params[instance].type.get();
    switch (_type) {

    case Type::LWSER:                                                                    // Sonin Aero: we will base our program on Lightwareserial
        if (AP_Ticommunication_LightWareSerial::detect(serial_instance)) {
            drivers[instance] = new AP_Ticommunication_LightWareSerial(state[instance], params[instance], serial_instance++);
        }
        break;
 
    case Type::MAVLink:
#ifndef HAL_BUILD_AP_PERIPH
        if (AP_Ticommunication_MAVLink::detect()) {
            drivers[instance] = new AP_Ticommunication_MAVLink(state[instance], params[instance]);
        }
#endif
        break;
    case Type::MBSER:
        if (AP_Ticommunication_MaxsonarSerialLV::detect(serial_instance)) {
            drivers[instance] = new AP_Ticommunication_MaxsonarSerialLV(state[instance], params[instance], serial_instance++);
        }
        break;

    case Type::NMEA:
        if (AP_Ticommunication_NMEA::detect(serial_instance)) {
            drivers[instance] = new AP_Ticommunication_NMEA(state[instance], params[instance], serial_instance++);
        }
        break;
    case Type::WASP:
        if (AP_Ticommunication_Wasp::detect(serial_instance)) {
            drivers[instance] = new AP_Ticommunication_Wasp(state[instance], params[instance], serial_instance++);
        }
        break;
 
    case Type::PWM:
#ifndef HAL_BUILD_AP_PERIPH
        if (AP_Ticommunication_PWM::detect()) {
            drivers[instance] = new AP_Ticommunication_PWM(state[instance], params[instance], estimated_terrain_height);
        }
#endif
        break;
 

    default:
        break;
    }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);
    }
}

AP_Ticommunication_Backend *Ticommunication::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == Type::NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

Ticommunication::Status Ticommunication::status_orient(enum Rotation orientation) const
{
    AP_Ticommunication_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return Status::NotConnected;
    }
    return backend->status();
}

void Ticommunication::handle_msg(const mavlink_message_t &msg)
{
    uint8_t i;
    for (i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && ((Type)params[i].type.get() != Type::NONE)) {
          drivers[i]->handle_msg(msg);
        }
    }
}

// return true if we have a range finder with the specified orientation
bool Ticommunication::has_orientation(enum Rotation orientation) const
{
    return (find_instance(orientation) != nullptr);
}

// find first range finder instance with the specified orientation
AP_Ticommunication_Backend *Ticommunication::find_instance(enum Rotation orientation) const
{
    // first try for a Ticommunication that is in range
    for (uint8_t i=0; i<num_instances; i++) {
        AP_Ticommunication_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->orientation() == orientation &&
            backend->status() == Status::Good) {
            return backend;
        }
    }
    // if none in range then return first with correct orientation
    for (uint8_t i=0; i<num_instances; i++) {
        AP_Ticommunication_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->orientation() == orientation) {
            return backend;
        }
    }
    return nullptr;
}

uint16_t Ticommunication::distance_cm_orient(enum Rotation orientation) const
{
    AP_Ticommunication_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->distance_cm();
}

uint16_t Ticommunication::voltage_mv_orient(enum Rotation orientation) const
{
    AP_Ticommunication_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->voltage_mv();
}

int16_t Ticommunication::max_distance_cm_orient(enum Rotation orientation) const
{
    AP_Ticommunication_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->max_distance_cm();
}

int16_t Ticommunication::min_distance_cm_orient(enum Rotation orientation) const
{
    AP_Ticommunication_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->min_distance_cm();
}

int16_t Ticommunication::ground_clearance_cm_orient(enum Rotation orientation) const
{
    AP_Ticommunication_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->ground_clearance_cm();
}

bool Ticommunication::has_data_orient(enum Rotation orientation) const
{
    AP_Ticommunication_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return false;
    }
    return backend->has_data();
}

uint8_t Ticommunication::range_valid_count_orient(enum Rotation orientation) const
{
    AP_Ticommunication_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->range_valid_count();
}

const Vector3f &Ticommunication::get_pos_offset_orient(enum Rotation orientation) const
{
    AP_Ticommunication_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return pos_offset_zero;
    }
    return backend->get_pos_offset();
}

uint32_t Ticommunication::last_reading_ms(enum Rotation orientation) const
{
    AP_Ticommunication_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->last_reading_ms();
}

MAV_DISTANCE_SENSOR Ticommunication::get_mav_distance_sensor_type_orient(enum Rotation orientation) const
{
    AP_Ticommunication_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
    return backend->get_mav_distance_sensor_type();
}

// Write an RFND (Ticommunication) packet
void Ticommunication::Log_RFND()
{
    if (_log_rfnd_bit == uint32_t(-1)) {
        return;
    }

    AP_Logger &logger = AP::logger();
    if (!logger.should_log(_log_rfnd_bit)) {
        return;
    }

    for (uint8_t i=0; i<Ticommunication_MAX_INSTANCES; i++) {
        const AP_Ticommunication_Backend *s = get_backend(i);
        if (s == nullptr) {
            continue;
        }

        const struct log_RFND pkt = {
                LOG_PACKET_HEADER_INIT(LOG_RFND_MSG),
                time_us      : AP_HAL::micros64(),
                instance     : i,
                dist         : s->distance_cm(),
                status       : (uint8_t)s->status(),
                orient       : s->orientation(),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}

bool Ticommunication::prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const
{
    for (uint8_t i = 0; i < Ticommunication_MAX_INSTANCES; i++) {
        if (((Type)params[i].type.get() != Type::NONE) && (drivers[i] == nullptr)) {
          hal.util->snprintf(failure_msg, failure_msg_len, "Ticommunication %d was not detected", i + 1);
          return false;
        }
    }

    return true;
}

Ticommunication *Ticommunication::_singleton;

namespace AP {

Ticommunication *Ticommunication()
{
    return Ticommunication::get_singleton();
}

}
