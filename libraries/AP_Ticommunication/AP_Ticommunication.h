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
#include <AP_Math/AP_Math.h>

/*
*This library is for communication with the Texas Instruments microcontroller, connected via Serial
* 
*
 */

class AP_Ticommunication {
public:


	//AP_Ticommunication();  // Constructor
    // Initializes backend  
    void init(void);          // chkd

    // Requests backend to update the frontend. Should be called at 10Hz.
    void update(void);   // main loop function Sonin Aero ************************************************
	
	
	
	
	
	//  check if message is valid
	
	bool check_message();
    
    // Returns the State of charge
    int get_soc() ; 
	
	// Returns remaining Flight time
	
	int get_remaining_flight_time () ;
	
	// Send heartbeat from Pixhawk to Ti
	
	void send_heartbeat();


    bool is_healthy();


private:
    // port
    AP_HAL::UARTDriver *port;
    // Front End Parameters
    //AP_Int8 type;
	
	char buffer[10];   // used to store incoming values
	
	uint32_t last_updated_ms;

    // Tracking backends
    
 
};



namespace AP {
    AP_Ticommunication *Ticommunication();
};

 
 
 //**********************************************************************DELETE******************************************************************
 
