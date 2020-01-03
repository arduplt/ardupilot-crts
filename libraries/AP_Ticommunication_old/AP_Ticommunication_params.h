#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_Ticommunication_params {                       // Class for Ti_Communication parameters  Sonin Aero
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_Ticommunication_Params(void);

	AP_Int8  type;                                     // Type of connection used Sonin Aero
	
    AP_Int8  pin;                                      // Variable to define the input pin that the Texas Instruments micro is connected to // Sonin Aero
    
};
