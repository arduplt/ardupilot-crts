#include "AP_Ticommunication_params.h"
#include "AP_Ticommunication.h"

// Sonin Aero Seasoning
// table of user settable parameters
const AP_Param::GroupInfo AP_Ticommunication_params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLite-I2C,5:PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini/Plus-Serial,21:LidarLightV3HP,22:PWM,23:BlueRoboticsPing,24:UAVCAN,25:BenewakeTFmini/Plus-I2C,27:BenewakeTF03
    // @User: Standard
    AP_GROUPINFO("TYPE",    1, AP_Ticommunication_params, type, 0),

    // @Param: PIN
    // @DisplayName: TI pin
    // @Description: Analog or PWM input pin that TI is connected to.  Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input
    // @Values: -1:Not Used,11:PX4-airspeed port, 15:Pixhawk-airspeed port,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6
    // @User: Standard
    AP_GROUPINFO("PIN",     2, AP_Ticommunication_params, pin, -1),

    AP_GROUPEND
};

AP_Ticommunication_params::AP_Ticommunication_params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
