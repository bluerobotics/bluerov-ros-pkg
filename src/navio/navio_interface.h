/**
* File: navio_interface.h
* Author: J. Neilan <jimbolysses@gmail.com>
* Date: October 2015
* Description: Interface class and impl for Navio+ Raspberry Pi shield. Users can p$
*   GPS, ADC, and AHRS fused IMU data via getter methods. Users can also set servo $
*   output RC or autonomous signals.
*/

#include "../../thirdparty/navio/gpio.h"
#include "../../thirdparty/navio/PCA9685.h"
#include "../../thirdparty/navio/MPU9250.h"
#include "../../thirdparty/navio/Ublox.h"
#include "../../thirdparty/navio/MS5611.h"
#include "../../thirdparty/navio/ADS1115.h"

#define ARRAY_SIZE(array) (sizeof(array)/sizeof(array[0]))

enum EServoChannels
{
    //Output 1 on the Navio is PCA9685 channel 3, and so forth
    NavioRCOutput1 = 3,
    NavioRCOutput2,
    NavioRCOutput3,
    NavioRCOutput4,
    NavioRCOutput5,
    NavioRCOutput6,
    NavioRCOutput7,
    NavioRCOutput8,
    NavioRCOutput9,
    NavioRCOutput10,
    NavioRCOutput11,
    NavioRCOutput12,
    NavioRCOutput13
};

class NavioInterface
{
 public:
    NavioInterface();
    virtual ~NavioInterface();

    // Attributes
    EServoChannels                      servoChannels;

    // Methods
    int Initialize();
    void SetFrequency( int freqIn );
    void SendPWM( int channelNumIn, double pwmValIn );

    std::vector<float> GetIMU();
    std::vector<double> GetGPS();

    std::vector<float> GetBaro();
    std::vector<float> GetAHRS();
    std::vector<float> GetADC();

 private:
    // Attributes
    PCA9685                             _pwm;
    MPU9250                             _imu;

    Ublox                               _gps;
    MS5611                              _baro;

    ADS1115                             _adc;
    uint16_t                            _muxes[4];

    float                               _results[];
    static const uint8_t                _outputEnablePin;

    Navio::Pin                          _pin;

    // Methods
};
