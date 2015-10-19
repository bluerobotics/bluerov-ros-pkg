/**
* File: navio_interface.cpp
* Author: J. Neilan <jimbolysses@gmail.com>
* Date: October 2015
* Description: Interface class and impl for Navio+ Raspberry Pi shield. Users can pull IMU, Barometer,
*   GPS, ADC, and AHRS fused IMU data via getter methods. Users can also set servo values and control
*   output RC or autonomous signals.
*/

#include "../../thirdparty/navio/gpio.h"
#include "../../thirdparty/navio/PCA9685.h"
#include "../../thirdparty/navio/MPU9250.h"
#include "../../thirdparty/navio/Ublox.h"
#include "../../thirdparty/navio/MS5611.h"
#include "../../thirdparty/navio/ADS1115.h"

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
    EServoChannels			servoChannels;

    // Methods
    int Initialize();
    void SetFrequency( int freqIn );
    void SendPWM( int channelNumIn, double servoValIn );

    std::vector<float> GetIMU();
    std::vector<float> GetGPS()

    std::vector<float> GetBaro();
    std::vector<float> GetAHRS();
    std::vector<float> GetADC();

 private:
    // Attributes
    PCA9685				_pwm;
    MPU9250 				_imu;

    Ublox				_gps;
    MS5611				_baro;

    ADS1115				_adc;
    uint16_t				_muxes[];

    float 				_results;
    static const uint8_t		_outputEnablePin;

    Navio::Pin				_pin;

    // Methods
};

NavioInterface::NavioInterface() : _outputEnablePin( Navio::RPI_GPIO_27 ), _pin( _outputEnablePin )
{
  _muxes = { ADS1115_MUX_P0_NG, ADS1115_MUX_P1_NG, ADS1115_MUX_P2_NG, ADS1115_MUX_P3_NG );
  _results[ ARRAY_SIZE( muxes ) ] = { 0.0f };
}

NavioInterface::~NavioInterface()
{

}

int NavioInterface::Initialize()
{
  int ret = _pin.init();
  if( ret )
  {
    _pin.setMode( Navio::Pin::GpioModeOutput );
    _pin.write( 0 );// drive output enable low

    // Servos
    _pwm.initialize();

    // IMU
    _imu.initialize();

    // Barometer
    _baro.initialize();

    // ADC
    _adc.setMode( ADS1115_MODE_SINGLESHOT );
    _adc.setRate( ADS1115_RATE_60 );

  }else
  {
    ROS_ERROR( "Output enable not set. Are you root? Use sudo ..." );
  }

  return ret;
}

void NavioInterface::SetFrequency( int freqIn )
{
  _pwm.setFrequency( freqIn );
}

void NavioInterface::SendPWM( int channelNumIn, double servoValIn )
{
  _pwm.setPWMmS( channelNumIn, servoValIn );
}

std::vector<float> NavioInterface::GetIMU()
{
  float ax, ay, az, gx, gy, gz, mx, my, mz;

  static std::vector<float> imuData( 0 );
  imuData.clear();
  imuData.empty();

  _imu_getMotion9( &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz );

  imuData.push_back( ax );
  imuData.push_back( ay );
  imuData.push_back( az );

  imuData.push_back( gz );
  imuData.push_back( gy );
  imuData.push_back( gz );

  imuData.push_back( mx );
  imuData.push_back( my );
  imuData.push_back( mz );

  return imuData;
}

std::vector<double> NavioInterface::GetGPS()
{
  static std::vector<double> gpsData( 0 );
  gpsData.clear();

  return gpsData;
}

std::vector<float> NavioInterface::GetBaro()
{
  static std::vector<float> baroData( 0 );
  baroData.clear();
  baroData.empty();

  _baro.refreshPressure();
  sleep( 10 ); // waiting for pressure data to be ready
  _baro.readPressure();

  _baro.refreshTemperature();
  sleep( 10 ); // Waiting for temperature data to be ready
  _baro.readTemperature();

  _baro.calculatePressureAndTemperature();

  baroData.push_back( _baro.getTemperature() );
  baroData.push_back( _baro.getPressure() );

  return baroData;
}

std::vector<float> NavinInterface::GetAHRS()
{
  static std::vector<float> ahrsData( 0 );
  ahrsData.clear();


  return ahrsData;
}

std::vector<float> NavioInterface::GetADC()
{
  static std::vector<float> adcData( 0 );
  adcData.clear();

  float conversion;

  for( size_t i = 0; i < ARRAY_SIZE( muxes ); ++i )
  {
    _adc.setMultiplexer( muxes[i] );

    conversion = _adc.getMilliVolts();
    _results[i] = conversion;

    for( size_t j = 0; j < ARRAY_SIZE( muxes ); ++j )
    {
      adcData.push_back( ( results[j] / 1000 );
    }
  }
  return adcData;
}
