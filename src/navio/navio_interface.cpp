/**
* File: navio_interface.cpp
* Author: J. Neilan <jimbolysses@gmail.com>
* Date: October 2015
* Description: Interface class and impl for Navio+ Raspberry Pi shield. Users can pull IMU, Barometer,
*   GPS, ADC, and AHRS fused IMU data via getter methods. Users can also set servo values and control
*   output RC or autonomous signals.
*/

#include "navio_interface.h"

// Init class constants
const uint8_t NavioInterface::_outputEnablePin = RPI_GPIO_27;

NavioInterface::NavioInterface() : _pin( _outputEnablePin )
{
  auto init = std::initializer_list<uint16_t>({ADS1115_MUX_P0_NG, ADS1115_MUX_P1_NG, ADS1115_MUX_P2_NG, ADS1115_MUX_P3_NG });
  std::copy( init.begin(), init.end(), _muxes );

//  _muxes = { ADS1115_MUX_P0_NG, ADS1115_MUX_P1_NG, ADS1115_MUX_P2_NG, ADS1115_MUX_P3_NG };
  _results[ ARRAY_SIZE( _muxes ) ] = { 0.0f };
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
    _adc.setRate( ADS1115_RATE_860 );

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

void NavioInterface::SendPWM( int channelNumIn, double pwmValIn )
{
  _pwm.setPWMmS( channelNumIn, pwmValIn );
}

std::vector<float> NavioInterface::GetIMU()
{
  float ax, ay, az, gx, gy, gz, mx, my, mz;

  static std::vector<float> imuData( 0 );
  imuData.clear();
  imuData.empty();

  _imu.getMotion9( &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz );

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
  gpsData.empty();

  if( _gps.decodeSingleMessage( Ublox::NAV_POSLLH, gpsData ) == 1 )
  {
    // iTow
    gpsData[0] /= 1000; 
    // Longitude
    gpsData[1] /= 10000000;
    // Latitude
    gpsData[2] /= 10000000;
    // Altitude
    gpsData[3] /= 1000;
  }else
  {
    ROS_INFO( "GPS Data NOT Captured" );
  }

  if( _gps.decodeSingleMessage( Ublox::NAV_STATUS, gpsData ) == 1 )
  {
    // GPS fix
    switch( (int)gpsData[0] )
    {
      case 0x00:
        // no fix
        break;
      case 0x01:
        // dead reckoning only
        break;
      case 0x02:
        // 2D fix
        break;
      case 0x03:
        // 3D fix
        break;
      case 0x04:
        // GPS + dead reckoning combined
        break;
      case 0x05:
        // Time only fix
        break;
      default:
        // current state unknown
        break;
     }
  }else
  {
    ROS_INFO( "Status Message NOT Captured" );
  }
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

std::vector<float> NavioInterface::GetAHRS()
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
  /* -- this doesn't make sense -- research
  for( int i = 0; i < ARRAY_SIZE( _muxes ); ++i )
  {
    _adc.setMultiplexer( _muxes[i] );

    conversion = _adc.getMilliVolts();
    _results[i] = conversion;

    for( int j = 0; j < ARRAY_SIZE( _muxes ); ++j )
    {
      adcData.push_back( ( _results[j] / 1000 );
    }
  }
  */
  return adcData;
}
