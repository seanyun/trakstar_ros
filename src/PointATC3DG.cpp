/*********************************************************************
*
* This is free and unencumbered software released into the public domain.
* 
* Anyone is free to copy, modify, publish, use, compile, sell, or
* distribute this software, either in source code form or as a compiled
* binary, for any purpose, commercial or non-commercial, and by any
* means.
* 
* In jurisdictions that recognize copyright laws, the author or authors
* of this software dedicate any and all copyright interest in the
* software to the public domain. We make this dedication for the benefit
* of the public at large and to the detriment of our heirs and
* successors. We intend this dedication to be an overt act of
* relinquishment in perpetuity of all present and future rights to this
* software under copyright law.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
* 
* For more information, please refer to <http://unlicense.org/>
* 
**********************************************************************/

/** 
 * Edited by: Sean Seungkook Yun <seungkook.yun@sri.com> 
*/

#include "trakstar/PointATC3DG.hpp"
#include <ros/ros.h>
#include "ros/console.h"
#include "ros/assert.h"

#include <cstdio>
#include <cstdarg>
#include <algorithm>

#include <usb.h>


#define DELAY                       500

// Commands
#define POINT                       0x42
#define RUN                         0x46
#define SLEEP                       0x47
#define EXAMINE_VALUE               0x4F
#define CHANGE_VALUE                0x50
#define POS_ANG                     0x59
#define POS_MAT                     0x5A
#define RESET                       0x62
#define METAL                       0x73
#define POS_QUAT      		    0x5D
#define HEMISPHERE                  0x4C // 2 bytes

// Examine options
#define BIRD_STATUS                 0x00 // 2 bytes
#define BIRD_POSITION_SCALING       0x03 // 2 bytes
#define MEASUREMENT_RATE            0x07 // 2 bytes
#define BIRD_ERROR_CODE             0x0A // 1 byte
#define SYSTEM_MODEL_IDENT          0x0F // 10 bytes
#define BIRD_SERIAL_NUMBER          0x19 // 2 bytes
#define SENSOR_SERIAL_NUMBER        0x1A // 2 bytes
#define TRANSMITTER_SERIAL_NUMBER   0x1B // 2 bytes
#define SUDDEN_OUTPUT_CHANGE_LOCK   0x0E // 1 byte
#define FBB_AUTO_CONFIGURATION      0x32


	
// Conversions
#define WTF         	(double) (1.0 / 32768.0)    		// word to float
#define INCH_TO_METER 	(double) (25.4/1000.0)	
#define ANGK        	(double) (180.0 * WTF)  		// word to angle
#define POSK36      	(double) (36.0 * WTF * INCH_TO_METER)   // word to position
#define POSK72      	(double) (72.0 * WTF * INCH_TO_METER)   // word to position

// helpful macros
int ret;

#define WRITE( data, bytes )                                                \
    ret = usb_bulk_write( handle, BIRD_EP_OUT, data, bytes, DELAY );

#define READ( data, bytes )                                                 \
    do {                                                                    \
        ret = usb_bulk_read( handle, BIRD_EP_IN, data, bytes, DELAY );      \
    } while( ret == 0 );                                                    \


namespace trakstar {

PointATC3DG::PointATC3DG() :
    isOk( true )
{    
    usb_init();

    dev = find_device( BIRD_VENDOR, BIRD_PRODUCT );
    if( !dev ) {
        error( 1, "finding device on USB bus. Turn it on?" );
        return;
    }

    handle = usb_open( dev );
    if( !handle ) {
        error( 2, "claiming USB device -> %s", usb_strerror() );
        return;
    }

    ret = usb_set_configuration( handle, 1 );
    if( ret < 0 ) {
        error( ret, "setting configuration on USB device."
                    " Check device permissions? -> %s", usb_strerror() );
        return;
    }
    ret = usb_claim_interface( handle, 0 );
    if( ret < 0 ) {
        error( ret, "claiming USB interface on device -> %s", usb_strerror() );
        return;
    }
    ret = usb_set_altinterface( handle, 0 );
    if( ret < 0 ) {
        error( ret, "setting alternate interface -> %s", usb_strerror() );
        return;
    }
    ret = usb_clear_halt( handle, BIRD_EP_IN );
    if( ret < 0 ) {
        error( ret, "clearing halt on EP_IN -> %s", usb_strerror() );
        return;
    }
    ret = usb_bulk_read( handle, BIRD_EP_IN, datain, 32, DELAY );

    //reset();
    //usleep(5000000);
    //check_bird_errors();

    dataout[0] = CHANGE_VALUE;
    dataout[1] = FBB_AUTO_CONFIGURATION;
    dataout[2] = 0x01;
    WRITE( dataout, 3 );
    if( ret < 0 ) {
        error( ret, "sending FBB_AUTO_CONFIGURATION -> %s", usb_strerror() );
        return;
    }
    usleep( 600000 ); // delay 600 ms after auto-configuration

    dataout[0] = EXAMINE_VALUE;
    dataout[1] = BIRD_POSITION_SCALING;
    WRITE( dataout, 2 );
    READ( datain, 2 );
    if( ret < 0 ) {
        error( ret, "querying scaling factor -> %s", usb_strerror() );
        return;
    }
    posk = POSK36;
    if( datain[0] == 1 )
        posk = POSK72;
    ROS_INFO("position scale: %f \n", posk);

    check_bird_errors();
}

PointATC3DG::~PointATC3DG()
{
    if( dev && handle ) {
        dataout[0] = SLEEP;
        WRITE( dataout, 1 );
        usb_close( handle );
    }
}

void PointATC3DG::reset() 
{
	dataout[0] = RESET;
	WRITE( dataout, 1 );   
}

bool PointATC3DG::operator!() const
{
    return !isOk;
}

bool PointATC3DG::ok() const
{
    return isOk;
}

int PointATC3DG::setSuddenOutputChangeLock( int iSensorId )
{
    dataout[0] = 0xf1 + iSensorId;
    dataout[1] = CHANGE_VALUE;
    dataout[2] = SUDDEN_OUTPUT_CHANGE_LOCK;
    dataout[3] = 0x01;
    WRITE( dataout, 4 );

    return check_bird_errors();
}

int PointATC3DG::setSensorRotMat( int iSensorId )
{
    dataout[0] = 0xf1 + iSensorId;
    dataout[1] = POS_MAT;
    WRITE( dataout, 2 );

    return check_bird_errors();
}

int PointATC3DG::setSensorQuaternion( int iSensorId )
{
    dataout[0] = 0xf1 + iSensorId;
    dataout[1] = POS_QUAT;
    WRITE( dataout, 2 );

    return check_bird_errors();    
}

int PointATC3DG::setSensorTopHemisphere( int iSensorId )
{
    return check_bird_errors();
}

int PointATC3DG::setSensorHemisphere( int iSensorId, char cSphereId )
{
  dataout[0] = 0xf1 + iSensorId;
  dataout[1] = HEMISPHERE;
  dataout[2] = 0x00;
  dataout[3] = cSphereId;
  WRITE(dataout, 4);

  return check_bird_errors();
}

int PointATC3DG::setMeasurementRate( double dRate )
{
    short sRate = (short) (dRate * 256);
    dataout[0] = CHANGE_VALUE;
    dataout[1] = MEASUREMENT_RATE;
    dataout[2] = (char) (sRate & 0xff);
    dataout[3] = (char) (sRate >> 8);
    WRITE( dataout, 4 );

    return check_bird_errors();
}

int PointATC3DG::getNumberOfSensors( void )

{
    int nSensors = 0;
    for( char i = 0x00 ; i < 0x04 ; ++i ) {
        dataout[0] = 0xf1 + i;
        dataout[1] = EXAMINE_VALUE;
        dataout[2] = SENSOR_SERIAL_NUMBER;
        WRITE( dataout, 3 );
        READ( datain, 2 );
        if( ret < 0 ) {
            error( ret, "getting serial number for sensor %d -> %s",
                        i, usb_strerror() );
        }
        else if( datain[0] || datain[1] ) ++nSensors;
    }
    return nSensors;
}

int PointATC3DG::getCoordinatesAngles( int iSensorId,
                     double& dX, double& dY, double& dZ,
                     double& dAzimuth, double& dElevation, double& dRoll )
{
    short nX, nY, nZ, nAzimuth, nElevation, nRoll;

    dataout[0] = 0xf1 + iSensorId;
    dataout[1] = POINT;
    WRITE( dataout, 2 );

    READ( datain, 12 );

    if( ret == 12 ) {
        nX = ( (datain[1] << 7) | (datain[0] & 0x7f) ) << 2;
        nY = ( (datain[3] << 7) | (datain[2] & 0x7f) ) << 2;
        nZ = ( (datain[5] << 7) | (datain[4] & 0x7f) ) << 2;
    
        nAzimuth = ( (datain[7] << 7) | (datain[6] & 0x7f) ) << 2;
        nElevation = ( (datain[9] << 7) | (datain[8] & 0x7f) ) << 2;
        nRoll = ( (datain[11] << 7) | (datain[10] & 0x7f) ) << 2;
    
        dX = nX * posk;
        dY = nY * posk;
        dZ = nZ * posk;
    
        dAzimuth = nAzimuth * ANGK;
        dElevation = nElevation * ANGK;
        dRoll = nRoll * ANGK;
    
        return check_bird_errors();
    }
    error( ret, "reading point data -> %s", usb_strerror() );
    return ret;
}

int PointATC3DG::getCoordinatesQuaternion( int iSensorId,
        double& dX, double& dY, double& dZ,
        double* quat )
{
    short q[4];
    short nX, nY, nZ;

    dataout[0] = 0xf1 + iSensorId;
    dataout[1] = POINT;
    WRITE( dataout, 2 );

    READ( datain, 14 );

    if( ret == 14 ) {
        nX = ( (datain[1] << 7) | (datain[0] & 0x7f) ) << 2;
        nY = ( (datain[3] << 7) | (datain[2] & 0x7f) ) << 2;
        nZ = ( (datain[5] << 7) | (datain[4] & 0x7f) ) << 2;
    
        short *dataptr = q;
        for( int i = 7 ; i < 14 ; i += 2, ++dataptr ) {
            *dataptr = ( (datain[i] << 7) | (datain[i-1] & 0x7f) ) << 2;
        }
    
        dX = nX * posk;
        dY = nY * posk;
        dZ = nZ * posk;
    
        for( int i = 0 ; i < 4 ; ++i )
            quat[i] = q[i] * WTF;
    
        return check_bird_errors();
    }
    error( ret, "reading point data -> %s", usb_strerror() );
    return ret;
}

int PointATC3DG::getCoordinatesMatrix( int iSensorId,
                     double& dX, double& dY, double& dZ,
                     double* pMat )
{
    short sMat[9];
    short nX, nY, nZ;

    dataout[0] = 0xf1 + iSensorId;
    dataout[1] = POINT;
    WRITE( dataout, 2 );

    READ( datain, 24 );

    if( ret == 24 ) {
        nX = ( (datain[1] << 7) | (datain[0] & 0x7f) ) << 2;
        nY = ( (datain[3] << 7) | (datain[2] & 0x7f) ) << 2;
        nZ = ( (datain[5] << 7) | (datain[4] & 0x7f) ) << 2;
    
        short *dataptr = sMat;
        for( int i = 7 ; i < 24 ; i += 2, ++dataptr ) {
            *dataptr = ( (datain[i] << 7) | (datain[i-1] & 0x7f) ) << 2;
        }
    
        dX = nX * posk;
        dY = nY * posk;
        dZ = nZ * posk;
    
        for( int i = 0 ; i < 9 ; ++i )
            pMat[i] = sMat[i] * WTF;
    
        return check_bird_errors();
    }
    error( ret, "reading point data -> %s", usb_strerror() );
    return ret;
}

bool PointATC3DG::transmitterAttached()
{
    dataout[0] = EXAMINE_VALUE;
    dataout[1] = TRANSMITTER_SERIAL_NUMBER;
    WRITE( dataout, 2 );
    READ( datain, 2 );
    if( ret < 0 ) {
        error( ret, "getting serial number for transmitter -> %s", usb_strerror() );
    }
    else if( datain[0] || datain[1] )   return true;
    return false;
}

bool PointATC3DG::sensorAttached(const int& iSensorId)
{
    dataout[0] = 0xf1 + iSensorId;
    dataout[1] = EXAMINE_VALUE;
    dataout[2] = SENSOR_SERIAL_NUMBER;
    WRITE( dataout, 3 );
    READ( datain, 2 );
    if( ret < 0 ) {
        error( ret, "getting serial number for sensor %d -> %s",
                    iSensorId, usb_strerror() );
    }
    else if( datain[0] || datain[1] )   return true;
    return false;
}

struct usb_device* PointATC3DG::find_device( int iVendorId, int iProductId )
{
    struct usb_bus *bus;
    struct usb_device *dev;

    usb_find_busses();
    usb_find_devices();

    for( bus = usb_busses ; bus ; bus = bus->next ) {
        for( dev = bus->devices ; dev ; dev = dev->next ) {
            if( dev->descriptor.idVendor == iVendorId &&
                dev->descriptor.idProduct == iProductId ) {
                return dev;
            }
        }
    }
    return NULL;
}

int PointATC3DG::check_bird_errors( void )
{
    bool fatal = false;
    dataout[0] = EXAMINE_VALUE;
    dataout[1] = BIRD_ERROR_CODE;
    WRITE( dataout, 2 );
    READ( datain, 1 );

    if( datain[0] == 0 ) return 0;

    switch( datain[0] ) {
        case 1:     ROS_ERROR("FATAL(1): System Ram Failure" ); fatal = true; break;
        case 2:     ROS_ERROR("FATAL(2): Non-Volatile Storage Write Failure" ); fatal = true; break;
        case 3:     ROS_ERROR("WARNING(3): PCB Configuration Data Corrupt" ); break;
        case 4:     ROS_ERROR("WARNING(4): Bird Transmitter Calibration Data Corrupt or Not Connected" ); break;
        case 5:     ROS_ERROR("WARNING(5): Bird Sensor Calibration Data Corrupt or Not Connected" ); break;
        case 6:     ROS_ERROR("WARNING(6): Invalid RS232 Command" ); break;
        case 7:     ROS_ERROR("WARNING(7): Not an FBB Master" ); break;
        case 8:     ROS_ERROR("WARNING(8): No Birds Accessible in Device List" ); break;
        case 9:     ROS_ERROR("WARNING(9): Bird is Not Initialized" ); break;
        case 10:    ROS_ERROR("WARNING(10): FBB Serial Port Receive Error - Intra Bird Bus" ); break;
        case 11:    ROS_ERROR("WARNING(11): RS232 Serial Port Receive Error" ); break;
        case 12:    ROS_ERROR("WARNING(12): FBB Serial Port Receive Error" ); break;
        case 13:    ROS_ERROR("WARNING(13): No FBB Command Response" ); break;
        case 14:    ROS_ERROR("WARNING(14): Invalid FBB Host Command" ); break;
        case 15:    ROS_ERROR("FATAL(15): FBB Run Time Error" ); fatal = true; break;
        case 16:    ROS_ERROR("FATAL(16): Invalid CPU Speed" ); fatal = true; break;
        case 17:    ROS_ERROR("WARNING(17): No FBB Data" ); break;
        case 18:    ROS_ERROR("WARNING(18): Illegal Baud Rate" ); break;
        case 19:    ROS_ERROR("WARNING(19): Slave Acknowledge Error" ); break;
        case 20: case 21: case 22: case 23:
        case 24: case 25: case 26: case 27:
            	    ROS_ERROR("FATAL(%d): Intel 80186 CPU Errors", datain[0] ); fatal = true; break;
        case 28:    ROS_ERROR("WARNING(28): CRT Synchronization" ); break;
        case 29:    ROS_ERROR("WARNING(29): Transmitter Not Accessible" ); break;
        case 30:    ROS_ERROR("WARNING(30): Extended Range Transmitter Not Attached" ); break;
        case 32:    ROS_ERROR("WARNING(32): Sensor Saturated" ); break;
        case 33:    ROS_ERROR("WARNING(33): Slave Configuration" ); break;
        case 34:    ROS_ERROR("WARNING(34): Watch Dog Timer" ); break;
        case 35:    ROS_ERROR("WARNING(35): Over Temperature" ); break;
        default:    ROS_ERROR("WARNING(%d): Unknown Error Code", datain[0] );
    }

    if( fatal ) {
        isOk = false;
        exit( datain[0] );
    }

    return datain[0];
}

void PointATC3DG::error( int val, const char* msg, ... )
{
    //va_list ap;
    //va_start( ap, msg );
    ROS_ERROR("error(%d): %s \n", val, msg );
    //ROS_ERRO( stderr, msg, ap );
    //ROS_ERROR("\n" );
    //va_end( ap );
    isOk = false;
}


int PointATC3DG::setMaximumRange(bool if_72inch)
{
  char range= if_72inch ? 1 : 0;
  posk = if_72inch ? POSK72 : POSK36;

  dataout[0] = CHANGE_VALUE;
  dataout[1] = BIRD_POSITION_SCALING;
  dataout[2] = range;
  WRITE( dataout, 3 );

  return check_bird_errors();
}

}
