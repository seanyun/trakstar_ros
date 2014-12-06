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

#ifndef _Trakstar_PointATC3DG_
#define _Trakstar_PointATC3DG_

// Device Vendor ID (idVendor)
#ifndef BIRD_VENDOR
#define BIRD_VENDOR     1204
#endif

// Device Product ID (idProduct)
#ifndef BIRD_PRODUCT
//#define BIRD_PRODUCT	4099 // medSAFE
#define BIRD_PRODUCT    4101 // trakSTAR
#endif

// Device Endpoint-Out
#ifndef BIRD_EP_OUT
#define BIRD_EP_OUT     0x02
#endif

// Device Endpoint-In
#ifndef BIRD_EP_IN
#define BIRD_EP_IN      0x86
#endif

#define HEMISPHERE_FRONT 0x00
#define HEMISPHERE_REAR  0x01

// forward declarations
struct usb_device;
struct usb_dev_handle;

namespace trakstar {

class PointATC3DG {
public:
    PointATC3DG();
    ~PointATC3DG();

    // evaluate whether initialized properly
    bool operator!() const;
    bool ok() const;

    int setSuddenOutputChangeLock( int iSensorId );

    int setSensorRotMat( int iSensorId );
    int setSensorQuaternion( int iSensorId );
    int setSensorTopHemisphere( int iSensorId );
    int setSensorHemisphere( int iSensorId, char cSphereId );
    int setMeasurementRate( double dRate );
    int setMaximumRange(bool if_72inch);
    int getNumberOfSensors( void );
    int getCoordinatesAngles( int iSensorId,
        double& dX, double& dY, double& dZ,
        double& dAzimuth, double& dElevation, double& dRoll );
    int getCoordinatesMatrix( int iSensorId,
        double& dX, double& dY, double& dZ,
        double* pMat );
    int getCoordinatesQuaternion( int iSensorId,
        double& dX, double& dY, double& dZ,
        double* quat );
    bool transmitterAttached();
    bool sensorAttached(const int& iSensorId);

protected:
    struct usb_device* find_device( int iVendorId, int iProductId );

    int check_bird_errors( void );
    void error( int val, const char* msg, ... );

    void reset();

protected:
    struct usb_device *dev;
    struct usb_dev_handle *handle;

    char dataout[16];
    char datain[32];

    bool isOk;
    double posk;
};

}

#endif // _PointATC3DG_

