#include <omegax_info/touch_pub.hpp>
#include <stdio.h>
#include <assert.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include "drdc.h"
#include "Robot.h"
#include "Transform.h"
#include "GL/glu.h"
typedef struct 
{
    HDboolean m_buttonState;       /* Has the device button has been pressed. */
    HDboolean m_buttonState2;
    hduVector3Dd m_devicePosition; /* Current device coordinates. */
    hduVector3Dd m_deviceOrientation;
    hduVector3Dd m_deviceOrientation2;
    HDErrorInfo m_error;
} DeviceData;

static DeviceData gServoDeviceData;

/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK updateDeviceCallback(void *pUserData)
{   
    int nButtons = 0;

    hdBeginFrame(hdGetCurrentDevice());

    /* Retrieve the current button(s). */
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    
    /* In order to get the specific button 1 state, we use a bitmask to
       test for the HD_DEVICE_BUTTON_1 bit. */
    gServoDeviceData.m_buttonState = 
        (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    
    gServoDeviceData.m_buttonState2 = 
        (nButtons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;
    /* Get the current location of the device (HD_GET_CURRENT_POSITION)
       We declare a vector of three doubles since hdGetDoublev returns 
       the information in a vector of size 3. */
    hdGetDoublev(HD_CURRENT_POSITION, gServoDeviceData.m_devicePosition);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gServoDeviceData.m_deviceOrientation);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, gServoDeviceData.m_deviceOrientation2);
    /* Also check the error state of HDAPI. */
    gServoDeviceData.m_error = hdGetError();

    /* Copy the position into our device_data tructure. */
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_CONTINUE;    
}


/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *pUserData)
{
    DeviceData *pDeviceData = (DeviceData *) pUserData;

    memcpy(pDeviceData, &gServoDeviceData, sizeof(DeviceData));

    return HD_CALLBACK_DONE;
}


namespace touch{

   publisher::publisher(const std::string& name) :
       Node( name ){
        touch_pub = create_publisher<omegax_info::msg::Touch>("Touch_msg", 200);
       
       }

  void publisher::publish( double x, double y, double z, double roll, double pitch, double yaw, bool button1, bool button2 ){
      omegax_info::msg::Touch msg;
      msg.twist.linear.x = x;
      msg.twist.linear.y = y;
      msg.twist.linear.z = z;
      msg.twist.angular.x = roll;
      msg.twist.angular.y = pitch;
      msg.twist.angular.z = yaw;
      
      msg.button1 = button1;
      msg.button2 = button2; 
      touch_pub->publish( msg );


}
}

int main( int argc, char** argv ){
      // Display information.
    std::cout << std::endl;
    std::cout << "Force Dimension - Virtual Teleoperation Example " << dhdGetSDKVersionStr() << ")" << std::endl;
    std::cout << "Copyright (C) 2001-2023 Force Dimension" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

    // Open haptic device.
    int deviceId = drdOpen();
    if (deviceId < 0)
    {
        std::cout << "error: failed to open haptic device (" << dhdErrorGetLastStr() << ")" << std::endl;
        return -1;
    }

    // Get haptic device serial number.
    uint16_t serialNumber = 0;
    if (dhdGetSerialNumber(&serialNumber, deviceId) < 0)
    {
        std::cout << "warning: failed to retrieve device serial number (" << dhdErrorGetLastStr() << ")" << std::endl;
    }

    // Initialize haptic device if required.
    std::cout << "initializing haptic device...\r";
    std::cout.flush();
    if (!drdIsInitialized(deviceId) && drdAutoInit(deviceId) < 0)
    {
        std::cout << "error: failed to initialize device (" << dhdErrorGetLastStr() << ")" << std::endl;
        return -1;
    }

    // Identify the device.
    std::cout << "connected to " << dhdGetSystemName(deviceId) << " S/N " << std::setw(5) << std::setfill('0') << serialNumber << std::endl;

    // Start robotic regulation if required.
    if (!drdIsRunning(deviceId) && drdStart(deviceId) < 0)
    {
        std::cout << "error: failed to start robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        return -1;
    }

    // Display instructions.
    std::cout << std::endl;
    std::cout << "Hold the user button on the haptic device to engage teleoperation, or" << std::endl;
    std::cout << "press ' ' in the UI window to engage/disengage teleoperation, or" << std::endl;
    std::cout << "press 'q' in the UI window to quit." << std::endl;
    std::cout << std::endl;

    // Align the master haptic device with the slave robot initial position.
    // In this example, the slave robot initial position is located at the center of the workspace.
    double initialPosition[DHD_MAX_DOF] = {};
    if (drdMoveTo(initialPosition, true, deviceId) < 0)
    {
        std::cout << "error: failed to align master with slave (" << dhdErrorGetLastStr() << ")" << std::endl;
        return -1;
    }
   if (drdStop(true, deviceId) < 0)
    {
        std::cout << "error: failed to stop robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
       // simulationRunning = false;
       
    }

    // Enable button emulation on devices featuring a gripper.
    if (dhdHasActiveGripper(deviceId) && dhdEmulateButton(DHD_ON, deviceId) < 0)
    {
        std::cout << "error: failed to enable button emulation (" << dhdErrorGetLastStr() << ")" << std::endl;
       // simulationRunning = false;
       
    }
  rclcpp::init( argc, argv );
  //touch::publisher publisher( "touch_info" );
  while(rclcpp::ok()){
  dhdGetPositionAndOrientationFrame(&masterX, &masterY, &masterZ, masterRotation, deviceId);    
  std::cout<<masterX<<" "<<masterY<<" "<<masterZ<<std::endl;
    
  
  }

 rclcpp::shutdown();
 
 return 0;
}
