#include <cisstParameterTypes/prmInputData.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <iostream>
#include <string>
#include <sstream>
#include "math.h"
#include <assert.h>
#include "mtsTeleoperation.h"
#include "drdc.h"
#include "Robot.h"
#include "Transform.h"


CMN_IMPLEMENT_SERVICES(mtsTeleoperation);

mtsTeleoperation::mtsTeleoperation(const std::string & taskName, double period) :
    mtsTaskPeriodic(taskName, period, false, 1000)
    {
          mtsInterfaceRequired * requiresControlGalil = AddInterfaceRequired("requiresControlGalil");
    if (requiresControlGalil) {
        // Standard CRTK interfaces
        requiresControlGalil->AddFunction("DisableMotorPower", crtk_disable);
     
    }
    }
mtsTeleoperation::~mtsTeleoperation(void)
{
    Cleanup();
}
void mtsTeleoperation::Startup(void){
    Timer.Start();
    deviceId = drdOpen();
    if (deviceId < 0)
    {
        std::cout << "error: failed to open haptic device (" << dhdErrorGetLastStr() << ")" << std::endl;
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
    }

    // Identify the device.
    std::cout << "connected to " << dhdGetSystemName(deviceId) << " S/N " << std::setw(5) << std::setfill('0') << serialNumber << std::endl;

    // Start robotic regulation if required.
    if (!drdIsRunning(deviceId) && drdStart(deviceId) < 0)
    {
        std::cout << "error: failed to start robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
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
}

void mtsTeleoperation::Run(void){
    ProcessQueuedEvents();
    ProcessQueuedCommands();
    dhdGetPositionAndOrientationFrame(&masterX, &masterY, &masterZ, masterRotation, deviceId);    
  std::cout<<masterX<<" "<<masterY<<" "<<masterZ<<std::endl;
}


void mtsTeleoperation::Cleanup() {crtk_disable();}
void mtsTeleoperation::GetAbsoluteTime(mtsDouble & sec) const {
    sec = mtsTaskManager::GetInstance()->GetTimeServer().GetAbsoluteTimeInSeconds();
}
