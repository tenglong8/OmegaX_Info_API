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
#include "Eigen/Eigen"


CMN_IMPLEMENT_SERVICES(mtsTeleoperation);

mtsTeleoperation::mtsTeleoperation(const std::string & taskName, double period) :
    mtsTaskPeriodic(taskName, period, false, 1000)
    {
   // StateTable.AddData(force, "omega_force");
   // StateTable.AddData(torque, "omega_torque");
    //StateTable.AddData(masterX, "omega_X");
    //StateTable.AddData(masterY, "omega_Y");
    //StateTable.AddData(masterZ, "omega_Z");
    //mtsInterfaceProvided *omegaAPI = AddInterfaceProvided("omegaAPI");
    //if (omegaAPI){
    //    omegaAPI->AddCommandReadState(StateTable, force, "omega_force");
    //    omegaAPI->AddCommandReadState(StateTable, torque, "omega_torque");
    //    omegaAPI->AddCommandReadState(StateTable, masterX, "omega_X");
    //    omegaAPI->AddCommandReadState(StateTable, masterY, "omega_Y");
    //    omegaAPI->AddCommandReadState(StateTable, masterZ, "omega_Z");
   // }
    
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
    buttonEngaged = (dhdGetButton(0) != DHD_OFF);
    std::cout << "//////////////////////////////////////////////////////////////////"<< std::endl;
    dhdGetPositionAndOrientationFrame(&masterX, &masterY, &masterZ, masterRotation, deviceId);    
    std::cout<<masterX<<" "<<masterY<<" "<<masterZ<<" "<<buttonEngaged<<std::endl;
    dhdGetLinearVelocity(&masterLinearVelocityX, &masterLinearVelocityY, &masterLinearVelocityZ, deviceId);
    std::cout<<masterLinearVelocityX<<" "<<masterLinearVelocityY<<" "<<masterLinearVelocityZ<<" "<<std::endl;
    dhdGetAngularVelocityRad(&masterAngularVelocityX, &masterAngularVelocityY, &masterAngularVelocityZ, deviceId);
    std::cout<<masterAngularVelocityX<<" "<<masterAngularVelocityY<<" "<<masterAngularVelocityZ<<" "<<std::endl;
    force<<buttonEngaged*3, buttonEngaged*0, buttonEngaged*0;
    torque<<buttonEngaged*10, buttonEngaged*10, buttonEngaged*10;
    dhdSetForceAndTorqueAndGripperForce(force(0), force(1), force(2), torque(0), torque(1), torque(2), 0.0, deviceId);
    std::cout<<force(0)<<" "<<force(1)<<" "<<force(2)<<" "<<std::endl;
    std::cout << "//////////////////////////////////////////////////////////////////"<< std::endl;
    
}


void mtsTeleoperation::Cleanup() {crtk_disable();}
void mtsTeleoperation::GetAbsoluteTime(mtsDouble & sec) const {
    sec = mtsTaskManager::GetInstance()->GetTimeServer().GetAbsoluteTimeInSeconds();
}
