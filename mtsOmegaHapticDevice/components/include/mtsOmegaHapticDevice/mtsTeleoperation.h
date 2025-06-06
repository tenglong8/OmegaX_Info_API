#include <cisstVector/vctTypes.h>
#include <cisstOSAbstraction.h>
#include <cisstMultiTask.h>
#include <cisstParameterTypes/prmActuatorState.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmRobotState.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianSet.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include "Eigen/Eigen"
//#include <mtsOmegaHapticDeviceExport.h>

class mtsTeleoperation : public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE);

public:
    
    mtsTeleoperation(const std::string & taskName, double period);
    ~mtsTeleoperation();
    
    void Startup();

    void Run();

    void Cleanup();

    void GetAbsoluteTime(mtsDouble & sec) const;
    
    
protected:
    osaStopwatch     Timer;
    mtsDouble        timestamp;
    mtsFunctionVoid  crtk_disable;
    int deviceId;
    Eigen::Vector3d force;
    Eigen::Vector3d torque;
    double masterX, masterY, masterZ;
    double masterLinearVelocityX, masterLinearVelocityY, masterLinearVelocityZ;
    double masterAngularVelocityX, masterAngularVelocityY, masterAngularVelocityZ;
    double masterRotation[3][3] = {};
    bool buttonEngaged;
    struct{
   double masterX;
   double masterY;
   double masterZ;
    } pos;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleoperation);
