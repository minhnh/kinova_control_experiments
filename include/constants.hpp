#ifndef _CONSTANTS_HPP_
#define _CONSTANTS_HPP_

#include <vector>
#include <string>

namespace constants {

namespace kinova {

    const std::vector<double> JOINT_ACCEL_LIMITS {5.19, 5.19, 5.19, 5.19, 9.99, 9.99, 9.99};
    const std::vector<double> JOINT_TORQUE {39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0};
    const std::vector<double> JOINT_STOP_TORQUE_LIMITS {39.0, 39.0, 39.0, 39.0, 13.0, 13.0, 13.0};
    const std::vector<double> JOINT_INERTIA {0.5580, 0.5580, 0.5580, 0.5580, 0.1389, 0.1389, 0.1389};

    const std::string URDF_PATH = "kinova_libs/extracted/urdf/kinova-gen3_urdf_V12.urdf";

    const std::string FRAME_JOINT_0 = "base_link";

    /**
    * With Bracelet_Link parameter, the last frame is at joint 7.
    * Mass and COM of the last (end-effector) link are included but not the real end-effector's frame.
    * Arm length: 1.12586m
    */
   const std::string FRAME_JOINT_6 = "Bracelet_Link";

   /**
    * With EndEffector_Link parameter, last frame is at the real end-effector's frame.
    * However, in the urdf model, joint between Bracelet_Link and EndEffector_Link is fixed (not counted in KDL).
    * Vereshchagin does not support un-equal number of joints and segments
    * Arm length: 1.1873m
    */
   const std::string FRAME_END_EFFECTOR = "EndEffector_Link";
};

};

#endif