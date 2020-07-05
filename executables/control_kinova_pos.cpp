/**
* @copyright 2020 Minh Nguyen
*/
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <chrono>
#include <libconfig.h++>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "util.h"
#include "kinova_util.h"
#include "abag.h"
#include "constants.hpp"

#define REPO_DIR "/home/minh/workspace/kinova_control_experiments/"
#define CTRL_APPROACH "position"

namespace k_api = Kinova::Api;
namespace sc = std::chrono;
namespace kc = kinova_ctrl;
namespace kc_const = kinova_ctrl::constants;
namespace kcc_conf = kinova_ctrl::constants::config;

void impedance_control_pos (
    k_api::Base::BaseClient* base,
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
    k_api::ActuatorConfig::ActuatorConfigClient* actuator_config,
    const KDL::Chain &pArmChain,
    const std::map<std::string, std::vector<double>> &pAbagConfigs,
    const std::vector<double> &pCartForceLimits
) {
    /* Initialize KDL data structures */
    int returnFlag = 0;
    const KDL::Vector GRAVITY(0.0, 0.0, -9.81289);
    const KDL::JntArray ZERO_ARRAY(ACTUATOR_COUNT);
    const KDL::Wrenches ZERO_WRENCHES(pArmChain.getNrOfSegments(), KDL::Wrench::Zero());
    KDL::JntArray jntCmdTorques(ACTUATOR_COUNT), jntPositions(ACTUATOR_COUNT), jntVelocities(ACTUATOR_COUNT),
                  jntImpedanceTorques(ACTUATOR_COUNT);

    KDL::Jacobian jacobianEndEff(pArmChain.getNrOfJoints());
    KDL::Frame endEffPose;
    Eigen::Matrix<double, 6, 1> endEffForce;
    endEffForce.setZero();

    /* KDL solvers */
    KDL::ChainJntToJacSolver jacobSolver(pArmChain);
    KDL::ChainFkSolverPos_recursive fkSolverPos(pArmChain);
    auto idSolver = std::make_shared<KDL::ChainIdSolver_RNE>(pArmChain, GRAVITY);

    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch(...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        throw;
    }

    /* Kinova API structures */
    k_api::BaseCyclic::Feedback baseFb;
    k_api::BaseCyclic::Command  baseCmd;
    auto servoingMode = k_api::Base::ServoingModeInformation();

    std::cout << "Initializing the arm for '" << CTRL_APPROACH << "' control" << std::endl;

    // Set the base in low-level servoing mode
    try {
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        baseFb = base_cyclic->RefreshFeedback();
    } catch (...) {
        std::cerr << "error setting low level control mode" << std::endl;
        throw;
    }

    // Initialize each actuator to their current position
    // Save the current actuator position, to avoid a following error
    for (int i = 0; i < ACTUATOR_COUNT; i++)
        baseCmd.add_actuators()->set_position(baseFb.actuators(i).position());

    // Send a first frame
    baseFb = base_cyclic->Refresh(baseCmd);

    // Set last actuator in torque mode now that the command is equal to measure
    auto ctrlModeInfo = k_api::ActuatorConfig::ControlModeInformation();
    ctrlModeInfo.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

    for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
        actuator_config->SetControlMode(ctrlModeInfo, actuator_id);

    // ABAG parameters
    KDL::Vector desiredPos, errorPos, commandPos;
    abagState_t abagStatePos[3];
    for (int i = 0; i < 3; i++) {
        initialize_abagState(&abagStatePos[i]);
    }

    // Real-time loop
    const int SECOND_IN_MICROSECONDS = 1000000;
    const int RATE_HZ = 600; // Hz
    const int TASK_TIME_LIMIT_SEC = 30;
    const sc::microseconds TASK_TIME_LIMIT_MICRO(TASK_TIME_LIMIT_SEC * SECOND_IN_MICROSECONDS);
    const sc::microseconds LOOP_DURATION(SECOND_IN_MICROSECONDS / RATE_HZ);

    int iterationCount = 0;
    int slowLoopCount = 0;
    sc::time_point<sc::steady_clock> controlStartTime = sc::steady_clock::now();;
    sc::time_point<sc::steady_clock> loopStartTime = sc::steady_clock::now();;
    sc::duration<int64_t, nano> totalElapsedTime = loopStartTime - controlStartTime;

    // open log file streams
    std::ofstream desiredPositionLog;
    desiredPositionLog.open("desired_values.csv", std::ofstream::out | std::ofstream::trunc);
    desiredPositionLog << "position X, position Y, position Z, velocity X, velocity Y, velocity Z" << std::endl;
    std::ofstream logPosX, logPosY, logPosZ;
    logPosX.open("ctrl_data_pos_x.csv", std::ofstream::out | std::ofstream::trunc);
    logPosY.open("ctrl_data_pos_y.csv", std::ofstream::out | std::ofstream::trunc);
    logPosZ.open("ctrl_data_pos_z.csv", std::ofstream::out | std::ofstream::trunc);
    logPosX << "time (ns), error, signed, bias, gain, e_bar, command x, meas pos x" << std::endl;
    logPosY << "time (ns), error, signed, bias, gain, e_bar, command y, meas pos y" << std::endl;
    logPosZ << "time (ns), error, signed, bias, gain, e_bar, command z, meas pos z" << std::endl;
    while (totalElapsedTime < TASK_TIME_LIMIT_MICRO)
    {
        iterationCount++;
        loopStartTime = sc::steady_clock::now();
        totalElapsedTime = loopStartTime - controlStartTime;

        try
        {
            baseFb = base_cyclic->RefreshFeedback();
        }
        catch(...)
        {
            if (desiredPositionLog.is_open()) desiredPositionLog.close();
            if (logPosX.is_open()) logPosX.close();
            if (logPosY.is_open()) logPosY.close();
            if (logPosZ.is_open()) logPosZ.close();
            std::cerr << "error reading sensors" << std::endl;
            throw;
        }

        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            jntPositions(i) = DEG_TO_RAD(baseFb.actuators(i).position()); // deg
            jntVelocities(i) = DEG_TO_RAD(baseFb.actuators(i).velocity()); // deg
        }

        // Kinova API provides only positive angle values
        // This operation is required to align the logic with our safety monitor
        // We need to convert some angles to negative values
        if (jntPositions(1) > DEG_TO_RAD(180.0)) jntPositions(1) = jntPositions(1) - DEG_TO_RAD(360.0);
        if (jntPositions(3) > DEG_TO_RAD(180.0)) jntPositions(3) = jntPositions(3) - DEG_TO_RAD(360.0);
        if (jntPositions(5) > DEG_TO_RAD(180.0)) jntPositions(5) = jntPositions(5) - DEG_TO_RAD(360.0);

        // gravity compensation (zero velocity, acceleration & external wrench)
        returnFlag = idSolver->CartToJnt(jntPositions, ZERO_ARRAY, ZERO_ARRAY, ZERO_WRENCHES, jntCmdTorques);
        if (returnFlag != 0) break;

        // solve for end effector Jacobian and pose with respect to base frame
        returnFlag = jacobSolver.JntToJac(jntPositions, jacobianEndEff);
        if (returnFlag != 0) break;
        returnFlag = fkSolverPos.JntToCart(jntPositions, endEffPose);
        if (returnFlag != 0) break;

        if (iterationCount == 1)
        {
            desiredPos[0] = endEffPose.p(0) + 0.03;
            desiredPos[1] = endEffPose.p(1) + 0.03;
            desiredPos[2] = endEffPose.p(2) + 0.03;
            // write desired position and zero velocities
            desiredPositionLog << desiredPos[0] << "," << desiredPos[1] << ","
                               << desiredPos[2] << "0.0, 0.0, 0.0" << std::endl;
            if (desiredPositionLog.is_open()) desiredPositionLog.close();
        }

        errorPos = desiredPos - endEffPose.p;

        for (int i = 0; i < 3; i++) {
            abag_sched(&abagStatePos[i], &errorPos[i], &commandPos[i], &(pAbagConfigs.at(kcc_conf::ALPHA)[i]),
                &(pAbagConfigs.at(kcc_conf::BIAS_THRES)[i]), &(pAbagConfigs.at(kcc_conf::BIAS_STEP)[i]),
                &(pAbagConfigs.at(kcc_conf::GAIN_THRES)[i]), &(pAbagConfigs.at(kcc_conf::GAIN_STEP)[i]));
        }

        endEffForce(0) = commandPos[0] * pCartForceLimits[0];
        endEffForce(1) = commandPos[1] * pCartForceLimits[1];
        endEffForce(2) = commandPos[2] * pCartForceLimits[2];

        jntImpedanceTorques.data = jacobianEndEff.data.transpose() * endEffForce;

        // std::cout << errorX << ", " << errorY << ", " << errorZ << std::endl;
        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            jntCmdTorques(i) = jntCmdTorques(i) + jntImpedanceTorques(i);
            if      (jntCmdTorques(i) >=  kc_const::kinova::JOINT_TORQUE_LIMITS[i])
                     jntCmdTorques(i) =   kc_const::kinova::JOINT_TORQUE_LIMITS[i] - 0.001;
            else if (jntCmdTorques(i) <= -kc_const::kinova::JOINT_TORQUE_LIMITS[i])
                     jntCmdTorques(i) =  -kc_const::kinova::JOINT_TORQUE_LIMITS[i] + 0.001;
            baseCmd.mutable_actuators(i)->set_position(baseFb.actuators(i).position());
            baseCmd.mutable_actuators(i)->set_torque_joint(jntCmdTorques(i));
        }

        // std::cout << jntCmdTorques.data.transpose() << std::endl;
        // write to log files
        kc::writeDataRow(logPosX, abagStatePos[0], totalElapsedTime.count(), errorPos[0], commandPos[0], endEffPose.p(0));
        kc::writeDataRow(logPosY, abagStatePos[1], totalElapsedTime.count(), errorPos[1], commandPos[1], endEffPose.p(1));
        kc::writeDataRow(logPosZ, abagStatePos[2], totalElapsedTime.count(), errorPos[2], commandPos[2], endEffPose.p(2));

        // Incrementing identifier ensures actuators can reject out of time frames
        baseCmd.set_frame_id(baseCmd.frame_id() + 1);
        if (baseCmd.frame_id() > 65535) baseCmd.set_frame_id(0);

        for (int idx = 0; idx < ACTUATOR_COUNT; idx++)
            baseCmd.mutable_actuators(idx)->set_command_id(baseCmd.frame_id());

        try
        {
            baseFb = base_cyclic->Refresh(baseCmd, 0);
        }
        catch(...)
        {
            if (desiredPositionLog.is_open()) desiredPositionLog.close();
            if (logPosX.is_open()) logPosX.close();
            if (logPosY.is_open()) logPosY.close();
            if (logPosZ.is_open()) logPosZ.close();
            std::cerr << "error sending command" << std::endl;
            throw;
        }

        // Enforce the constant loop time and count how many times the loop was late
        if (kc::waitMicroSeconds(loopStartTime, LOOP_DURATION) != 0) slowLoopCount++;
    }

    // Set first actuator back in position
    kc::stopRobot(actuator_config);

    std::cout << "Torque control example completed" << std::endl;
    std::cout << "Number of loops which took longer than specified duration: " << slowLoopCount << std::endl;

    // Set the servoing mode back to Single Level
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    // close log files
    if (logPosX.is_open()) logPosX.close();
    if (logPosY.is_open()) logPosY.close();
    if (logPosZ.is_open()) logPosZ.close();

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

int main(int argc, char **argv)
{
    // Load configurations
    libconfig::Config cfg;    // this needs to exist to query the root setting, otherwise will cause segfault
    std::map<std::string, std::vector<double>> abagConfigs;
    std::vector<double> cartForceLimits;
    std::string hostName, username, password;
    unsigned int port, portRealTime;
    try
    {
        libconfig::Setting& root = kc::loadConfigFile(cfg, REPO_DIR"config/kinova.cfg");

        kc::loadAbagConfig(root, CTRL_APPROACH, abagConfigs);

        kc::loadKinovaConfig(root, cartForceLimits, hostName, username, password, port, portRealTime);

        kc::printConfigurations(CTRL_APPROACH, hostName, username, password, port, portRealTime,
                                cartForceLimits, abagConfigs);
    }
    catch (const std::exception &ex)
    {
        std::cerr << "failed to load configuration file: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Load URDF file
    const std::string UrdfPath = REPO_DIR + kc_const::kinova::URDF_PATH;
    std::cout << "loading URDF file: " << UrdfPath << std::endl;
    KDL::Tree kinovaTree; KDL::Chain kinovaChain;
    kc::loadUrdfModel(UrdfPath, kinovaTree, kinovaChain);

    try
    {
        kc::KinovaBaseConnection connection(hostName, port, portRealTime, username, password);

        // Example core
        kc::move_to_home_position(connection.mBaseClient.get());
        try
        {
            impedance_control_pos(
                connection.mBaseClient.get(), connection.mBaseCyclicClient.get(), connection.mActuatorConfigClient.get(),
                kinovaChain, abagConfigs, cartForceLimits);
        }
        catch (k_api::KDetailedException& ex)
        {
            kc::stopRobot(connection.mActuatorConfigClient.get());
            kc::printKinovaException(ex);
            return EXIT_FAILURE;
        }
        catch (std::exception& ex)
        {
            kc::stopRobot(connection.mActuatorConfigClient.get());
            std::cout << "Unhandled Error: " << ex.what() << std::endl;
            return EXIT_FAILURE;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << "failed to establish connection: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
