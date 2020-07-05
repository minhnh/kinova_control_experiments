/**
* @copyright 2020 Minh Nguyen
*/
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <libconfig.h++>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

#include "util.h"
#include "kinova_util.h"
#include "abag.h"
#include "constants.hpp"

#define REPO_DIR "/home/minh/workspace/kinova_control_experiments/"
#define CTRL_APPROACH "position_velocity"

namespace k_api = Kinova::Api;
namespace sc = std::chrono;
namespace kc = kinova_ctrl;
namespace kc_const = kinova_ctrl::constants;
namespace kcc_conf = kinova_ctrl::constants::config;

void impedance_control_posvel (
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
                  jntTorques(ACTUATOR_COUNT), jntImpedanceTorques(ACTUATOR_COUNT);
    KDL::Frame endEffPose;
    KDL::FrameVel endEffTwist;
    KDL::JntArrayVel jntPositionVelocity(ACTUATOR_COUNT);
    KDL::Jacobian jacobianEndEff(pArmChain.getNrOfJoints());
    Eigen::Matrix<double, 6, 1> endEffForce;
    endEffForce.setZero();

    /* KDL solvers */
    KDL::ChainJntToJacSolver jacobSolver(pArmChain);
    KDL::ChainFkSolverPos_recursive fkSolverPos(pArmChain);
    KDL::ChainFkSolverVel_recursive fkSolverVel(pArmChain);
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
    auto servoingModeInfo = k_api::Base::ServoingModeInformation();

    std::cout << "Initializing the arm for '" << CTRL_APPROACH << "' control" << std::endl;

    // Set the base in low-level servoing mode
    try {
        servoingModeInfo.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingModeInfo);
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
    const double desiredTwist[3] = { 0.01, 0.01, 0.01 };
    double desiredPos[3] = { 0.0 };
    double errors[6] = { 0.0 };
    double commands[6] = { 0.0 };
    abagState_t abagStates[6];
    for (int i = 0; i < 6; i++) {
        initialize_abagState(&abagStates[i]);
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
    std::ofstream desiredValuesLog;
    desiredValuesLog.open("desired_values.csv", std::ofstream::out | std::ofstream::trunc);
    desiredValuesLog << "position X, position Y, position Z, velocity X, velocity Y, velocity Z" << std::endl;
    std::ofstream logPosX, logPosY, logPosZ, logVelX, logVelY, logVelZ;
    logPosX.open("ctrl_data_pos_x.csv", std::ofstream::out | std::ofstream::trunc);
    logPosY.open("ctrl_data_pos_y.csv", std::ofstream::out | std::ofstream::trunc);
    logPosZ.open("ctrl_data_pos_z.csv", std::ofstream::out | std::ofstream::trunc);
    logVelX.open("ctrl_data_vel_x.csv", std::ofstream::out | std::ofstream::trunc);
    logVelY.open("ctrl_data_vel_y.csv", std::ofstream::out | std::ofstream::trunc);
    logVelZ.open("ctrl_data_vel_z.csv", std::ofstream::out | std::ofstream::trunc);
    logPosX << "time (ns), error, signed, bias, gain, e_bar, command x, meas pos x" << std::endl;
    logPosY << "time (ns), error, signed, bias, gain, e_bar, command y, meas pos y" << std::endl;
    logPosZ << "time (ns), error, signed, bias, gain, e_bar, command z, meas pos z" << std::endl;
    logVelX << "time (ns), error, signed, bias, gain, e_bar, command x, meas vel x" << std::endl;
    logVelY << "time (ns), error, signed, bias, gain, e_bar, command y, meas vel y" << std::endl;
    logVelZ << "time (ns), error, signed, bias, gain, e_bar, command z, meas vel z" << std::endl;
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
            kc::stopRobot(actuator_config);
            if (desiredValuesLog.is_open()) desiredValuesLog.close();
            if (logPosX.is_open()) logPosX.close();
            if (logPosY.is_open()) logPosY.close();
            if (logPosZ.is_open()) logPosZ.close();
            if (logVelX.is_open()) logVelX.close();
            if (logVelY.is_open()) logVelY.close();
            if (logVelZ.is_open()) logVelZ.close();
            std::cerr << "error reading sensors" << std::endl;
            throw;
        }

        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            jntPositions(i) = DEG_TO_RAD(baseFb.actuators(i).position()); // deg
            jntVelocities(i) = DEG_TO_RAD(baseFb.actuators(i).velocity()); // deg
            jntTorques(i)   = baseFb.actuators(i).torque(); // nm
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
        jntPositionVelocity.q = jntPositions;
        jntPositionVelocity.qdot = jntVelocities;
        returnFlag = fkSolverVel.JntToCart(jntPositionVelocity, endEffTwist);
        if (returnFlag != 0) break;

        if (iterationCount == 1)
        {
            desiredPos[0] = endEffPose.p(0) + 0.10;
            desiredPos[1] = endEffPose.p(1) + 0.10;
            desiredPos[2] = endEffPose.p(2) + 0.10;
            desiredValuesLog << desiredPos[0] << "," << desiredPos[1] << "," << desiredPos[2] << ","
                             << desiredTwist[0] << "," << desiredTwist[1] << "," << desiredTwist[2] << std::endl;
            if (desiredValuesLog.is_open()) desiredValuesLog.close();
        }

        errors[0] = desiredPos[0] - endEffPose.p(0);
        errors[1] = desiredPos[1] - endEffPose.p(1);
        errors[2] = desiredPos[2] - endEffPose.p(2);
        errors[3] = desiredTwist[0] - endEffTwist.p.v(0);
        errors[4] = desiredTwist[1] - endEffTwist.p.v(1);
        errors[5] = desiredTwist[2] - endEffTwist.p.v(2);

        for (int i = 0; i < 6; i++) {
            abag_sched(&abagStates[i], &errors[i], &commands[i], &(pAbagConfigs.at(kcc_conf::ALPHA)[i]),
                &(pAbagConfigs.at(kcc_conf::BIAS_THRES)[i]), &(pAbagConfigs.at(kcc_conf::BIAS_STEP)[i]),
                &(pAbagConfigs.at(kcc_conf::GAIN_THRES)[i]), &(pAbagConfigs.at(kcc_conf::GAIN_STEP)[i]));
        }

        endEffForce(0) = commands[0] * pCartForceLimits[0];
        endEffForce(1) = commands[1] * pCartForceLimits[1];
        endEffForce(2) = commands[2] * pCartForceLimits[2];

        endEffForce(0) += commands[3] * pCartForceLimits[3];
        endEffForce(1) += commands[4] * pCartForceLimits[4];
        endEffForce(2) += commands[5] * pCartForceLimits[5];

        // std::cout << "ee force: " << endEffForce(0) << ", " << endEffForce(1) << ", " << endEffForce(2) << std::endl;

        jntImpedanceTorques.data = jacobianEndEff.data.transpose() * endEffForce;

        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            jntCmdTorques(i) = jntCmdTorques(i) + jntImpedanceTorques(i);
            if      (jntCmdTorques(i) >=  kc_const::kinova::JOINT_TORQUE_LIMITS[i])
                jntCmdTorques(i) =  kc_const::kinova::JOINT_TORQUE_LIMITS[i] - 0.001;
            else if (jntCmdTorques(i) <= -kc_const::kinova::JOINT_TORQUE_LIMITS[i])
                jntCmdTorques(i) = -kc_const::kinova::JOINT_TORQUE_LIMITS[i] + 0.001;
            baseCmd.mutable_actuators(i)->set_position(baseFb.actuators(i).position());
            baseCmd.mutable_actuators(i)->set_torque_joint(jntCmdTorques(i));
        }

        // control
        kc::writeDataRow(logPosX, abagStates[0], totalElapsedTime.count(), errors[0], commands[0], endEffPose.p(0));
        kc::writeDataRow(logPosY, abagStates[1], totalElapsedTime.count(), errors[1], commands[1], endEffPose.p(1));
        kc::writeDataRow(logPosZ, abagStates[2], totalElapsedTime.count(), errors[2], commands[2], endEffPose.p(2));
        kc::writeDataRow(logVelX, abagStates[3], totalElapsedTime.count(), errors[3], commands[3], endEffTwist.p.v(0));
        kc::writeDataRow(logVelY, abagStates[4], totalElapsedTime.count(), errors[4], commands[4], endEffTwist.p.v(1));
        kc::writeDataRow(logVelZ, abagStates[5], totalElapsedTime.count(), errors[5], commands[5], endEffTwist.p.v(2));

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
            kc::stopRobot(actuator_config);
            if (desiredValuesLog.is_open()) desiredValuesLog.close();
            if (logPosX.is_open()) logPosX.close();
            if (logPosY.is_open()) logPosY.close();
            if (logPosZ.is_open()) logPosZ.close();
            if (logVelX.is_open()) logVelX.close();
            if (logVelY.is_open()) logVelY.close();
            if (logVelZ.is_open()) logVelZ.close();
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
    servoingModeInfo.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingModeInfo);

    // close log files
    if (logPosX.is_open()) logPosX.close();
    if (logPosY.is_open()) logPosY.close();
    if (logPosZ.is_open()) logPosZ.close();
    if (logVelX.is_open()) logVelX.close();
    if (logVelY.is_open()) logVelY.close();
    if (logVelZ.is_open()) logVelZ.close();

    // Wait for a bit
    std::this_thread::sleep_for(sc::milliseconds(2000));
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
        // Establish connection with arm
        kc::KinovaBaseConnection connection(hostName, port, portRealTime, username, password);

        try
        {
            // Move to safe position
            kc::move_to_home_position(connection.mBaseClient.get());
            // execute controller
            impedance_control_posvel(
                connection.mBaseClient.get(), connection.mBaseCyclicClient.get(), connection.mActuatorConfigClient.get(),
                kinovaChain, abagConfigs, cartForceLimits);
        }
        catch (k_api::KDetailedException& ex)
        {
            kc::stopRobot(connection.mActuatorConfigClient.get());
            kc::printKinovaException(ex);
            return EXIT_FAILURE;
        }
        catch (const std::exception& ex)
        {
            kc::stopRobot(connection.mActuatorConfigClient.get());
            std::cout << "Unhandled Error: " << ex.what() << std::endl;
            return EXIT_FAILURE;
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << "error while establishing connection: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
