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

#define IP_ADDRESS "192.168.1.10"
#define PORT 10000
#define PORT_REAL_TIME 10001

sc::duration <double, std::micro> loop_interval{};

void example_cyclic_torque_control (
    k_api::Base::BaseClient* base,
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
    k_api::ActuatorConfig::ActuatorConfigClient* actuator_config
)
{
    const std::string UrdfPath = REPO_DIR + kc_const::kinova::URDF_PATH;
    std::cout << "loading URDF file at: " << UrdfPath << std::endl;
    KDL::Tree kinovaTree; KDL::Chain kinovaChain;
    kc::loadUrdfModel(UrdfPath, kinovaTree, kinovaChain);
    // Initialize solvers
    const KDL::JntArray ZERO_ARRAY(7);
    const KDL::Wrenches ZERO_WRENCHES(kinovaChain.getNrOfSegments(), KDL::Wrench::Zero());

    KDL::Jacobian jacobianEndEff(kinovaChain.getNrOfJoints());
    KDL::Frame endEffPose;
    Eigen::Matrix<double, 6, 1> endEffForce;
    endEffForce.setZero();

    int returnFlag = 0;
    KDL::ChainJntToJacSolver jacobSolver(kinovaChain);
    KDL::ChainFkSolverPos_recursive fkSolverPos(kinovaChain);
    auto idSolver = std::make_shared<KDL::ChainIdSolver_RNE>(kinovaChain, KDL::Vector(0.0, 0.0, -9.81289));

    KDL::JntArray jntCmdTorques(7), jntPositions(7), jntVelocities(7), jnt_torque(7), jntImpedanceTorques(7);

    KDL::FrameVel endEffTwist;
    KDL::ChainFkSolverVel_recursive fkSolverVel(kinovaChain);
    KDL::JntArrayVel jntPositionVelocity(7);

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

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;
    auto servoing_mode = k_api::Base::ServoingModeInformation();

    std::cout << "Initializing the arm for torque control example" << std::endl;

    // Set the base in low-level servoing mode
    try {
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();
    } catch (...) {
        std::cerr << "error setting low level control mode" << std::endl;
        throw;
    }

    // Initialize each actuator to their current position
    // Save the current actuator position, to avoid a following error
    for (int i = 0; i < ACTUATOR_COUNT; i++)
        base_command.add_actuators()->set_position(base_feedback.actuators(i).position());

    // Send a first frame
    base_feedback = base_cyclic->Refresh(base_command);

    // Set last actuator in torque mode now that the command is equal to measure
    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

    for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
        actuator_config->SetControlMode(control_mode_message, actuator_id);

    const std::vector<double> cart_force_limit {5.0, 5.0, 5.0, 5.0, 5.0, 5.0}; // N

    // ABAG parameters
    const double alphas[6] = { 0.9, 0.9, 0.95, 0.9, 0.9, 0.95 };
    const double biasThresholds[6] = { 0.000407, 0.000407, 0.000407, 0.000407, 0.000407, 0.000407 };
    const double biasSteps[6] = { 0.000400, 0.000400, 0.000400, 0.000400, 0.000400, 0.000400 };
    const double gainThresholds[6] = { 0.502492, 0.55, 0.550000, 0.502492, 0.55, 0.550000 };
    const double gainSteps[6] = { 0.002, 0.003, 0.003, 0.002, 0.003, 0.003 };
    double desiredPosistion[3] = { 0.0 };
    const double desiredTwist[3] = { 0.01, 0.01, 0.01 };
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
            base_feedback = base_cyclic->RefreshFeedback();
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
            jntPositions(i) = DEG_TO_RAD(base_feedback.actuators(i).position()); // deg
            jntVelocities(i) = DEG_TO_RAD(base_feedback.actuators(i).velocity()); // deg
            jnt_torque(i)   = base_feedback.actuators(i).torque(); // nm
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
            desiredPosistion[0] = endEffPose.p(0) + 0.10;
            desiredPosistion[1] = endEffPose.p(1) + 0.10;
            desiredPosistion[2] = endEffPose.p(2) + 0.10;
            desiredValuesLog << desiredPosistion[0] << "," << desiredPosistion[1] << "," << desiredPosistion[2] << ","
                             << desiredTwist[0] << "," << desiredTwist[1] << "," << desiredTwist[2] << std::endl;
            if (desiredValuesLog.is_open()) desiredValuesLog.close();
        }

        errors[0] = desiredPosistion[0] - endEffPose.p(0);
        errors[1] = desiredPosistion[1] - endEffPose.p(1);
        errors[2] = desiredPosistion[2] - endEffPose.p(2);
        errors[3] = desiredTwist[0] - endEffTwist.p.v(0);
        errors[4] = desiredTwist[1] - endEffTwist.p.v(1);
        errors[5] = desiredTwist[2] - endEffTwist.p.v(2);

        abag_sched(&abagStates[0], &errors[0], &commands[0], &alphas[0],
                   &biasThresholds[0], &biasSteps[0], &gainThresholds[0], &gainSteps[0]);
        abag_sched(&abagStates[1], &errors[1], &commands[1], &alphas[1],
                   &biasThresholds[1], &biasSteps[1], &gainThresholds[1], &gainSteps[1]);
        abag_sched(&abagStates[2], &errors[2], &commands[2], &alphas[2],
                   &biasThresholds[2], &biasSteps[2], &gainThresholds[2], &gainSteps[2]);
        abag_sched(&abagStates[3], &errors[3], &commands[3], &alphas[3],
                   &biasThresholds[3], &biasSteps[3], &gainThresholds[3], &gainSteps[3]);
        abag_sched(&abagStates[4], &errors[4], &commands[4], &alphas[4],
                   &biasThresholds[4], &biasSteps[4], &gainThresholds[4], &gainSteps[4]);
        abag_sched(&abagStates[5], &errors[5], &commands[5], &alphas[5],
                   &biasThresholds[5], &biasSteps[5], &gainThresholds[5], &gainSteps[5]);

        endEffForce(0) = commands[0] * cart_force_limit[0];
        endEffForce(1) = commands[1] * cart_force_limit[1];
        endEffForce(2) = commands[2] * cart_force_limit[2];

        endEffForce(0) += commands[3] * cart_force_limit[3];
        endEffForce(1) += commands[4] * cart_force_limit[4];
        endEffForce(2) += commands[5] * cart_force_limit[5];

        // std::cout << "ee force: " << endEffForce(0) << ", " << endEffForce(1) << ", " << endEffForce(2) << std::endl;

        jntImpedanceTorques.data = jacobianEndEff.data.transpose() * endEffForce;

        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            jntCmdTorques(i) = jntCmdTorques(i) + jntImpedanceTorques(i);
            if      (jntCmdTorques(i) >=  kc_const::kinova::JOINT_TORQUE_LIMITS[i])
                jntCmdTorques(i) =  kc_const::kinova::JOINT_TORQUE_LIMITS[i] - 0.001;
            else if (jntCmdTorques(i) <= -kc_const::kinova::JOINT_TORQUE_LIMITS[i])
                jntCmdTorques(i) = -kc_const::kinova::JOINT_TORQUE_LIMITS[i] + 0.001;
            base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
            base_command.mutable_actuators(i)->set_torque_joint(jntCmdTorques(i));
        }

        // control
        kc::writeDataRow(logPosX, abagStates[0], totalElapsedTime.count(), errors[0], commands[0], endEffPose.p(0));
        kc::writeDataRow(logPosY, abagStates[1], totalElapsedTime.count(), errors[1], commands[1], endEffPose.p(1));
        kc::writeDataRow(logPosZ, abagStates[2], totalElapsedTime.count(), errors[2], commands[2], endEffPose.p(2));
        kc::writeDataRow(logVelX, abagStates[3], totalElapsedTime.count(), errors[3], commands[3], endEffTwist.p.v(0));
        kc::writeDataRow(logVelY, abagStates[4], totalElapsedTime.count(), errors[4], commands[4], endEffTwist.p.v(1));
        kc::writeDataRow(logVelZ, abagStates[5], totalElapsedTime.count(), errors[5], commands[5], endEffTwist.p.v(2));

        // Incrementing identifier ensures actuators can reject out of time frames
        base_command.set_frame_id(base_command.frame_id() + 1);
        if (base_command.frame_id() > 65535) base_command.set_frame_id(0);

        for (int idx = 0; idx < ACTUATOR_COUNT; idx++)
            base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());

        try
        {
            base_feedback = base_cyclic->Refresh(base_command, 0);
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

    // actuator_config->SetControlMode(control_mode_message, last_actuator_device_id);
    std::cout << "Torque control example completed" << std::endl;
    std::cout << "Number of loops which took longer than specified duration: " << slowLoopCount << std::endl;

    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

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
    try
    {
        libconfig::Setting& root = kc::loadConfigFile(cfg, REPO_DIR"config/abag.cfg");
        std::map<std::string, std::vector<double>> abagConfigs;
        kc::loadAbagConfig(root, CTRL_APPROACH, abagConfigs);
        for (auto& alpha : abagConfigs[kc_const::ALPHA]) {
            std::cout << alpha << std::endl;
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << "failed to load configuration file: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Establish connection with arm
    kc::KinovaBaseConnection connection(IP_ADDRESS, PORT, PORT_REAL_TIME, "admin", "kinova1_area4251");

    // Move to safe position
    kc::move_to_home_position(connection.mBaseClient.get());
    try
    {
        // execute controller
        example_cyclic_torque_control(
            connection.mBaseClient.get(), connection.mBaseCyclicClient.get(), connection.mActuatorConfigClient.get());
    }
    catch (k_api::KDetailedException& ex)
    {
        kc::stopRobot(connection.mActuatorConfigClient.get());
        kc::handleKinovaException(ex);
        return EXIT_FAILURE;
    }
    catch (std::exception& ex)
    {
        kc::stopRobot(connection.mActuatorConfigClient.get());
        std::cout << "Unhandled Error: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
