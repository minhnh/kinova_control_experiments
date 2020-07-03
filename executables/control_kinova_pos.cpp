/**
* @copyright 2020 Minh Nguyen
*/
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <stdlib.h> /* abs */
#include <chrono>
#include <time.h>
#include <unistd.h>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "kinova_util.h"
#include "abag.h"
#include "constants.hpp"

#define REPO_DIR "/home/minh/workspace/kinova_control_experiments/"

namespace k_api = Kinova::Api;
namespace sc = std::chrono;

#define IP_ADDRESS "192.168.1.10"
#define PORT 10000
#define PORT_REAL_TIME 10001

std::chrono::duration <double, std::micro> loop_interval{};

void example_cyclic_torque_control (
    k_api::Base::BaseClient* base,
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
    k_api::ActuatorConfig::ActuatorConfigClient* actuator_config
)
{
    const std::string UrdfPath = REPO_DIR + constants::kinova::URDF_PATH;
    std::cout << "loading URDF file at: " << UrdfPath << std::endl;
    KDL::Tree kinovaTree; KDL::Chain kinovaChain;
    loadUrdfModel(UrdfPath, kinovaTree, kinovaChain);
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
    std::shared_ptr<KDL::ChainIdSolver_RNE> idSolver = std::make_shared<KDL::ChainIdSolver_RNE>(kinovaChain,
        KDL::Vector(0.0, 0.0, -9.81289));

    KDL::JntArray jntCmdTorques(7), jntPositions(7), jntVelocities(7), jnt_torque(7), jntImpedanceTorques(7);

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

    const std::vector<double> joint_torque_limits {39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0};
    const std::vector<double> cart_force_limit {5.0, 5.0, 5.0, 5.0, 5.0, 5.0}; // N

    // ABAG parameters
    const double alphaPosition[3] = { 0.8, 0.9, 0.95 };
    const double biasThresPos[3] = { 0.000407, 0.000407, 0.000407 };
    const double biasStepPos[3] = { 0.000400, 0.000400, 0.000400 };
    const double gainThresPos[3] = { 0.502492, 0.55, 0.550000 };
    const double gainStepPos[3] = { 0.002, 0.003, 0.003 };
    double desiredPosistion[3] = { 0.0 };
    double errorPos[3] = { 0.0 };
    double commandPos[3] = { 0.0 };
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
    desiredPositionLog << "position X, position Y, position Z" << std::endl;
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
            base_feedback = base_cyclic->RefreshFeedback();
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

        // returnFlag = fk_solver_vel.JntToCart(jntPositions, jnt_vel_, end_eff_twist);
        // if (returnFlag != 0) break;

        if (iterationCount == 1)
        {
            desiredPosistion[0] = endEffPose.p(0) + 0.03;
            desiredPosistion[1] = endEffPose.p(1) + 0.03;
            desiredPosistion[2] = endEffPose.p(2) + 0.03;
            desiredPositionLog << desiredPosistion[0] << "," << desiredPosistion[1] << ","
                               << desiredPosistion[2] << std::endl;
            if (desiredPositionLog.is_open()) desiredPositionLog.close();
        }

        errorPos[0] = desiredPosistion[0] - endEffPose.p(0);
        errorPos[1] = desiredPosistion[1] - endEffPose.p(1);
        errorPos[2] = desiredPosistion[2] - endEffPose.p(2);

        abag_sched(&abagStatePos[0], &errorPos[0], &commandPos[0], &alphaPosition[0],
                   &biasThresPos[0], &biasStepPos[0], &gainThresPos[0], &gainStepPos[0]);
        abag_sched(&abagStatePos[1], &errorPos[1], &commandPos[1], &alphaPosition[1],
                   &biasThresPos[1], &biasStepPos[1], &gainThresPos[1], &gainStepPos[1]);
        abag_sched(&abagStatePos[2], &errorPos[2], &commandPos[2], &alphaPosition[2],
                   &biasThresPos[2], &biasStepPos[2], &gainThresPos[2], &gainStepPos[2]);

        endEffForce(0) = commandPos[0] * cart_force_limit[0];
        endEffForce(1) = commandPos[1] * cart_force_limit[1];
        endEffForce(2) = commandPos[2] * cart_force_limit[2];
        // std::cout << end_eff_force.transpose() << std::endl;

        jntImpedanceTorques.data = jacobianEndEff.data.transpose() * endEffForce;

        // std::cout << errorX << ", " << errorY << ", " << errorZ << std::endl;
        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            jntCmdTorques(i) = jntCmdTorques(i) + jntImpedanceTorques(i);
            if      (jntCmdTorques(i) >=  joint_torque_limits[i]) jntCmdTorques(i) =  joint_torque_limits[i] - 0.001;
            else if (jntCmdTorques(i) <= -joint_torque_limits[i]) jntCmdTorques(i) = -joint_torque_limits[i] + 0.001;
            base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
            base_command.mutable_actuators(i)->set_torque_joint(jntCmdTorques(i));
        }

        // std::cout << jntCmdTorques.data.transpose() << std::endl;
        // control
        writeDataRow(logPosX, abagStatePos[0], totalElapsedTime.count(), errorPos[0], commandPos[0], endEffPose.p(0));
        writeDataRow(logPosY, abagStatePos[1], totalElapsedTime.count(), errorPos[1], commandPos[1], endEffPose.p(1));
        writeDataRow(logPosZ, abagStatePos[2], totalElapsedTime.count(), errorPos[2], commandPos[2], endEffPose.p(2));

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
            if (desiredPositionLog.is_open()) desiredPositionLog.close();
            if (logPosX.is_open()) logPosX.close();
            if (logPosY.is_open()) logPosY.close();
            if (logPosZ.is_open()) logPosZ.close();
            std::cerr << "error sending command" << std::endl;
            throw;
        }

        // Enforce the constant loop time and count how many times the loop was late
        if (waitMicroSeconds(loopStartTime, LOOP_DURATION) != 0) slowLoopCount++;
    }

    // Set first actuator back in position
    stopRobot(actuator_config);

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

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

int main(int argc, char **argv)
{
    KinovaBaseConnection connection(IP_ADDRESS, PORT, PORT_REAL_TIME, "admin", "kinova1_area4251");

    // Example core
    move_to_home_position(connection.mBaseClient.get());
    try
    {
        example_cyclic_torque_control(connection.mBaseClient.get(), connection.mBaseCyclicClient.get(), connection.mActuatorConfigClient.get());
    }
    catch (k_api::KDetailedException& ex)
    {
        stopRobot(connection.mActuatorConfigClient.get());
        handleKinovaException(ex);
        return EXIT_FAILURE;
    }
    catch (std::exception& ex)
    {
        stopRobot(connection.mActuatorConfigClient.get());
        std::cout << "Unhandled Error: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
