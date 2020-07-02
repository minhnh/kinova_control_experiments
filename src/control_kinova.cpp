/*
* Copyright (c) 2020 Minh Nguyen inc. All rights reserved.
*
*/
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <stdlib.h> /* abs */
#include <chrono>
#include <time.h>

#include "kinova_util.h"
#include "abag.h"
#include "constants.hpp"

#include <unistd.h>

#define REPO_DIR "/home/minh/workspace/kinova_control_experiments/"

namespace k_api = Kinova::Api;
namespace sc = std::chrono;

#define IP_ADDRESS "192.168.1.10"

#define PORT 10000
#define PORT_REAL_TIME 10001
#define DEG_TO_RAD(x) (x) * PI / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / PI
#define ACTUATOR_COUNT 7

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

    int last_actuator_device_id = 7;
    actuator_config->SetControlMode(control_mode_message, last_actuator_device_id);


    const std::vector<double> joint_torque_limits {39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0};
    const double alpha = 0.8;
    double desired_vel = 6.0, current_vel = 0.0; //deg/s
    double error = 0.0;
    double abag_command = 0.0;
    abagState_t abagState;
    initialize_abagState(&abagState);
    FILE * fp = fopen("simulated_data.csv", "w+");
    fprintf(fp, "error, signed, bias, gain, e_bar, e_bar_prev, actuation, currentVel\n");

    // Real-time loop
    const int SECOND_IN_MICROSECONDS = 1000000;
    const int RATE_HZ = 600; // Hz
    const int TASK_TIME_LIMIT_SEC = 30;
    const sc::microseconds TASK_TIME_LIMIT_MICRO(TASK_TIME_LIMIT_SEC * SECOND_IN_MICROSECONDS);
    const sc::microseconds LOOP_DURATION(SECOND_IN_MICROSECONDS / RATE_HZ);

    int slowLoopCount = 0;
    sc::time_point<sc::steady_clock> controlStartTime = sc::steady_clock::now();;
    sc::time_point<sc::steady_clock> loopStartTime;
    sc::duration<int64_t, nano> totalElapsedTime;
    while (loopStartTime - controlStartTime < TASK_TIME_LIMIT_MICRO)
    {
        loopStartTime = sc::steady_clock::now();
        totalElapsedTime = loopStartTime - controlStartTime;

        try
        {
            base_feedback = base_cyclic->RefreshFeedback();
        }
        catch(...)
        {
            if (fp != nullptr) fclose(fp);
            std::cerr << "error reading sensors" << std::endl;
            throw;
        }

        // for (int i = 0; i < ACTUATOR_COUNT; i++)
        // {
        //     jnt_position(i) = DEG_TO_RAD(base_feedback.actuators(i).position()); //deg
        //     jnt_velocity(i) = DEG_TO_RAD(base_feedback.actuators(i).velocity()); // deg
        //     jnt_torque(i)   = base_feedback.actuators(i).torque(); //nm
        // }

        // Kinova API provides only positive angle values
        // This operation is required to align the logic with our safety monitor
        // We need to convert some angles to negative values
        // if (jnt_position(1) > DEG_TO_RAD(180.0)) jnt_position(1) = jnt_position(1) - DEG_TO_RAD(360.0);
        // if (jnt_position(3) > DEG_TO_RAD(180.0)) jnt_position(3) = jnt_position(3) - DEG_TO_RAD(360.0);
        // if (jnt_position(5) > DEG_TO_RAD(180.0)) jnt_position(5) = jnt_position(5) - DEG_TO_RAD(360.0);

        // for (int i = 0; i < ACTUATOR_COUNT; i++)
        // {
        //     base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
        //     base_command.mutable_actuators(i)->set_torque_joint(jnt_command_torque(i));
        // }

        current_vel = base_feedback.actuators(6).velocity();
        if (totalElapsedTime > TASK_TIME_LIMIT_MICRO * 0.1 && totalElapsedTime < TASK_TIME_LIMIT_MICRO * 0.9) {
            error = desired_vel - current_vel;
        } else {
            error = 0.0 - current_vel;
        }

        fprintf(fp, "%.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n",
            error, abagState.signedErr_access, abagState.bias_access, abagState.gain_access,abagState.eBar_access,
            abag_command, current_vel);
        abag_sched(&abagState, &error, &abag_command, &alpha);

        base_command.mutable_actuators(6)->set_position(base_feedback.actuators(6).position());

        base_command.mutable_actuators(6)->set_torque_joint(abag_command * joint_torque_limits[6] * 0.8);
        // base_command.mutable_actuators(6)->set_torque_joint(1.0);

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
            if (fp != nullptr) fclose(fp);
            std::cerr << "error sending command" << std::endl;
            throw;
        }

        // Enforce the constant loop time and count how many times the loop was late
        if (waitMicroSeconds(loopStartTime, LOOP_DURATION) != 0) slowLoopCount++;
    }

    if (fp != nullptr) fclose(fp);

    std::cout << "Torque control example completed" << std::endl;
    std::cout << "Number of loops which took longer than specified duration: " << slowLoopCount << std::endl;
    // Set first actuator back in position
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    actuator_config->SetControlMode(control_mode_message, last_actuator_device_id);
    std::cout << "Torque control example clean exit" << std::endl;

    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

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
        handleKinovaException(ex);
        return EXIT_FAILURE;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}