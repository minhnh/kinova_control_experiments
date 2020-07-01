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

#include <unistd.h>

namespace k_api = Kinova::Api;
const int SECOND = 1000000;

#define IP_ADDRESS "192.168.1.10"

#define PORT 10000
#define PORT_REAL_TIME 10001
#define DEG_TO_RAD(x) (x) * PI / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / PI
#define ACTUATOR_COUNT 7

float TIME_DURATION = 5.0f; // Duration of the example (seconds)

std::chrono::steady_clock::time_point loop_start_time;
std::chrono::duration <double, std::micro> loop_interval{};

//Make sure that the control loop runs exactly with the specified frequency
int enforce_loop_frequency(const int dt)
{
    loop_interval = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time);

    if (loop_interval < std::chrono::microseconds(dt)) // Loop is sufficiently fast
    {
        while (loop_interval < std::chrono::microseconds(dt - 1))
            loop_interval = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time);

        return 0;
    }
    else return -1; //Loop is too slow
}

bool example_cyclic_torque_control (
    k_api::Base::BaseClient* base,
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
    k_api::ActuatorConfig::ActuatorConfigClient* actuator_config
)
{
    const int RATE_HZ = 600; // Hz
    const int DT_MICRO = SECOND / RATE_HZ;
    const double DT_SEC = 1.0 / static_cast<double>(RATE_HZ);
    double total_time_sec = 0.0;
    double task_time_limit_sec = 60.0;
    int iteration_count = 0;
    int control_loop_delay_count = 0;
    int return_flag = 0;
    bool return_status = true;

    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch(...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;
    auto servoing_mode = k_api::Base::ServoingModeInformation();

    std::cout << "Initializing the arm for torque control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

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
        while (total_time_sec < task_time_limit_sec)
        {
            loop_start_time = std::chrono::steady_clock::now();
            iteration_count++;
            total_time_sec = iteration_count * DT_SEC;

            try
            {
                base_feedback = base_cyclic->RefreshFeedback();
            }
            catch (Kinova::Api::KDetailedException& ex)
            {
                std::cout << "Kortex exception 1: " << ex.what() << std::endl;
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
            if (total_time_sec > task_time_limit_sec * 0.1 && total_time_sec < task_time_limit_sec * 0.9) {
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
            catch (k_api::KDetailedException& ex)
            {
                std::cout << "Kortex exception: " << ex.what() << std::endl;

                std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
            }
            catch (std::runtime_error& ex2)
            {
                std::cout << "runtime error: " << ex2.what() << std::endl;
            }
            catch(...)
            {
                std::cout << "Unknown error." << std::endl;
            }

            // Enforce the constant loop time and count how many times the loop was late
            if (enforce_loop_frequency(DT_MICRO) != 0) control_loop_delay_count++;
        }
        fclose(fp);

        std::cout << "Torque control example completed" << std::endl;

        // Set first actuator back in position
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, last_actuator_device_id);

        std::cout << "Torque control example clean exit" << std::endl;

    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

int main(int argc, char **argv)
{
    KinovaBaseConnection connection(IP_ADDRESS, PORT, PORT_REAL_TIME, "admin", "kinova1_area4251");

    // Example core
    move_to_home_position(connection.mBaseClient.get());
    auto isOk = example_cyclic_torque_control(connection.mBaseClient.get(), connection.mBaseCyclicClient.get(), connection.mActuatorConfigClient.get());
    if (!isOk)
    {
        std::cout << "There has been an unexpected error in example_cyclic_torque_control() function." << endl;;
    }
}