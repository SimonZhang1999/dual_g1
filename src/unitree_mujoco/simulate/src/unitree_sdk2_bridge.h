#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "param.h"
#include "physics_joystick.h"

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(
        mjModel *model,
        mjData *data,
        const std::string &joint_prefix = "",
        int expected_motor_count = 0)
    : mj_model_(model), mj_data_(data), joint_prefix_(joint_prefix),
      expected_motor_count_(expected_motor_count)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    virtual void start() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;
    std::string joint_prefix_;
    int expected_motor_count_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    std::vector<int> actuator_ids_;
    std::vector<int> motor_pos_adr_;
    std::vector<int> motor_vel_adr_;
    std::vector<int> motor_torque_adr_;

    // Sensor data indices
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    inline static constexpr std::array<const char *, 29> kG1MotorNames = {
        "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
        "left_ankle_pitch", "left_ankle_roll",
        "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
        "right_ankle_pitch", "right_ankle_roll",
        "waist_yaw", "waist_roll", "waist_pitch",
        "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
        "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
        "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
        "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw"};

    int sensorAddress(const std::string &name) const
    {
        const int sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, name.c_str());
        return sensor_id >= 0 ? mj_model_->sensor_adr[sensor_id] : -1;
    }

    void _check_sensor()
    {
        num_motor_ = expected_motor_count_ > 0 ? expected_motor_count_ : mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

        actuator_ids_.assign(num_motor_, -1);
        motor_pos_adr_.assign(num_motor_, -1);
        motor_vel_adr_.assign(num_motor_, -1);
        motor_torque_adr_.assign(num_motor_, -1);

        const bool use_named_g1_layout = expected_motor_count_ == static_cast<int>(kG1MotorNames.size());
        for (int i = 0; i < num_motor_; ++i)
        {
            if (use_named_g1_layout)
            {
                const std::string motor_name = joint_prefix_ + kG1MotorNames[i];
                actuator_ids_[i] = mj_name2id(mj_model_, mjOBJ_ACTUATOR, motor_name.c_str());
                motor_pos_adr_[i] = sensorAddress(motor_name + "_pos");
                motor_vel_adr_[i] = sensorAddress(motor_name + "_vel");
                motor_torque_adr_[i] = sensorAddress(motor_name + "_torque");
                if (actuator_ids_[i] < 0 || motor_pos_adr_[i] < 0 || motor_vel_adr_[i] < 0)
                {
                    std::cerr << "Missing MuJoCo mapping for motor " << motor_name
                              << " actuator=" << actuator_ids_[i]
                              << " pos=" << motor_pos_adr_[i]
                              << " vel=" << motor_vel_adr_[i] << std::endl;
                }
            }
            else
            {
                actuator_ids_[i] = i;
                motor_pos_adr_[i] = i;
                motor_vel_adr_[i] = i + num_motor_;
                motor_torque_adr_[i] = i + 2 * num_motor_;
            }
        }
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, (joint_prefix_ + "imu_quat").c_str());
        if (sensor_id >= 0) {
            imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, (joint_prefix_ + "imu_gyro").c_str());
        if (sensor_id >= 0) {
            imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, (joint_prefix_ + "imu_acc").c_str());
        if (sensor_id >= 0) {
            imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, (joint_prefix_ + "frame_pos").c_str());
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, (joint_prefix_ + "frame_vel").c_str());
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, (joint_prefix_ + "secondary_imu_quat").c_str());
        if (sensor_id >= 0) {
            secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, (joint_prefix_ + "secondary_imu_gyro").c_str());
        if (sensor_id >= 0) {
            secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, (joint_prefix_ + "secondary_imu_acc").c_str());
        if (sensor_id >= 0) {
            secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(
        mjModel *model,
        mjData *data,
        const std::string &joint_prefix = "",
        int expected_motor_count = 0,
        const std::string &lowcmd_topic = "rt/lowcmd",
        const std::string &lowstate_topic = "rt/lowstate",
        const std::string &highstate_topic = "rt/sportmodestate",
        const std::string &wireless_topic = "rt/wirelesscontroller")
    : UnitreeSDK2BridgeBase(model, data, joint_prefix, expected_motor_count)
    {
        lowcmd = std::make_shared<LowCmd_t>(lowcmd_topic);
        lowstate = std::make_unique<LowState_t>(lowstate_topic);
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>(highstate_topic);
        wireless_controller = std::make_unique<WirelessController_t>(wireless_topic);
        wireless_controller->joystick = joystick;
        std::cout << "Unitree bridge " << (joint_prefix.empty() ? "leader" : joint_prefix)
                  << " lowcmd=" << lowcmd_topic << " lowstate=" << lowstate_topic
                  << " motors=" << num_motor_ << std::endl;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                if (actuator_ids_[i] < 0 || motor_pos_adr_[i] < 0 || motor_vel_adr_[i] < 0)
                {
                    continue;
                }
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[actuator_ids_[i]] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[motor_pos_adr_[i]]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[motor_vel_adr_[i]]);
            }
        }

        // lowstate
        if(lowstate->trylock()) {
            for(int i(0); i<num_motor_; i++) {
                if (motor_pos_adr_[i] >= 0) {
                    lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[motor_pos_adr_[i]];
                }
                if (motor_vel_adr_[i] >= 0) {
                    lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[motor_vel_adr_[i]];
                }
                if (motor_torque_adr_[i] >= 0) {
                    lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[motor_torque_adr_[i]];
                }
            }
            
            if(imu_quat_adr_ >= 0) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }
            
            if(imu_gyro_adr_ >= 0) {
                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
            }

            if(imu_acc_adr_ >= 0) {
                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
            }
            
            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate->unlockAndPublish();
        }
        // highstate
        if(highstate->trylock()) {
            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;
    
private:
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(
        mjModel *model,
        mjData *data,
        const std::string &joint_prefix = "",
        const std::string &lowcmd_topic = "rt/lowcmd",
        const std::string &lowstate_topic = "rt/lowstate",
        const std::string &highstate_topic = "rt/sportmodestate",
        const std::string &wireless_topic = "rt/wirelesscontroller",
        const std::string &secondary_imu_topic = "rt/secondary_imu",
        const std::string &bmsstate_topic = "rt/lf/bmsstate")
    : RobotBridge(
          model, data, joint_prefix, static_cast<int>(kG1MotorNames.size()),
          lowcmd_topic, lowstate_topic, highstate_topic, wireless_topic)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>(bmsstate_topic);
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>(secondary_imu_topic);
    }

    void run() override
    {
        RobotBridge::run();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;
};
