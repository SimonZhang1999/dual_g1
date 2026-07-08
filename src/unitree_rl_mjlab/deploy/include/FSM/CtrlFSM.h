// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <chrono>
#include <vector>
#include <unitree/common/thread/recurrent_thread.hpp>
#include "BaseState.h"
#include "input/fixstand_lock_source.h"
#include "input/velocity_command_source.h"
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

class CtrlFSM
{
public:
    CtrlFSM(std::shared_ptr<BaseState> initstate)
    {
        // Initialize FSM states
        states.push_back(std::move(initstate));

    }

    CtrlFSM(YAML::Node cfg, bool auto_mode = false, bool auto_to_velocity = true)
    : auto_mode_(auto_mode), auto_to_velocity_(auto_to_velocity)
    {
        auto fsms = cfg["_"]; // enabled FSMs

        // register FSM string map; used for state transition
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            FSMStringMap.insert({id, fsm_name});
        }

        // Initialize FSM states
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            std::string fsm_type = it->second["type"] ? it->second["type"].as<std::string>() : fsm_name;
            auto fsm_class = getFsmMap().find("State_" + fsm_type);
            if (fsm_class == getFsmMap().end()) {
                throw std::runtime_error("FSM: Unknown FSM type " + fsm_type);
            }
            auto state_instance = fsm_class->second(id, fsm_name);
            add(state_instance);
        }

        if (cfg["FixStand"]["ts"]) {
            auto ts = cfg["FixStand"]["ts"].as<std::vector<float>>();
            if (!ts.empty()) {
                auto_fixstand_wait_ = std::chrono::duration<double>(ts.back() + 0.2f);
            }
        }

        // In auto-start mode, keep Passive very short so the robot does not collapse
        // before FixStand gains are applied.
        if (auto_mode_) {
            auto_passive_wait_ = std::chrono::duration<double>(0.15);
        }

        if (cfg["Passive"]["auto_wait"]) {
            auto_passive_wait_ = std::chrono::duration<double>(cfg["Passive"]["auto_wait"].as<double>());
        }
        if (cfg["FixStand"]["auto_wait"]) {
            auto_fixstand_wait_ = std::chrono::duration<double>(cfg["FixStand"]["auto_wait"].as<double>());
        }
    }

    void start() 
    {
        // Start From State_Passive
        input::FixStandLockSource::setActive(true);
        currentState = states[0];
        currentState->enter();
        state_enter_time_ = std::chrono::steady_clock::now();

        fsm_thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "FSM", 0, this->dt * 1e6, &CtrlFSM::run_, this);
        spdlog::info("FSM: Start {}", currentState->getStateString());
    }

    void add(std::shared_ptr<BaseState> state)
    {
        for(auto & s : states)
        {
            if(s->isState(state->getState()))
            {
                spdlog::error("FSM: State_{} already exists", state->getStateString());
                std::exit(0);
            }
        }

        states.push_back(std::move(state));
    }
    
    ~CtrlFSM()
    {
        states.clear();
    }

    std::vector<std::shared_ptr<BaseState>> states;
private:
    const double dt = 0.001;
    bool auto_mode_ = false;
    bool auto_to_velocity_ = true;
    std::chrono::steady_clock::time_point state_enter_time_;
    std::chrono::duration<double> auto_passive_wait_{1.5};
    std::chrono::duration<double> auto_fixstand_wait_{2.2};

    void run_()
    {
        currentState->pre_run();
        currentState->run();
        currentState->post_run();
        input::FixStandLockSource::setActive(currentState->getStateString() != "Velocity");
        
        // Check if need to change state
        int nextStateMode = autoTransition();
        if(nextStateMode == 0)
        {
            for(int i(0); i<currentState->registered_checks.size(); i++)
            {
                if(currentState->registered_checks[i].first())
                {
                    nextStateMode = currentState->registered_checks[i].second;
                    break;
                }
            }
        }

        if(nextStateMode == 0 && auto_to_velocity_ && currentState->getStateString() == "FixStand")
        {
            if(input::VelocityCommandSource::hasActiveCommand())
            {
                nextStateMode = FSMStringMap.right.at("Velocity");
            }
        }

        if(nextStateMode != 0 && !currentState->isState(nextStateMode))
        {
            changeState(nextStateMode);
        }
    }

    int autoTransition()
    {
        if(!auto_mode_)
        {
            return 0;
        }

        const auto elapsed = std::chrono::steady_clock::now() - state_enter_time_;
        const auto state_name = currentState->getStateString();
        if(state_name == "Passive" && elapsed >= auto_passive_wait_)
        {
            return FSMStringMap.right.at("FixStand");
        }
        if(state_name == "FixStand" && elapsed >= auto_fixstand_wait_)
        {
            if(auto_to_velocity_)
            {
                return FSMStringMap.right.at("Velocity");
            }
        }
        return 0;
    }

    void changeState(int nextStateMode)
    {
        for(auto & state : states)
        {
            if(state->isState(nextStateMode))
            {
                input::FixStandLockSource::setActive(state->getStateString() != "Velocity");
                spdlog::info("FSM: Change state from {} to {}", currentState->getStateString(), state->getStateString());
                currentState->exit();
                currentState = state;
                currentState->enter();
                state_enter_time_ = std::chrono::steady_clock::now();
                break;
            }
        }
    }

    std::shared_ptr<BaseState> currentState;
    unitree::common::RecurrentThreadPtr fsm_thread_;
};
