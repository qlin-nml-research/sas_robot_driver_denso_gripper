#pragma once
/*
# Copyright (c) 2024 Hung-Ching Lin
#
#    This file is part of sas_robot_driver_dense.
#
#    sas_robot_driver_dense is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_dense is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_dense.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Hung-Ching Lin, email: qlin1806@g.ecc.u-tokyo.ac.jp
#
#   - Added to sas_robot_driver_dense for
#        gripper control and some other functions for cobotta robot
# ################################################################*/
#include <semaphore.h>
#include <string>
#include <fcntl.h>
#include <atomic>
#include <stdexcept>
#include <cerrno>
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>

#define ROBOT_MUTEX_PREFIX "/sas_denso_"
#define ROBOT_MUTEX_MAX_TIMEOUT_MS 1000
#define ROBOT_MUTEX_PERMISSIONS 0777

#define THROW_RUNTIME_ERROR(msg) throw std::runtime_error("["+lock_name_space_+"/"+instance_name_+"]"+msg);
#define THROW_RUNTIME_ERROR_FN(fn_name, msg) throw std::runtime_error("["+lock_name_space_+"/"+instance_name_+"]::"+fn_name+":"+msg);

namespace sas {
    class RobotMutex {
    private:
        sem_t *shared_mutex_;
        std::atomic_bool has_locked_;
        std::string lock_name_space_;
        std::string instance_name_;
        const bool master_;

    public:
        RobotMutex()=delete;
        RobotMutex(const RobotMutex&)=delete;
        RobotMutex(const std::string &lock_name_space, const std::string &instance_name, const bool &master=false);
        ~RobotMutex();

        bool acquire(const unsigned int &timeout_ms=0);

        void release();

        bool has_locked() const;

        bool is_locked() const;

    protected:
        void _check_erroron(const std::string &f_name) const;

    };
}



