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

#include "robot_mutex.h"

namespace sas {
    void RobotMutex::_check_erroron(const std::string &f_name) const {

        switch(errno) {
            case EACCES :
                THROW_RUNTIME_ERROR_FN(f_name, "EACCES: permission denied")
            case EINVAL  :
                THROW_RUNTIME_ERROR_FN(f_name, "EINVAL: value was greater than SEM_VALUE_MAX OR name consists of just /")
            case EMFILE  :
                THROW_RUNTIME_ERROR_FN(f_name, "EMFILE: process has too many files open")
            case ENAMETOOLONG :
                THROW_RUNTIME_ERROR_FN(f_name, "ENAMETOOLONG: name was too long")
            case ENFILE :
                THROW_RUNTIME_ERROR_FN(f_name, "ENFILE: system limit on the total number of open files has been reached")
            case ENOMEM :
                THROW_RUNTIME_ERROR_FN(f_name, "ENOMEM: not enough memory")
            case ENOENT :
                THROW_RUNTIME_ERROR_FN(f_name,"ENOENT: name does not exist")
            case ETIMEDOUT :
                THROW_RUNTIME_ERROR_FN(f_name,"ETIMEDOUT: timeout")
            case EDEADLK :
                THROW_RUNTIME_ERROR_FN(f_name, "EDEADLK: deadlock detected")
            case EINTR :
                THROW_RUNTIME_ERROR_FN(f_name, "EINTR: signal caught")
            default:
                THROW_RUNTIME_ERROR_FN(f_name, "Reason unknown")

        };
    }

    RobotMutex::RobotMutex(const std::string &lock_name_space, const std::string &instance_name, const bool &is_master, const bool &debug):
            has_locked_(false),
            lock_name_space_(lock_name_space),
            instance_name_(instance_name),
            is_master_(is_master),  // if true, the semaphore will be deleted in the destructor
            debug_(debug)
    {
        /**
         * TODO: better checking of only one is_master instance
         * if the semaphore is not deleted in the previous run and is currently 0, there might be a deadlock with no one able to acquire it.
         * the is_master instance in this case will manage the deletion of the semaphore.
         */
        // if lock_name_space_ has leading / remove it
        if(lock_name_space_.front() == '/') {
            lock_name_space_ = lock_name_space_.substr(1);
        }
        if ((shared_mutex_ = sem_open((ROBOT_MUTEX_PREFIX+lock_name_space_).c_str(), O_CREAT, ROBOT_MUTEX_PERMISSIONS, 1)) == SEM_FAILED) {
            _check_erroron("RobotMutex");
        }else {
            std::cout << "RobotMutex:["<<lock_name_space_<<"/"<<instance_name_<<"]:created." << std::endl;
        }

    }
    RobotMutex::~RobotMutex(){
        if(has_locked_) {
            release();
        }
        if(sem_close(shared_mutex_) == -1) {
            _check_erroron("~RobotMutex");
        }
        if(is_master_) {
            if(sem_unlink((ROBOT_MUTEX_PREFIX+lock_name_space_).c_str()) == -1) {
                _check_erroron("~RobotMutex");
            }
        }
        std::cout << "RobotMutex:["<<lock_name_space_<<"/"<<instance_name_<<"]:released." << std::endl;
    }

    bool RobotMutex::acquire(const unsigned int &timeout_ms) {
        if(has_locked_) {
            THROW_RUNTIME_ERROR_FN("acquire", "already locked")
        }
        unsigned int ms_offset = timeout_ms==0?ROBOT_MUTEX_MAX_TIMEOUT_MS:timeout_ms;
        auto time_now = std::chrono::steady_clock::now();
        auto timeout_ts = std::chrono::time_point_cast<std::chrono::milliseconds>(time_now + std::chrono::milliseconds(ms_offset));
        while(std::chrono::steady_clock::now() < timeout_ts) {
            if (sem_trywait (shared_mutex_) == 0) {
                has_locked_ = true;
                if(debug_){std::cout << "RobotMutex:[" << lock_name_space_ << "/" << instance_name_ << "]::acquire::locked" << std::endl;}
                return true;
            }
            if(errno == EAGAIN) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }else {
                switch (errno) {
                    case EDEADLK :
                        THROW_RUNTIME_ERROR_FN("acquire", "deadlock detected");
                    case EINTR :
                        THROW_RUNTIME_ERROR_FN("acquire", "signal caught");
                    default:
                        THROW_RUNTIME_ERROR_FN("acquire", "reason unknown");
                }
            }
        }
        return false;
    }

    void RobotMutex::release(){
        if(!has_locked_) {
            if(debug_){THROW_RUNTIME_ERROR_FN("release", "not locked")}
            else{return;}
        }
        if(sem_post(shared_mutex_) == -1) {
            switch(errno) {
                case EINVAL :
                    THROW_RUNTIME_ERROR_FN("release", "EINVAL:invalid semaphore");
                default:
                    THROW_RUNTIME_ERROR_FN("release", "reason unknown");
            }
        }else{
            has_locked_ = false;
        }

    }

    bool RobotMutex::has_locked() const {
        return has_locked_;
    }

    bool RobotMutex::is_locked() const {
        int sval;
        if(sem_getvalue(shared_mutex_, &sval) == 0){
            return sval == 0;
        }
        switch(errno) {
            case EINVAL :
                THROW_RUNTIME_ERROR_FN("is_locked", "EINVAL:invalid semaphore");
            default:
                THROW_RUNTIME_ERROR_FN("is_locked", "reason unknown");
        }
    }

}
