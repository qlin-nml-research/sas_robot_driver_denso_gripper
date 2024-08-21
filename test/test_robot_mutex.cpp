#include "modules/robot_mutex.h"


#include <exception>


/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<csignal>
#include <atomic>
#include <iostream>
#include <thread>

static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

void simple_loop(const std::string &loop_name, const bool &master=false)
{
    sas::RobotMutex robot_mutex("arm1", loop_name, master);
    std::cout << "Starting ["<<loop_name<<"] loop." << std::endl;
    try{
        while(!kill_this_process)
        {
            std::cout << "["<<loop_name<<"]trying..." << std::endl;
            auto ret = robot_mutex.acquire(2000);
            if(!ret) {
                std::cerr << "["<<loop_name<<"]Failed to acquire mutex." << std::endl;
            }else {
                std::cout << "["<<loop_name<<"]Acquired mutex." << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if(ret)
            {
                robot_mutex.release();
            }
            if(robot_mutex.has_locked()) {
                throw std::runtime_error("Mutex still locked.");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        }
    }catch(std::exception &e)
    {
        std::cerr << "["<<loop_name<<"]Exception: "<<e.what()<<std::endl;
    }
    std::cout << "Exiting ["<<loop_name<<"] loop." << std::endl;
}

void max_time_loop(const std::string &loop_name)
{
    auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    sas::RobotMutex robot_mutex("arm1", loop_name);
    std::cout << "Starting ["<<loop_name<<"] loop." << std::endl;
    try{
        while(!kill_this_process)
        {
            std::cout << "["<<loop_name<<"]trying..." << std::endl;
            if (std::chrono::steady_clock::now() > timeout) {
                std::cout << "["<<loop_name<<"]Timeout." << std::endl;
                break;
            }
            auto ret = robot_mutex.acquire(2000);
            if(!ret) {
                std::cerr << "["<<loop_name<<"]Failed to acquire mutex." << std::endl;
            }else {
                std::cout << "["<<loop_name<<"]Acquired mutex." << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if(ret)
            {
                robot_mutex.release();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        }
    }catch(std::exception &e)
    {
        std::cerr << "["<<loop_name<<"]Exception: "<<e.what()<<std::endl;
    }
    std::cout << "Exiting ["<<loop_name<<"] loop." << std::endl;
}


int main(int argc, char** argv)
{
    bool init_as_master = false;
    if(argc>1) {
        if(std::string(argv[1]) == "master") {
            init_as_master = true;
            std::cout << "Init as master." << std::endl;
        }
    }
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("Error setting the signal int handler.");
    }
    if(signal(SIGTERM, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("Error setting the signal term handler.");
    }

    std::thread t1(simple_loop,"LOOP1", init_as_master);
    // t1.detach();
    // std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // std::thread t2(simple_loop,"LOOP2");
    // t2.detach();
    // std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // std::thread t3(max_time_loop,"LOOP3");
    // t3.detach();

    std::cout << "STARTING MAIN PROGRAM." << std::endl;
    while(!kill_this_process)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Exiting MAIN PROGRAM." << std::endl;

    try {
        if(t1.joinable()) t1.join();
        // if(t2.joinable()) t2.join();
        // if(t3.joinable()) t3.join();
    }catch (...) {
        std::cerr<<"Error joining threads."<<std::endl;
    }
    return 0;

}
