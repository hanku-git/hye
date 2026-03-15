#include "ssd_binding.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <array>
#include <memory>
#include <stdexcept>
#include <unistd.h>
#include <limits.h>

std::string execCommand(const std::string& command) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

std::string getExecPath() {
    char path[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", path, PATH_MAX);
    if (count == -1) {
        throw std::runtime_error("Failed to read /proc/self/exe");
    }
    return std::string(path, count);
}

std::string getSSDDevice() {
    std::string exec_path = getExecPath();
    //std::cout << "Exec path: " << exec_path << std::endl;
    std::string command = "findmnt -n -o SOURCE --target \"" + exec_path + "\"";
    std::string device = execCommand(command);
    //std::cout << "device ::" << device << std::endl;
    if (!device.empty() && device.back() == '\n') {
        device.pop_back(); // Remove newline
    }
    return device;
}

std::string getSSDSerialNumber(const std::string& device) {
    std::string command = "udevadm info --query=all --name=" + device + " | grep ID_SERIAL_SHORT | awk -F= '{print $2}'";
    std::string serial_number = execCommand(command);
    //std::cout << "serial num ::"<< serial_number << std::endl;
    if (!serial_number.empty() && serial_number.back() == '\n') {
        serial_number.pop_back(); // Remove newline
    }
    return serial_number;
}

bool verifyHardwareBinding(const std::string& expected_serial) {
    try {
        std::string device = getSSDDevice();
        if (device.empty()) {
            //std::cerr << "No NVMe device found." << std::endl;
            return false;
        }
        //std::cout << "Found SSD device: " << device << std::endl;

        std::string actual_serial = getSSDSerialNumber(device);
        if (actual_serial.empty()) {
            //std::cerr << "No serial number found for device " << device << std::endl;
            return false;
        }
        //std::cout << "Current SSD serial number: " << actual_serial << std::endl;
        //std::cout << "Expected SSD serial number: " << expected_serial << std::endl;
        return actual_serial == expected_serial;
    } catch (const std::exception& ex) {
        //std::cerr << "Error: " << ex.what() << std::endl;
        return false;
    }
}