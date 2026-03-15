#ifndef SSD_BINDING_HPP
#define SSD_BINDING_HPP

#include <string>

std::string execCommand(const std::string& command);
std::string getExecPath();
std::string getSSDDevice();
std::string getSSDSerialNumber(const std::string& device);
bool verifyHardwareBinding(const std::string& expected_serial);

#endif // SSD_BINDING_HPP