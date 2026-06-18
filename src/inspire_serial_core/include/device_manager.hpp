#pragma once
#include <string>
#include <memory>
#include <unordered_map>

class SerialPortBase;
class Protocol;

class DeviceManager {
public:
    void addDevice(const std::string& port, std::shared_ptr<Protocol> protocol, int baudRate);
    std::shared_ptr<SerialPortBase> getDevice(const std::string& port);
    void removeDevice(const std::string& port);

private:
    std::unordered_map<std::string, std::shared_ptr<SerialPortBase>> devices;
};

