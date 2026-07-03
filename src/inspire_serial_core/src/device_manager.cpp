#include "device_manager.hpp"
#include "serial_port.hpp"

void DeviceManager::addDevice(const std::string& port, std::shared_ptr<Protocol> protocol, int baudRate) {
    auto serial_port = std::make_shared<SerialPortBase>(port, baudRate);
    serial_port->setProtocol(protocol);
    devices[port] = serial_port;
}

std::shared_ptr<SerialPortBase> DeviceManager::getDevice(const std::string& port) {
    auto it = devices.find(port);
    return (it != devices.end()) ? it->second : nullptr;
}

void DeviceManager::removeDevice(const std::string& port) { devices.erase(port); }
