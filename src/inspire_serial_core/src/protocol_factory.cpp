#include "protocol_factory.hpp"
#include "protocol.hpp"
#include <stdexcept>

void ProtocolFactory::registerProtocol(const std::string& type, ProtocolCreator creator) {
    getRegistry()[type] = creator;
}

std::shared_ptr<Protocol> ProtocolFactory::create(const std::string& type) {
    auto& registry = getRegistry();
    auto it = registry.find(type);
    if (it != registry.end()) {
        return it->second();  // 调用创建函数
    }
    throw std::runtime_error("Unknown protocol type: " + type + ". Available types: " + 
                            [&registry]() {
                                std::string result;
                                for (const auto& [key, value] : registry) {
                                    if (!result.empty()) result += ", ";
                                    result += key;
                                }
                                return result.empty() ? "(none)" : result;
                            }());
}

std::vector<std::string> ProtocolFactory::getRegisteredTypes() {
    std::vector<std::string> types;
    for (const auto& [type, creator] : getRegistry()) {
        types.push_back(type);
    }
    return types;
}

bool ProtocolFactory::isRegistered(const std::string& type) {
    auto& registry = getRegistry();
    return registry.find(type) != registry.end();
}

std::unordered_map<std::string, ProtocolFactory::ProtocolCreator>& ProtocolFactory::getRegistry() {
    static std::unordered_map<std::string, ProtocolCreator> registry;
    return registry;
}

