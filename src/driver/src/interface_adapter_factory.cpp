#include "interface_adapter.hpp"
#include "EG5CD1_interface_adapter.hpp"
#include "RH5DG2_interface_adapter.hpp"
#include "RH56F1_interface_adapter.hpp"

std::unique_ptr<InterfaceAdapter> makeInterfaceAdapter(
    const std::string& interfaces_profile,
    IRegisterIoBackend& backend,
    const DeviceNodeConfig& config,
    RosEntityMaps maps)
{
    if (interfaces_profile == "RH56F1") {
        return std::make_unique<RH56F1InterfaceAdapter>(backend, config, maps);
    }
    if (interfaces_profile == "EG5CD1") {
        return std::make_unique<EG5CD1InterfaceAdapter>(backend, config, maps);
    }
    return std::make_unique<RH5DG2InterfaceAdapter>(backend, config, maps);
}
