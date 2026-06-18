#pragma once

#include <string>
#include <functional>
#include <unordered_map>
#include <vector>
#include <memory>

// 前向声明
class Protocol;

/**
 * @brief 协议工厂类
 * 
 * 使用工厂模式和注册机制，支持动态注册和创建协议对象。
 * 新协议只需注册，无需修改ConfigLoader等代码。
 * 
 * 使用示例：
 *   // 在协议实现文件中注册
 *   REGISTER_PROTOCOL("RH56F1_485", RH56F1_485_Protocol);
 *   
 *   // 使用时
 *   auto protocol = ProtocolFactory::create("RH56F1_485");
 */
class ProtocolFactory {
public:
    // 协议创建函数类型
    using ProtocolCreator = std::function<std::shared_ptr<Protocol>()>;
    
    /**
     * @brief 注册协议类型
     * @param type 协议类型名称（如 "RH56F1_485"）
     * @param creator 协议创建函数
     */
    static void registerProtocol(const std::string& type, ProtocolCreator creator);
    
    /**
     * @brief 创建协议对象
     * @param type 协议类型名称
     * @return 协议对象指针
     * @throws std::runtime_error 如果协议类型未注册
     */
    static std::shared_ptr<Protocol> create(const std::string& type);
    
    /**
     * @brief 获取已注册的协议类型列表
     * @return 协议类型名称列表
     */
    static std::vector<std::string> getRegisteredTypes();
    
    /**
     * @brief 检查协议类型是否已注册
     * @param type 协议类型名称
     * @return true表示已注册，false表示未注册
     */
    static bool isRegistered(const std::string& type);

private:
    // 获取注册表（单例模式）
    static std::unordered_map<std::string, ProtocolCreator>& getRegistry();
};

/**
 * @brief 自动注册协议宏
 * 
 * 在协议实现类的cpp文件中使用此宏，程序启动时会自动注册协议。
 * 
 * @param type 协议类型名称字符串
 * @param class_name 协议类名
 * 
 * 示例：
 *   REGISTER_PROTOCOL("RH56F1_485", RH56F1_485_Protocol);
 */
#define REGISTER_PROTOCOL(type, class_name) \
    static struct class_name##_Registrar { \
        class_name##_Registrar() { \
            ProtocolFactory::registerProtocol(type, []() { \
                return std::make_shared<class_name>(); \
            }); \
        } \
    } class_name##_registrar;

