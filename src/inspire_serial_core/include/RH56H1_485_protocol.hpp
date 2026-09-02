#pragma once

#include "RH56F1_485_protocol.hpp"

/**
 * @brief RH56H1 485 协议：寄存器映射与帧格式与 RH56F1_485 完全一致。
 *
 * 触觉（touchAct）传感器类型与 F1 不同，后续在子类中覆盖 parseTouchData / readTouchData。
 */
class RH56H1_485_Protocol : public RH56F1_485_Protocol {};
