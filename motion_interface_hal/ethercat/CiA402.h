// CiA402.h
#ifndef CIA402_H
#define CIA402_H

#pragma once
#include <cstdint>
#include <string>
#include <map>
#include <ecrt.h>

namespace RDT {
namespace CiA402 {

// --- Идентификаторы производителя и продукта ---
constexpr uint32_t DVS_VENDOR_ID   = 0x00445653;
constexpr uint32_t DVS_PRODUCT_CODE = 0x00009252;

// --- Индексы и субиндексы объектов CiA 402 (Object Dictionary) ---
namespace Objects {
    // System Information
    constexpr uint16_t DeviceType          = 0x1000;
    constexpr uint16_t ErrorRegister       = 0x1001;
    constexpr uint16_t IdentityObject      = 0x1018;

    // PDO Mapping
    constexpr uint16_t RxPdoMap0           = 0x1600;
    constexpr uint16_t TxPdoMap0           = 0x1A00;

    // CiA 402 State Machine
    constexpr uint16_t Controlword         = 0x6040;
    constexpr uint16_t Statusword          = 0x6041;
    constexpr uint16_t ErrorCode           = 0x603F;
    constexpr uint16_t ModesOfOperation    = 0x6060;
    constexpr uint16_t ModesOfOperationDisplay = 0x6061;

    // Motion Commands
    constexpr uint16_t TargetPosition      = 0x607A;
    constexpr uint16_t TargetVelocity      = 0x60FF;
    constexpr uint16_t TargetTorque        = 0x6071;
    constexpr uint16_t ProfileVelocity     = 0x6081;
    constexpr uint16_t ProfileAcceleration = 0x6083;
    constexpr uint16_t ProfileDeceleration = 0x6084;
    
    // Feedback
    constexpr uint16_t ActualPosition      = 0x6064;
    constexpr uint16_t ActualVelocity      = 0x606C;
    constexpr uint16_t ActualTorque        = 0x6077;
    
    // Limits and Protection
    constexpr uint16_t SoftwarePositionLimit = 0x607D;
    constexpr uint16_t FollowingErrorWindow  = 0x6065;
    constexpr uint16_t PositiveTorqueLimit   = 0x60E0;
    constexpr uint16_t NegativeTorqueLimit   = 0x60E1;
    constexpr uint16_t MaxProfileVelocity    = 0x607F; // <-- Наш целевой объект

    // Manufacturer Specific (из документации P100E)
    namespace Vendor {
        constexpr uint16_t UnlockIndex = 0x2000;
        constexpr uint8_t  UnlockSubindex = 1;
        constexpr uint32_t UnlockPassword = 385;
        constexpr uint16_t SaveIndex = 0x2005;
        constexpr uint8_t  SaveSubindex = 3;
    }
}

// --- Биты Controlword (команды, которые мы отправляем) ---
namespace Controlword {
    constexpr uint16_t SwitchOn         = 0x0001;
    constexpr uint16_t EnableVoltage    = 0x0002;
    constexpr uint16_t QuickStop        = 0x0004;
    constexpr uint16_t EnableOperation  = 0x0008;
    constexpr uint16_t FaultReset       = 0x0080;
    
    // Комбинации битов для управления конечным автоматом
    constexpr uint16_t CmdShutdown      = 0x0006;
    constexpr uint16_t CmdSwitchOn      = 0x0007;
    constexpr uint16_t CmdEnable        = 0x000F;
    constexpr uint16_t CmdDisable       = 0x0000;
}

// --- Биты Statusword (состояния, которые мы читаем) ---
namespace Statusword {
    constexpr uint16_t ReadyToSwitchOn      = 0x0001;
    constexpr uint16_t SwitchedOn           = 0x0002;
    constexpr uint16_t OperationEnabled     = 0x0004;
    constexpr uint16_t Fault                = 0x0008;
    constexpr uint16_t VoltageEnabled       = 0x0010;
    constexpr uint16_t QuickStop            = 0x0020;
    constexpr uint16_t SwitchOnDisabled     = 0x0040;
    constexpr uint16_t TargetReached        = 0x0400;
    
    // Маски для точного определения состояний конечного автомата
    constexpr uint16_t MaskNotReady         = 0x004F;
    constexpr uint16_t ValNotReady          = 0x0000;
    constexpr uint16_t MaskSwitchOnDisabled = 0x004F;
    constexpr uint16_t ValSwitchOnDisabled  = 0x0040;
    constexpr uint16_t MaskReadyToSwitchOn  = 0x006F;
    constexpr uint16_t ValReadyToSwitchOn   = 0x0021;
    constexpr uint16_t MaskSwitchedOn       = 0x006F;
    constexpr uint16_t ValSwitchedOn        = 0x0023;
    constexpr uint16_t MaskOperationEnabled = 0x006F;
    constexpr uint16_t ValOperationEnabled  = 0x0027;
    constexpr uint16_t MaskFault            = 0x004F;
    constexpr uint16_t ValFault             = 0x0008;
}

// --- Режимы работы (Modes of Operation) ---
namespace OperationMode {
    constexpr int8_t CyclicSynchronousPosition = 8; // Наш основной режим
}

// --- Конфигурация PDO Mapping ---
namespace Pdo {
    // ИСПРАВЛЕНО: Объекты сгруппированы по направлению (Rx/Tx)
    inline ec_pdo_entry_info_t pdo_entries[] = {
        /* RxPDOs: Master -> Slave */
        {Objects::Controlword, 0x00, 16},
        {Objects::TargetPosition, 0x00, 32},
        {Objects::ModesOfOperation, 0x00, 8},
        {Objects::MaxProfileVelocity, 0x00, 32}, 
        /* TxPDOs: Slave -> Master */
        {Objects::Statusword, 0x00, 16},
        {Objects::ActualPosition, 0x00, 32},
        {Objects::ErrorCode, 0x00, 16},
        {Objects::ActualTorque, 0x00, 16},
    };

    // ИСПРАВЛЕНО: Количество элементов обновлено, смещения корректны
    inline ec_pdo_info_t pdos[] = {
        {Objects::RxPdoMap0, 4, pdo_entries + 0}, /* 4 элемента в RxPDO */
        {Objects::TxPdoMap0, 4, pdo_entries + 4}, /* 4 элемента в TxPDO (начинаются с 4-го индекса) */
    };

    inline ec_sync_info_t syncs[] = {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, pdos + 0, EC_WD_ENABLE},
        {3, EC_DIR_INPUT, 1, pdos + 1, EC_WD_DISABLE},
        {0xFF}
    };
}

// --- Карта кодов ошибок для диагностики ---
inline const std::map<uint16_t, std::string> ErrorCodeMap = {
    {0x1000, "Generic error"},
    {0x2220, "Over current"},
    {0x3210, "Bus overvoltage fault"},
    {0x3220, "Bus undervoltage fault"},
    {0x4210, "Over heat"},
    {0x5443, "Exceed the soft limit (Positive)"},
    {0x5444, "Exceed the soft limit (Negative)"},
    {0x6320, "Parameter storage exception"},
    {0x7305, "Encoder signal error"},
    {0x8400, "Over speed"},
    {0x8611, "Following error too large"},
};

inline std::string getErrorString(uint16_t code) {
    if (code == 0) return "No error";
    auto it = ErrorCodeMap.find(code);
    if (it != ErrorCodeMap.end()) {
        return it->second;
    }
    return "Unknown error code";
}

} // namespace CiA402
} // namespace RDT

#endif // CIA402_H