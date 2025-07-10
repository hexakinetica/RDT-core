// RDT_Qt_MetaTypes.h
#ifndef RDT_QT_METATYPES_H
#define RDT_QT_METATYPES_H

#pragma once
#include <QMetaType> //  Q_DECLARE_METATYPE  qRegisterMetaType

//    RDT ,     /
#include "DataTypes.h" //  RDT::CartPose, AxisSet, Axis, ToolFrame, BaseFrame, RobotMode, TrajectoryPoint
#include "Units.h"     //  RDT::Radians, Meters, Seconds, etc.
//  ControllerState  RobotController.h,    
// #include "RobotController.h" //  ControllerState 

// ===  Q_DECLARE_METATYPE ===
//      ( RDT)

Q_DECLARE_METATYPE(RDT::CartPose)
Q_DECLARE_METATYPE(RDT::Axis)
Q_DECLARE_METATYPE(RDT::AxisSet)
Q_DECLARE_METATYPE(RDT::ToolFrame)
Q_DECLARE_METATYPE(RDT::BaseFrame)
Q_DECLARE_METATYPE(RDT::RobotMode)
// Q_DECLARE_METATYPE(RDT::ControllerState) //  ControllerState   
Q_DECLARE_METATYPE(RDT::TrajectoryPoint)

Q_DECLARE_METATYPE(RDT::Radians)
Q_DECLARE_METATYPE(RDT::Degrees)
Q_DECLARE_METATYPE(RDT::Meters)
Q_DECLARE_METATYPE(RDT::Millimeters)
Q_DECLARE_METATYPE(RDT::Seconds)

Q_DECLARE_METATYPE(RDT::RadiansPerSecond)
Q_DECLARE_METATYPE(RDT::DegreesPerSecond)
Q_DECLARE_METATYPE(RDT::MetersPerSecond)
Q_DECLARE_METATYPE(RDT::MillimetersPerSecond)

Q_DECLARE_METATYPE(RDT::RadiansPerSecondSq)
Q_DECLARE_METATYPE(RDT::DegreesPerSecondSq) //  
Q_DECLARE_METATYPE(RDT::MetersPerSecondSq)
Q_DECLARE_METATYPE(RDT::MillimetersPerSecondSq) //  

//      
//     main.cpp
namespace RDT {
inline void registerRdtMetaTypes() {
    qRegisterMetaType<RDT::CartPose>("RDT::CartPose");
    qRegisterMetaType<RDT::Axis>("RDT::Axis"); // ,  AxisSet     Qt  
    qRegisterMetaType<RDT::AxisSet>("RDT::AxisSet");
    qRegisterMetaType<RDT::ToolFrame>("RDT::ToolFrame");
    qRegisterMetaType<RDT::BaseFrame>("RDT::BaseFrame");
    qRegisterMetaType<RDT::RobotMode>("RDT::RobotMode");
    // qRegisterMetaType<RDT::ControllerState>("RDT::ControllerState");
    qRegisterMetaType<RDT::TrajectoryPoint>("RDT::TrajectoryPoint");

    qRegisterMetaType<RDT::Radians>("RDT::Radians");
    qRegisterMetaType<RDT::Degrees>("RDT::Degrees");
    qRegisterMetaType<RDT::Meters>("RDT::Meters");
    qRegisterMetaType<RDT::Millimeters>("RDT::Millimeters");
    qRegisterMetaType<RDT::Seconds>("RDT::Seconds");

    qRegisterMetaType<RDT::RadiansPerSecond>("RDT::RadiansPerSecond");
    qRegisterMetaType<RDT::DegreesPerSecond>("RDT::DegreesPerSecond");
    qRegisterMetaType<RDT::MetersPerSecond>("RDT::MetersPerSecond");
    qRegisterMetaType<RDT::MillimetersPerSecond>("RDT::MillimetersPerSecond");
    
    qRegisterMetaType<RDT::RadiansPerSecondSq>("RDT::RadiansPerSecondSq");
    // qRegisterMetaType<RDT::DegreesPerSecondSq>("RDT::DegreesPerSecondSq");
    qRegisterMetaType<RDT::MetersPerSecondSq>("RDT::MetersPerSecondSq");
    // qRegisterMetaType<RDT::MillimetersPerSecondSq>("RDT::MillimetersPerSecondSq");
    
    // qDebug() << "RDT MetaTypes registered with Qt."; //  
}
} // namespace RDT

#endif // RDT_QT_METATYPES_H