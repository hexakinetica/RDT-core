#ifndef MOTIONTYPESSERIALIZER_H
#define MOTIONTYPESSERIALIZER_H

#pragma once
#include "MotionTypes.h"
#include "tinyxml2.h"
#include <sstream>
#include <iomanip>

using namespace tinyxml2;

namespace MotionXml {

inline std::string formatDouble(double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << value;
    return oss.str();
}

inline std::string PoseToXml(const Pose& pose) {
    XMLDocument doc;
    auto* root = doc.NewElement("Pose");
    doc.InsertFirstChild(root);

    root->SetAttribute("x", formatDouble(pose.x).c_str());
    root->SetAttribute("y", formatDouble(pose.y).c_str());
    root->SetAttribute("z", formatDouble(pose.z).c_str());
    root->SetAttribute("rx", formatDouble(pose.rx).c_str());
    root->SetAttribute("ry", formatDouble(pose.ry).c_str());
    root->SetAttribute("rz", formatDouble(pose.rz).c_str());

    XMLPrinter printer;
    doc.Print(&printer);
    return std::string(printer.CStr());
}

inline Pose XmlToPose(const std::string& xml) {
    Pose pose;
    XMLDocument doc;
    doc.Parse(xml.c_str());
    auto* root = doc.FirstChildElement("Pose");
    if (root) {
        root->QueryDoubleAttribute("x", &pose.x);
        root->QueryDoubleAttribute("y", &pose.y);
        root->QueryDoubleAttribute("z", &pose.z);
        root->QueryDoubleAttribute("rx", &pose.rx);
        root->QueryDoubleAttribute("ry", &pose.ry);
        root->QueryDoubleAttribute("rz", &pose.rz);
    }
    return pose;
}

inline std::string AxisSetToXml(const AxisSet& axes) {
    XMLDocument doc;
    auto* root = doc.NewElement("AxisSet");
    doc.InsertFirstChild(root);

    for (std::size_t i = 0; i < axes.size(); ++i) {
        const Axis& a = axes.at(i);
        auto* axisElem = doc.NewElement("Axis");
        axisElem->SetAttribute("index", static_cast<int>(i));
        axisElem->SetAttribute("position", formatDouble(a.position).c_str());
        axisElem->SetAttribute("velocity", formatDouble(a.velocity).c_str());
        axisElem->SetAttribute("acceleration", formatDouble(a.acceleration).c_str());
        axisElem->SetAttribute("torque", formatDouble(a.torque).c_str());
        axisElem->SetAttribute("brake", a.brake);
        axisElem->SetAttribute("enabled", a.enabled);
        root->InsertEndChild(axisElem);
    }

    XMLPrinter printer;
    doc.Print(&printer);
    return std::string(printer.CStr());
}

inline AxisSet XmlToAxisSet(const std::string& xml) {
    AxisSet set;
    XMLDocument doc;
    doc.Parse(xml.c_str());
    auto* root = doc.FirstChildElement("AxisSet");
    if (!root) return set;

    int index = 0;
    for (auto* axisElem = root->FirstChildElement("Axis"); axisElem; axisElem = axisElem->NextSiblingElement("Axis")) {
        axisElem->QueryIntAttribute("index", &index);
        if (index < 0 || index >= static_cast<int>(set.size())) continue;
        axisElem->QueryDoubleAttribute("position", &set.at(index).position);
        axisElem->QueryDoubleAttribute("velocity", &set.at(index).velocity);
        axisElem->QueryDoubleAttribute("acceleration", &set.at(index).acceleration);
        axisElem->QueryDoubleAttribute("torque", &set.at(index).torque);
        axisElem->QueryBoolAttribute("brake", &set.at(index).brake);
        axisElem->QueryBoolAttribute("enabled", &set.at(index).enabled);
    }
    return set;
}

} // namespace MotionXml

#endif // MOTIONTYPESSERIALIZER_H
