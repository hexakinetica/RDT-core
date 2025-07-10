// Panel_StateDisplay.cpp
#include "Panel_StateDisplay.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout> // Added for frames_layout
#include <QFont>
#include <QString>
#include <cmath>     // For M_PI (though RDT::UnitConstants::PI is preferred if available)
#include <iomanip>   // For std::setprecision in RDT::Units::toString

// Use RDT::UnitConstants::PI if available and M_PI is not standard
#ifndef M_PI
    #define M_PI RDT::UnitConstants::PI
#endif

namespace RDT {

// For convenience with literals if used within this file
using namespace RDT::literals;

Panel_StateDisplay::Panel_StateDisplay(QWidget* parent)
    : QWidget(parent) {
    // cached_... members are default-constructed with RDT types (e.g., 0.0_m, 0.0_rad)
    setupUI();
    // Initial display update with default (zero) values
    updateTCPDisplay();
    updateJointDisplay();
    LOG_DEBUG(MODULE_NAME, "Panel_StateDisplay created and UI set up.");
}

void Panel_StateDisplay::setupUI() {

    auto* main_layout = new QVBoxLayout(this);
    main_layout->setSpacing(4); //    
    main_layout->setContentsMargins(5, 3, 5, 3); //   

    QFont title_font("Arial", 9, QFont::Bold); //      
    QFont label_font("Arial", 8);              //    X, Y, A1  ..
    QFont value_font("Consolas", 9, QFont::Bold); //   
    if (value_font.family().compare("Consolas", Qt::CaseInsensitive) != 0) {
        value_font.setFamily("Monospace");
    }

    // --- Frames Group (Tool + Base   ) ---
    auto* frames_group = new QGroupBox("Active Frames", this);
    frames_group->setFont(title_font);
    auto* frames_line_layout = new QHBoxLayout; //  layout   
    frames_line_layout->setSpacing(5);      //    

    frames_line_layout->addWidget(new QLabel("Tool:", this));
    tool_name_value_label_ = new QLabel("N/A", this);
    tool_name_value_label_->setFont(value_font);
    tool_name_value_label_->setMinimumWidth(80); // 
    frames_line_layout->addWidget(tool_name_value_label_);

    frames_line_layout->addSpacing(10); //   Tool  Base

    frames_line_layout->addWidget(new QLabel("Base:", this));
    base_name_value_label_ = new QLabel("N/A", this);
    base_name_value_label_->setFont(value_font);
    base_name_value_label_->setMinimumWidth(80); // 
    frames_line_layout->addWidget(base_name_value_label_);

    frames_line_layout->addStretch(1); //  
    frames_group->setLayout(frames_line_layout);
    main_layout->addWidget(frames_group);

    // --- TCP Coordinates Group (  ) ---
    auto* tcp_group = new QGroupBox("TCP Pose (Active Base: mm, deg)", this); //   
    tcp_group->setFont(title_font);
    auto* tcp_line_layout = new QHBoxLayout;
    tcp_line_layout->setSpacing(3); //   
    const QStringList tcp_names = {"X", "Y", "Z", "Rx", "Ry", "Rz"};

    for (int i = 0; i < 6; ++i) {
        //  ,     ,  ":"
        // tcp_coord_labels_[i] = new QLabel(tcp_names[i] + ":", this);
        // tcp_coord_labels_[i]->setFont(label_font);
        // tcp_line_layout->addWidget(tcp_coord_labels_[i]);

        //  
        tcp_coord_values_[i] = new QLabel("0.0", this);
        tcp_coord_values_[i]->setFont(value_font);
        tcp_coord_values_[i]->setMinimumWidth(55); //  
        tcp_coord_values_[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        tcp_coord_values_[i]->setToolTip(tcp_names[i]); //    
        tcp_coord_values_[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
        tcp_line_layout->addWidget(tcp_coord_values_[i]);
        
        //        QLabel   ,
        //     .
        // tcp_units_labels_[i] = new QLabel(...)

        if (i < 5) {
            tcp_line_layout->addSpacing(5); //    
        }
    }
    tcp_line_layout->addStretch(1);
    tcp_group->setLayout(tcp_line_layout);
    main_layout->addWidget(tcp_group);

    // --- Joint Coordinates Group (  ) ---
    auto* joints_group = new QGroupBox("Joint Angles (deg)", this); //   
    joints_group->setFont(title_font);
    auto* joints_line_layout = new QHBoxLayout;
    joints_line_layout->setSpacing(3);
    const QStringList joint_axis_names = {"A1", "A2", "A3", "A4", "A5", "A6"};

    for (int i = 0; i < 6; ++i) {
        // joint_angle_labels_[i] = new QLabel(joint_axis_names[i] + ":", this);
        // joint_angle_labels_[i]->setFont(label_font);
        // joints_line_layout->addWidget(joint_angle_labels_[i]);

        joint_angle_values_[i] = new QLabel("0.00", this);
        joint_angle_values_[i]->setFont(value_font);
        joint_angle_values_[i]->setMinimumWidth(50); // 
        joint_angle_values_[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        joint_angle_values_[i]->setToolTip(joint_axis_names[i]);
        joint_angle_values_[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
        joints_line_layout->addWidget(joint_angle_values_[i]);

        // joint_units_labels_[i] = new QLabel(...)

        if (i < 5) {
            joints_line_layout->addSpacing(5);
        }
    }
    joints_line_layout->addStretch(1);
    joints_group->setLayout(joints_line_layout);
    main_layout->addWidget(joints_group);

    // main_layout->addStretch(1); //  ,      
    setLayout(main_layout);
}

// --- Public Slots ---
void Panel_StateDisplay::onCurrentPoseUpdate(const RDT::CartPose& tcp_in_current_base, const RDT::AxisSet& joints) {
    LOG_DEBUG_F(MODULE_NAME, "Slot onCurrentPoseUpdate: TCP X %s, J1 %s",
                tcp_in_current_base.x.toString().c_str(), joints[AxisId::A1].angle.toDegrees().toString().c_str());
    cached_tcp_display_ = tcp_in_current_base;
    cached_joints_display_ = joints;
    updateTCPDisplay();
    updateJointDisplay();
}

void Panel_StateDisplay::onActiveToolUpdate(const RDT::ToolFrame& tool) {
    LOG_DEBUG_F(MODULE_NAME, "Slot onActiveToolUpdate: %s", tool.name.c_str());
    cached_tool_display_ = tool;
    // Tool name is updated as part of updateTCPDisplay if it reads from cached_tool_display_
    // Or update label directly:
    if (tool_name_value_label_) {
        tool_name_value_label_->setText(QString::fromStdString(cached_tool_display_.name));
    }
    // updateTCPDisplay(); // TCP values might change if base was relative to tool, but unlikely for this panel
}

void Panel_StateDisplay::onActiveBaseUpdate(const RDT::BaseFrame& base) {
    LOG_DEBUG_F(MODULE_NAME, "Slot onActiveBaseUpdate: %s", base.name.c_str());
    cached_base_display_ = base;
    if (base_name_value_label_) {
        base_name_value_label_->setText(QString::fromStdString(cached_base_display_.name));
    }
    // TCP values are relative to this base, so if the base changes, the displayed TCP numbers
    // for the *same world point* would change. However, this slot just updates the *name*
    // of the base frame in which the *already provided* cached_tcp_display_ is expressed.
    // The adapter is responsible for sending a new cached_tcp_display_ if its numerical values change.
}

void Panel_StateDisplay::setActiveFrameNames(const QString& toolName, const QString& baseName) {
    LOG_DEBUG_F(MODULE_NAME, "Slot setActiveFrameNames: Tool='%s', Base='%s'", toolName.toStdString().c_str(), baseName.toStdString().c_str());
    // This is a simpler update if full frame objects are not needed or come later.
    if(tool_name_value_label_) tool_name_value_label_->setText(toolName);
    if(base_name_value_label_) base_name_value_label_->setText(baseName);
    // Update internal cache if these names correspond to known frames.
    // For now, just display.
    cached_tool_display_.name = toolName.toStdString(); // Update cache
    cached_base_display_.name = baseName.toStdString(); // Update cache
}


// --- Private Update Methods ---
void Panel_StateDisplay::updateTCPDisplay() {
    if (!tool_name_value_label_) return; // UI not ready

    tool_name_value_label_->setText(QString::fromStdString(cached_tool_display_.name));
    base_name_value_label_->setText(QString::fromStdString(cached_base_display_.name));

    // cached_tcp_display_ contains RDT::Meters and RDT::Radians
    // Display X, Y, Z in millimeters with 1 decimal place
    tcp_coord_values_[0]->setText(QString::number(cached_tcp_display_.x.toMillimeters().value(), 'f', 1));
    tcp_coord_values_[1]->setText(QString::number(cached_tcp_display_.y.toMillimeters().value(), 'f', 1));
    tcp_coord_values_[2]->setText(QString::number(cached_tcp_display_.z.toMillimeters().value(), 'f', 1));
    // Display Rx, Ry, Rz in degrees with 2 decimal places
    tcp_coord_values_[3]->setText(QString::number(cached_tcp_display_.rx.toDegrees().value(), 'f', 2));
    tcp_coord_values_[4]->setText(QString::number(cached_tcp_display_.ry.toDegrees().value(), 'f', 2));
    tcp_coord_values_[5]->setText(QString::number(cached_tcp_display_.rz.toDegrees().value(), 'f', 2));
}

void Panel_StateDisplay::updateJointDisplay() {
    if (!joint_angle_values_[0]) return; // UI not ready

    // cached_joints_display_ contains RDT::Axis, where Axis.angle is RDT::Radians
    // Display joint angles in degrees with 2 decimal places
    for(int i = 0; i < 6; ++i) {
        if (i < static_cast<int>(cached_joints_display_.size())) { // Check bounds
            joint_angle_values_[i]->setText(QString::number(
                cached_joints_display_.at(i).angle.toDegrees().value(), 'f', 2)
            );
        } else {
            joint_angle_values_[i]->setText("N/A"); // Should not happen with AxisSet
        }
    }
}

} // namespace RDT