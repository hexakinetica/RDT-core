// Panel_JogControl.cpp
#include "Panel_JogControl.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout> // For a more structured axes layout
#include <QGroupBox>
#include <QDebug>      // For qDebug, can be replaced by RDT::Logger

namespace RDT {

// For convenience
using namespace RDT::literals;

Panel_JogControl::Panel_JogControl(QWidget* parent)
    : QWidget(parent) {
    setupUI();
    updateButtonAndAxisLabels(); // Initial setup of labels
    LOG_DEBUG(MODULE_NAME, "Panel_JogControl created and UI set up.");
}

void Panel_JogControl::setupUI() {
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setSpacing(10);
    main_layout->setContentsMargins(8,8,8,8);

    QFont groupFont("Arial", 10, QFont::Bold);

    // --- Jog Mode Group ---
    auto* mode_group = new QGroupBox("Jogging Mode", this);
    mode_group->setFont(groupFont);
    auto* mode_layout = new QVBoxLayout;
    mode_toggle_button_ = new QPushButton(this); 
    connect(mode_toggle_button_, &QPushButton::clicked, this, &Panel_JogControl::onInternalModeToggle);
    mode_layout->addWidget(mode_toggle_button_);
    mode_group->setLayout(mode_layout);
    main_layout->addWidget(mode_group);

    // --- Coordinate Frame Group (for Cartesian) ---
    auto* frame_group = new QGroupBox("Cartesian Jog Frame", this);
    frame_group->setFont(groupFont);
    auto* frame_layout = new QVBoxLayout;
    jog_frame_combo_ = new QComboBox(this);
    jog_frame_combo_->addItems({"Robot Base", "Active Tool", "Active User Base"}); 
    jog_frame_combo_->setToolTip("Select the coordinate system for Cartesian jogging.");
    frame_layout->addWidget(jog_frame_combo_);
    frame_group->setLayout(frame_layout);
    main_layout->addWidget(frame_group);

    // --- Speed Control Group ---
    auto* speed_group = new QGroupBox("Jog Speed", this);
    speed_group->setFont(groupFont);
    auto* speed_layout = new QHBoxLayout; 
    jog_speed_slider_ = new QSlider(Qt::Horizontal, this);
    jog_speed_slider_->setRange(1, 100); 
    jog_speed_slider_->setValue(25);    
    jog_speed_slider_->setTickInterval(10);
    jog_speed_slider_->setTickPosition(QSlider::TicksBelow);
    speed_display_label_ = new QLabel(this); 
    speed_display_label_->setMinimumWidth(80); 
    speed_display_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    connect(jog_speed_slider_, &QSlider::valueChanged, this, &Panel_JogControl::onInternalSpeedSliderChanged);
    onInternalSpeedSliderChanged(jog_speed_slider_->value()); 
    speed_layout->addWidget(speed_display_label_);
    speed_layout->addWidget(jog_speed_slider_);
    speed_group->setLayout(speed_layout);
    main_layout->addWidget(speed_group);

    // --- Step Mode Group ---
    auto* step_group = new QGroupBox("Step Mode & Size", this);
    step_group->setFont(groupFont);
    auto* step_layout = new QGridLayout;
    step_mode_toggle_button_ = new QPushButton(this); 
    connect(step_mode_toggle_button_, &QPushButton::clicked, this, &Panel_JogControl::onInternalStepModeToggle);
    step_layout->addWidget(step_mode_toggle_button_, 0, 0, 1, 2); 

    step_layout->addWidget(new QLabel("Step Size:", this), 1, 0);
    step_size_spin_ = new QDoubleSpinBox(this);
    step_size_spin_->setRange(0.01, 1000.0);
    step_size_spin_->setDecimals(2);
    step_size_spin_->setValue(1.0); 
    step_layout->addWidget(step_size_spin_, 1, 1);
    step_group->setLayout(step_layout);
    main_layout->addWidget(step_group);
    onInternalStepModeToggle(); 

    // --- Axes/Coordinates Control Group ---
    auto* axes_control_group = new QGroupBox("Axes / Coordinates", this);
    axes_control_group->setFont(groupFont);
    auto* axes_grid_layout = new QGridLayout; 
    for (unsigned int i = 0; i < ROBOT_AXES_COUNT; ++i) {
        axis_display_labels_[i] = new QLabel(this);
        axis_display_labels_[i]->setMinimumWidth(40);
        axis_display_labels_[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        negative_jog_buttons_[i] = new QPushButton("-");
        positive_jog_buttons_[i] = new QPushButton("+");
        
        negative_jog_buttons_[i]->setAutoRepeat(true); 
        positive_jog_buttons_[i]->setAutoRepeat(true);
        negative_jog_buttons_[i]->setAutoRepeatDelay(300);
        positive_jog_buttons_[i]->setAutoRepeatDelay(300);
        negative_jog_buttons_[i]->setAutoRepeatInterval(100);
        positive_jog_buttons_[i]->setAutoRepeatInterval(100);

        int axis_idx = static_cast<int>(i);
        connect(negative_jog_buttons_[i], &QPushButton::pressed,  this, [this, axis_idx]{ onInternalJogButtonPressed(axis_idx, false); });
        connect(negative_jog_buttons_[i], &QPushButton::released, this, [this, axis_idx]{ onInternalJogButtonReleased(axis_idx, false); });
        connect(positive_jog_buttons_[i], &QPushButton::pressed,  this, [this, axis_idx]{ onInternalJogButtonPressed(axis_idx, true); });
        connect(positive_jog_buttons_[i], &QPushButton::released, this, [this, axis_idx]{ onInternalJogButtonReleased(axis_idx, true); });

        axes_grid_layout->addWidget(axis_display_labels_[i], i, 0);
        axes_grid_layout->addWidget(negative_jog_buttons_[i], i, 1);
        axes_grid_layout->addWidget(positive_jog_buttons_[i], i, 2);
    }
    axes_control_group->setLayout(axes_grid_layout);
    main_layout->addWidget(axes_control_group);

    auto* actions_layout = new QHBoxLayout;
    go_home_button_ = new QPushButton("Go Home", this);
    connect(go_home_button_, &QPushButton::clicked, this, &Panel_JogControl::goHomeRequested);
    actions_layout->addWidget(go_home_button_);

    e_stop_button_ = new QPushButton("🛑 E-STOP", this);
    e_stop_button_->setStyleSheet("background-color: red; color: white; font-weight: bold;");
    connect(e_stop_button_, &QPushButton::clicked, this, &Panel_JogControl::emergencyStopRequested);
    actions_layout->addWidget(e_stop_button_);
    main_layout->addLayout(actions_layout);

    main_layout->addStretch(1); 
    setLayout(main_layout);
}

void Panel_JogControl::updateButtonAndAxisLabels() {
    const QStringList joint_axis_display_names = {"A1", "A2", "A3", "A4", "A5", "A6"};
    const QStringList cart_axis_display_names = {"X", "Y", "Z", "Rx", "Ry", "Rz"};
    const QStringList* current_display_names = (current_jog_mode_ == JogMode::Joint) ?
                                               &joint_axis_display_names : &cart_axis_display_names;

    for (unsigned int i = 0; i < ROBOT_AXES_COUNT; ++i) {
        if (axis_display_labels_[i]) { 
            axis_display_labels_[i]->setText((*current_display_names)[i] + ":");
        }
    }
    mode_toggle_button_->setText(current_jog_mode_ == JogMode::Joint ?
                                 "Current: Joint (Switch to Cartesian)" :
                                 "Current: Cartesian (Switch to Joint)");
    jog_frame_combo_->setEnabled(current_jog_mode_ == JogMode::Cartesian);
    updateStepSizeSuffix();
    LOG_DEBUG_F(MODULE_NAME, "UI labels updated for JogMode: %d", static_cast<int>(current_jog_mode_));
}

void Panel_JogControl::updateStepSizeSuffix() {
    if (current_jog_mode_ == JogMode::Joint) {
        step_size_spin_->setSuffix(" deg");
    } else {
        //      'mm'  'deg'    
        //  QDoubleSpinBox    .
        //      "mm/deg",   ,     .
        //   "mm/deg"  , ..      +/-.
        step_size_spin_->setSuffix(" mm | deg"); 
    }
}


// --- Internal Slots ---
void Panel_JogControl::onInternalModeToggle() {
    current_jog_mode_ = (current_jog_mode_ == JogMode::Joint) ? JogMode::Cartesian : JogMode::Joint;
    updateButtonAndAxisLabels();
    LOG_INFO_F(MODULE_NAME, "Jog mode toggled to: %s", (current_jog_mode_ == JogMode::Joint ? "Joint" : "Cartesian"));
}

void Panel_JogControl::onInternalStepModeToggle() {
    is_incremental_mode_ = !is_incremental_mode_;
    step_mode_toggle_button_->setText(is_incremental_mode_ ?
                                     "Step: Incremental (Switch to Continuous)" :
                                     "Step: Continuous (Switch to Incremental)");
    step_size_spin_->setEnabled(is_incremental_mode_);
    LOG_INFO_F(MODULE_NAME, "Step mode toggled to: %s", (is_incremental_mode_ ? "Incremental" : "Continuous"));
}

void Panel_JogControl::onInternalJogButtonPressed(int axis_index_0_based, bool is_positive_direction) {
    // BEGIN CORRECTION:  speed_ratio 
    double speed_ratio = static_cast<double>(jog_speed_slider_->value()) / 100.0;
    speed_ratio = std::max(0.01, speed_ratio); // Ensure minimum speed factor
    // END CORRECTION

    int button_state_idx = axis_index_0_based + (is_positive_direction ? static_cast<int>(ROBOT_AXES_COUNT) : 0);
    if(button_state_idx < static_cast<int>(continuous_button_pressed_state_.size())) { // Check bounds
        continuous_button_pressed_state_[button_state_idx] = true;
    }


    if (is_incremental_mode_) {
        double step_value_gui = step_size_spin_->value();
        double delta_value_cmd = is_positive_direction ? step_value_gui : -step_value_gui;

        if (current_jog_mode_ == JogMode::Joint) {
            LOG_DEBUG_F(MODULE_NAME, "Emitting jogIncrementJointRequested: Axis %d, Delta %.2f deg, SpeedRatio %.2f",
                        axis_index_0_based, delta_value_cmd, speed_ratio); // speed_ratio  
            emit jogIncrementJointRequested(axis_index_0_based, delta_value_cmd, speed_ratio);
        } else { // Cartesian
            double delta_for_signal = delta_value_cmd;
            if (axis_index_0_based >=3) { 
                delta_for_signal = Degrees(delta_value_cmd).toRadians().value();
            } 
            LOG_DEBUG_F(MODULE_NAME, "Emitting jogIncrementCartesianRequested: Axis %d, DeltaRaw %.2f, Frame %s, SpeedRatio %.2f",
                        axis_index_0_based, delta_value_cmd, jog_frame_combo_->currentText().toStdString().c_str(), speed_ratio); // speed_ratio  
            emit jogIncrementCartesianRequested(axis_index_0_based, delta_for_signal, jog_frame_combo_->currentText(), speed_ratio);
        }
    } else { // Continuous mode
        if (current_jog_mode_ == JogMode::Joint) {
            LOG_DEBUG_F(MODULE_NAME, "Emitting jogContinuousJointStartRequested: Axis %d, Dir %s, SpeedRatio %.2f",
                        axis_index_0_based, (is_positive_direction ? "+" : "-"), speed_ratio);
            emit jogContinuousJointStartRequested(axis_index_0_based, is_positive_direction, speed_ratio);
        } else { // Cartesian
            LOG_DEBUG_F(MODULE_NAME, "Emitting jogContinuousCartesianStartRequested: Axis %d, Dir %s, Frame %s, SpeedRatio %.2f",
                        axis_index_0_based, (is_positive_direction ? "+" : "-"), jog_frame_combo_->currentText().toStdString().c_str(), speed_ratio);
            emit jogContinuousCartesianStartRequested(axis_index_0_based, is_positive_direction, jog_frame_combo_->currentText(), speed_ratio);
        }
    }
}

void Panel_JogControl::onInternalJogButtonReleased(int axis_index_0_based, bool is_positive_direction) {
    int button_state_idx = axis_index_0_based + (is_positive_direction ? static_cast<int>(ROBOT_AXES_COUNT) : 0);
     if(button_state_idx < static_cast<int>(continuous_button_pressed_state_.size())) { // Check bounds
        continuous_button_pressed_state_[button_state_idx] = false;
    }

    if (!is_incremental_mode_) { 
        if (current_jog_mode_ == JogMode::Joint) {
            LOG_DEBUG_F(MODULE_NAME, "Emitting jogContinuousJointStopRequested: Axis %d", axis_index_0_based);
            emit jogContinuousJointStopRequested(axis_index_0_based);
        } else { 
             LOG_DEBUG_F(MODULE_NAME, "Emitting jogContinuousCartesianStopRequested: Axis %d, Frame %s",
                        axis_index_0_based, jog_frame_combo_->currentText().toStdString().c_str());
            emit jogContinuousCartesianStopRequested(axis_index_0_based, jog_frame_combo_->currentText());
        }
    }
}

void Panel_JogControl::onInternalSpeedSliderChanged(int value) {
    speed_display_label_->setText(QString("Speed: %1%").arg(value));
    emit jogSpeedChanged(value); 
    LOG_DEBUG_F(MODULE_NAME, "Jog speed changed to %d%%", value);
}

} // namespace RDT