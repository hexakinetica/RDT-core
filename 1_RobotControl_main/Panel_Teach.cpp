// Panel_Teach.cpp
#include "Panel_Teach.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QInputDialog> //     
#include <QDebug>       //  qDebug
#include <QGroupBox>

namespace RDT {

using namespace RDT::literals;

Panel_Teach::Panel_Teach(QWidget* parent_widget)
    : QObject(nullptr) // QObject   - ,    
{
    main_widget_ = new QWidget(parent_widget); //   
    setupUI();
    onProgramSpeedSliderChanged(program_speed_slider_->value()); // Initialize label
    LOG_DEBUG(MODULE_NAME, "Panel_Teach created and UI set up.");
}

QWidget* Panel_Teach::getWidget() {
    return main_widget_;
}

void Panel_Teach::setupUI() {
    auto* main_layout = new QVBoxLayout(main_widget_); // Layout  main_widget_
    main_layout->setSpacing(10);
    main_layout->setContentsMargins(8,8,8,8);

    QFont groupFont("Arial", 10, QFont::Bold);

    // --- Frame & Tool Selection Group ---
    auto* frame_tool_group = new QGroupBox("Active Context for New/Taught Points", main_widget_);
    frame_tool_group->setFont(groupFont);
    auto* ft_layout = new QGridLayout;
    ft_layout->addWidget(new QLabel("Tool:", main_widget_), 0, 0, Qt::AlignRight);
    tool_combo_ = new QComboBox(main_widget_);
    tool_combo_->addItems({"DefaultTool", "Gripper", "Welder", "ThinTool"}); // 
    connect(tool_combo_, &QComboBox::currentTextChanged, this, &Panel_Teach::onToolComboBoxChanged);
    ft_layout->addWidget(tool_combo_, 0, 1);

    ft_layout->addWidget(new QLabel("Base:", main_widget_), 1, 0, Qt::AlignRight);
    base_combo_ = new QComboBox(main_widget_);
    base_combo_->addItems({"RobotBase", "UserBase1", "FixtureA", "TableSurface"}); // 
    connect(base_combo_, &QComboBox::currentTextChanged, this, &Panel_Teach::onBaseComboBoxChanged);
    ft_layout->addWidget(base_combo_, 1, 1);
    ft_layout->setColumnStretch(1,1);
    frame_tool_group->setLayout(ft_layout);
    main_layout->addWidget(frame_tool_group);

    // --- Program List ---
    program_list_widget_ = new QListWidget(main_widget_);
    program_list_widget_->setSelectionMode(QAbstractItemView::SingleSelection);
    program_list_widget_->setAlternatingRowColors(true);
    main_layout->addWidget(program_list_widget_, 1); // Allow list to stretch

    // --- Point Editing Buttons ---
    auto* edit_buttons_layout = new QHBoxLayout;
    btn_add_point_ = new QPushButton("Teach Current Pose", main_widget_);
    connect(btn_add_point_, &QPushButton::clicked, this, &Panel_Teach::onBtnTeachClicked);
    edit_buttons_layout->addWidget(btn_add_point_);

    btn_touch_up_point_ = new QPushButton("Touch-Up Selected", main_widget_);
    connect(btn_touch_up_point_, &QPushButton::clicked, this, &Panel_Teach::onBtnTouchUpClicked);
    edit_buttons_layout->addWidget(btn_touch_up_point_);

    btn_edit_attribs_ = new QPushButton("Edit Attrs Selected", main_widget_);
    btn_edit_attribs_->setToolTip("Edit motion type, speed of the selected point (not pose)");
    // connect(btn_edit_attribs_, &QPushButton::clicked, this, &Panel_Teach::onBtnEditAttribsClicked); // TODO
    btn_edit_attribs_->setEnabled(false); // Placeholder
    edit_buttons_layout->addWidget(btn_edit_attribs_);

    btn_delete_point_ = new QPushButton("Delete Selected", main_widget_);
    connect(btn_delete_point_, &QPushButton::clicked, this, &Panel_Teach::onBtnDeleteClicked);
    edit_buttons_layout->addWidget(btn_delete_point_);
    main_layout->addLayout(edit_buttons_layout);

    // --- Motion Type for New Points ---
    auto* motion_type_group = new QGroupBox("Motion Type for New/Taught Point", main_widget_);
    motion_type_group->setFont(groupFont);
    auto* mt_layout = new QHBoxLayout;
    motion_type_combo_ = new QComboBox(main_widget_);
    //  RDT::MotionType,      int
    motion_type_combo_->addItem("JOINT (PTP to joint target)", static_cast<int>(MotionType::JOINT));
    motion_type_combo_->addItem("LIN (Linear to Cartesian TCP)", static_cast<int>(MotionType::LIN));
    motion_type_combo_->addItem("PTP (Cartesian target, joint interp)", static_cast<int>(MotionType::PTP));
    // motion_type_combo_->addItem("CIRC", static_cast<int>(MotionType::CIRC)); //  
    mt_layout->addWidget(motion_type_combo_);
    motion_type_group->setLayout(mt_layout);
    main_layout->addWidget(motion_type_group);

    // --- Program Speed Control ---
    auto* speed_control_group = new QGroupBox("Program Execution Speed Override", main_widget_);
    speed_control_group->setFont(groupFont);
    auto* speed_layout = new QHBoxLayout;
    program_speed_slider_ = new QSlider(Qt::Horizontal, main_widget_);
    program_speed_slider_->setRange(1, 100); // 1% to 100%
    program_speed_slider_->setValue(100);
    program_speed_slider_->setTickInterval(10);
    program_speed_slider_->setTickPosition(QSlider::TicksBelow);
    program_speed_label_ = new QLabel(main_widget_); // Text set by slot
    program_speed_label_->setMinimumWidth(120);
    program_speed_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    connect(program_speed_slider_, &QSlider::valueChanged, this, &Panel_Teach::onProgramSpeedSliderChanged);
    speed_layout->addWidget(program_speed_label_);
    speed_layout->addWidget(program_speed_slider_);
    speed_control_group->setLayout(speed_layout);
    main_layout->addWidget(speed_control_group);

    // --- Program Execution Buttons ---
    auto* exec_buttons_layout = new QHBoxLayout;
    btn_run_program_ = new QPushButton("▶ Run Program", main_widget_);
    connect(btn_run_program_, &QPushButton::clicked, this, &Panel_Teach::onBtnRunClicked);
    exec_buttons_layout->addWidget(btn_run_program_);

    btn_pause_program_ = new QPushButton("⏸ Pause/Resume", main_widget_);
    connect(btn_pause_program_, &QPushButton::clicked, this, &Panel_Teach::onBtnPauseResumeClicked);
    exec_buttons_layout->addWidget(btn_pause_program_);

    btn_stop_program_ = new QPushButton("⏹ Stop Program", main_widget_);
    connect(btn_stop_program_, &QPushButton::clicked, this, &Panel_Teach::onBtnStopClicked);
    exec_buttons_layout->addWidget(btn_stop_program_);
    main_layout->addLayout(exec_buttons_layout);

    main_layout->addStretch(1); // Pushes content to the top
}

// --- Public Methods ---
void Panel_Teach::setCurrentToolInComboBox(const QString& toolName) {
    tool_combo_->blockSignals(true);
    tool_combo_->setCurrentText(toolName);
    tool_combo_->blockSignals(false);
}

void Panel_Teach::setCurrentBaseInComboBox(const QString& baseName) {
    base_combo_->blockSignals(true);
    base_combo_->setCurrentText(baseName);
    base_combo_->blockSignals(false);
}

void Panel_Teach::addTrajectoryPointToDisplay(const RDT::TrajectoryPoint& tp, const QString& pointName) {
    QString name_to_display = pointName;
    if (name_to_display.isEmpty()) {
        name_to_display = QString("P%1").arg(program_list_widget_->count() + 1);
    }
    program_list_widget_->addItem(formatTrajectoryPointForDisplay(tp, name_to_display));
    //   , current_program_data_   ,    .
}

void Panel_Teach::clearProgramDisplay() {
    program_list_widget_->clear();
    current_program_data_.clear(); //    
}

void Panel_Teach::refreshProgramDisplay(const QVector<RDT::TrajectoryPoint>& program) {
    program_list_widget_->clear();
    current_program_data_ = program; //    
    for (int i = 0; i < program.size(); ++i) {
        program_list_widget_->addItem(formatTrajectoryPointForDisplay(program[i], QString("P%1").arg(i + 1)));
    }
}

void Panel_Teach::highlightProgramStep(int index) {
    if (index >= 0 && index < program_list_widget_->count()) {
        program_list_widget_->setCurrentRow(index);
        //     
        for (int i = 0; i < program_list_widget_->count(); ++i) {
            program_list_widget_->item(i)->setBackground( (i == index) ? Qt::yellow : Qt::white );
        }
    } else {
        program_list_widget_->setCurrentRow(-1); //  
         for (int i = 0; i < program_list_widget_->count(); ++i) {
            program_list_widget_->item(i)->setBackground(Qt::white);
        }
    }
}

// --- Getters for Panel Settings ---
RDT::MotionType Panel_Teach::getSelectedMotionTypeForNewPoint() const {
    return static_cast<RDT::MotionType>(motion_type_combo_->currentData().toInt());
}
int Panel_Teach::getProgramSpeedPercent() const {
    return program_speed_slider_->value();
}
QString Panel_Teach::getCurrentToolNameFromComboBox() const {
    return tool_combo_->currentText();
}
QString Panel_Teach::getCurrentBaseNameFromComboBox() const {
    return base_combo_->currentText();
}
int Panel_Teach::getSelectedProgramStepIndex() const {
    return program_list_widget_->currentRow();
}

// --- Private Slots ---
void Panel_Teach::onBtnTeachClicked() {
    LOG_DEBUG(MODULE_NAME, "Teach button clicked.");
    emit teachCurrentPoseRequested(
        getSelectedMotionTypeForNewPoint(),
        getCurrentToolNameFromComboBox(),
        getCurrentBaseNameFromComboBox(),
        static_cast<double>(getProgramSpeedPercent()) / 100.0
    );
}
void Panel_Teach::onBtnDeleteClicked() {
    int current_row = program_list_widget_->currentRow();
    LOG_DEBUG_F(MODULE_NAME, "Delete button clicked for row: %d", current_row);
    if (current_row >= 0) {
        emit deleteProgramPointRequested(current_row);
        //    current_program_data_  QListWidget   ,
        //     ,     .
        //  ,    refreshProgramDisplay.
    } else {
        LOG_WARN(MODULE_NAME, "Delete clicked but no item selected.");
    }
}
void Panel_Teach::onBtnTouchUpClicked() {
    int current_row = program_list_widget_->currentRow();
    LOG_DEBUG_F(MODULE_NAME, "Touch-up button clicked for row: %d", current_row);
    if (current_row >= 0) {
        emit touchUpProgramPointRequested(
            current_row,
            getSelectedMotionTypeForNewPoint(), //           
            getCurrentToolNameFromComboBox(),
            getCurrentBaseNameFromComboBox(),
            static_cast<double>(getProgramSpeedPercent()) / 100.0
        );
    } else {
        LOG_WARN(MODULE_NAME, "Touch-up clicked but no item selected.");
    }
}
void Panel_Teach::onBtnRunClicked() {
    LOG_INFO(MODULE_NAME, "Run Program button clicked.");
    emit runProgramExecutionRequested();
}
void Panel_Teach::onBtnPauseResumeClicked() {
    LOG_INFO(MODULE_NAME, "Pause/Resume Program button clicked.");
    emit pauseResumeProgramExecutionRequested();
}
void Panel_Teach::onBtnStopClicked() {
    LOG_INFO(MODULE_NAME, "Stop Program button clicked.");
    emit stopProgramExecutionRequested();
}
void Panel_Teach::onProgramSpeedSliderChanged(int value) {
    program_speed_label_->setText(QString("Program Speed: %1%").arg(value));
    emit programSpeedOverrideChanged(value);
}
void Panel_Teach::onToolComboBoxChanged(const QString& toolName) {
    LOG_DEBUG_F(MODULE_NAME, "Tool ComboBox changed to: %s", toolName.toStdString().c_str());
    emit activeToolSelectionChanged(toolName);
}
void Panel_Teach::onBaseComboBoxChanged(const QString& baseName) {
    LOG_DEBUG_F(MODULE_NAME, "Base ComboBox changed to: %s", baseName.toStdString().c_str());
    emit activeBaseSelectionChanged(baseName);
}


// --- Static Helper Function ---
static QString getMotionTypeString(RDT::MotionType type) {
    switch (type) {
        case MotionType::JOINT: return "JOINT";
        case MotionType::LIN:   return "LIN";
        case MotionType::PTP:   return "PTP";
        case MotionType::CIRC:  return "CIRC";
        case MotionType::SPLINE:return "SPLINE";
        default: return "UNKNOWN";
    }
}

// --- Private Helper Methods ---
QString Panel_Teach::formatTrajectoryPointForDisplay(const RDT::TrajectoryPoint& tp, const QString& pointName) const {
    QString pose_str;
    //    ,    
    if (tp.header.data_type == WaypointDataType::JOINT_DOMINANT_CMD || tp.header.motion_type == MotionType::JOINT || tp.header.motion_type == MotionType::PTP) {
        pose_str = QString::fromStdString(tp.command.joint_target.toJointPoseString(2)); // 2     
    } else { // CARTESIAN_DOMINANT_CMD (LIN)
        pose_str = QString::fromStdString(tp.command.cartesian_target.toDescriptiveString()); //  toString()  CartPose
    }

    return QString("%1 [%2] %3 (T:'%4' B:'%5') Speed:%6%")
        .arg(pointName.isEmpty() ? "P?" : pointName)
        .arg(getMotionTypeString(tp.header.motion_type)) //   getMotionTypeString
        .arg(pose_str)
        .arg(QString::fromStdString(tp.header.tool.name))
        .arg(QString::fromStdString(tp.header.base.name))
        .arg(static_cast<int>(tp.command.speed_ratio * 100.0));
}

} // namespace RDT