// Mock_Adapter.cpp
#include "Mock_Adapter.h"
#include <QApplication> // <-- Add this include

namespace RDT {

using namespace RDT::literals;

Mock_Adapter::Mock_Adapter(Panel_Teach* panel_to_connect, QObject* parent)
    : QObject(parent), teach_panel_ptr_(panel_to_connect) {
    if (!teach_panel_ptr_) {
        LOG_CRITICAL(MODULE_NAME, "Panel_Teach pointer is null in MockAdapter constructor.");
        return; // Or throw
    }

    // // Connect signals from Panel_Teach to slots in this mock adapter
    // connect(teach_panel_ptr_, &Panel_Teach::teachCurrentPoseRequested,
    //         this, &Mock_Adapter::onGuiTeachCurrentPoseRequested);
    // connect(teach_panel_ptr_, &Panel_Teach::deleteProgramPointRequested,
    //         this, &Mock_Adapter::onGuiDeleteProgramPointRequested);
    // connect(teach_panel_ptr_, &Panel_Teach::touchUpProgramPointRequested,
    //         this, &Mock_Adapter::onGuiTouchUpProgramPointRequested);
    // connect(teach_panel_ptr_, &Panel_Teach::runProgramExecutionRequested,
    //         this, &Mock_Adapter::onGuiRunProgramExecutionRequested);
    // connect(teach_panel_ptr_, &Panel_Teach::pauseResumeProgramExecutionRequested,
    //         this, &Mock_Adapter::onGuiPauseResumeProgramExecutionRequested);
    // connect(teach_panel_ptr_, &Panel_Teach::stopProgramExecutionRequested,
    //         this, &Mock_Adapter::onGuiStopProgramExecutionRequested);
    // connect(teach_panel_ptr_, &Panel_Teach::programSpeedOverrideChanged,
    //         this, &Mock_Adapter::onGuiProgramSpeedOverrideChanged);
    // connect(teach_panel_ptr_, &Panel_Teach::activeToolSelectionChanged,
    //         this, &Mock_Adapter::onGuiActiveToolSelectionChanged);
    // connect(teach_panel_ptr_, &Panel_Teach::activeBaseSelectionChanged,
    //         this, &Mock_Adapter::onGuiActiveBaseSelectionChanged);

    // // Connect signal from this mock to panel's slot for highlighting
    // connect(this, &Mock_Adapter::highlightStepInPanelSignal,
    //         teach_panel_ptr_, &Panel_Teach::highlightProgramStep);


    LOG_INFO_F(MODULE_NAME, "Mock_Adapter created and connected to Panel_Teach '%s'.",
               teach_panel_ptr_->getWidget()->objectName().toStdString().c_str());
}

void Mock_Adapter::setInitialProgram(const QVector<RDT::TrajectoryPoint>& program) {
    program_data_mock_ = program;
    if (teach_panel_ptr_) {
        teach_panel_ptr_->refreshProgramDisplay(program_data_mock_);
    }
}


// --- Slots Implementation ---
void Mock_Adapter::onGuiTeachCurrentPoseRequested(RDT::MotionType type, const QString& toolName, const QString& baseName, double speedRatio) {
    LOG_INFO_F(MODULE_NAME, "SLOT: TeachCurrentPose: Type %d, Tool '%s', Base '%s', Speed %.2f",
               static_cast<int>(type), toolName.toStdString().c_str(), baseName.toStdString().c_str(), speedRatio);
    
    TrajectoryPoint new_tp;
    new_tp.header.motion_type = type;
    new_tp.header.tool = getToolFrameByName(toolName); //  
    new_tp.header.base = getBaseFrameByName(baseName); //  
    new_tp.command.speed_ratio = speedRatio;

    // Simulate getting current robot pose (e.g., slightly offset from last point or a fixed pose)
    if (!program_data_mock_.isEmpty()) {
        new_tp.command.joint_target = program_data_mock_.last().command.joint_target;
        new_tp.command.joint_target[AxisId::A1].angle += (5.0_deg).toRadians(); // Increment A1
        new_tp.command.cartesian_target = program_data_mock_.last().command.cartesian_target;
        new_tp.command.cartesian_target.x += 0.01_m; // Increment X
    } else { // First point
        new_tp.command.joint_target[AxisId::A1].angle = (10.0_deg).toRadians();
        new_tp.command.cartesian_target.x = 0.1_m;
    }
    new_tp.header.sequence_index = static_cast<uint32_t>(program_data_mock_.size());
    
    program_data_mock_.append(new_tp);
    if (teach_panel_ptr_) {
        //  addTrajectoryPointToDisplay,     ,
        //  refreshProgramDisplay,  QListWidget   program_data_mock_
        teach_panel_ptr_->refreshProgramDisplay(program_data_mock_);
        // teach_panel_ptr_->addTrajectoryPointToDisplay(new_tp, QString("P%1_mock").arg(program_data_mock_.size()));
    }
}

void Mock_Adapter::onGuiDeleteProgramPointRequested(int index) {
    LOG_INFO_F(MODULE_NAME, "SLOT: DeleteProgramPoint: Index %d", index);
    if (index >= 0 && index < program_data_mock_.size()) {
        program_data_mock_.remove(index);
        if (teach_panel_ptr_) {
            teach_panel_ptr_->refreshProgramDisplay(program_data_mock_);
        }
    } else {
        LOG_WARN_F(MODULE_NAME, "Delete requested for invalid index: %d (size: %d)", index, program_data_mock_.size());
    }
}

void Mock_Adapter::onGuiTouchUpProgramPointRequested(int index, RDT::MotionType type, const QString& newToolName, const QString& newBaseName, double speedRatio) {
    LOG_INFO_F(MODULE_NAME, "SLOT: TouchUpProgramPoint: Index %d, Type %d, Tool '%s', Base '%s', Speed %.2f",
               index, static_cast<int>(type), newToolName.toStdString().c_str(), newBaseName.toStdString().c_str(), speedRatio);
    if (index >= 0 && index < program_data_mock_.size()) {
        program_data_mock_[index].header.motion_type = type;
        program_data_mock_[index].header.tool = getToolFrameByName(newToolName);
        program_data_mock_[index].header.base = getBaseFrameByName(newBaseName);
        program_data_mock_[index].command.speed_ratio = speedRatio;
        
        // Simulate re-teaching pose to a new "current" robot pose
        program_data_mock_[index].command.joint_target[AxisId::A1].angle = (45.0_deg + Degrees(5.0 * index)).toRadians(); // New mock pose
        program_data_mock_[index].command.cartesian_target.x = (0.2_m + (Meters(0.05) * static_cast<double>(index))); // New mock pose
        
        if (teach_panel_ptr_) {
            teach_panel_ptr_->refreshProgramDisplay(program_data_mock_);
        }
    } else {
        LOG_WARN_F(MODULE_NAME, "TouchUp requested for invalid index: %d (size: %d)", index, program_data_mock_.size());
    }
}

void Mock_Adapter::onGuiRunProgramExecutionRequested() {
    LOG_INFO(MODULE_NAME, "SLOT: RunProgramExecutionRequested");
    // Simulate program execution by highlighting steps
    if (!program_data_mock_.isEmpty() && teach_panel_ptr_) {
        LOG_INFO_F(MODULE_NAME, "Simulating program run (%d points)...", program_data_mock_.size());
        for (int i = 0; i < program_data_mock_.size(); ++i) {
            //        RobotController
            //        GUI
            LOG_DEBUG_F(MODULE_NAME, "  Executing mock step %d", i);
            emit highlightStepInPanelSignal(i); //    
            QApplication::processEvents(); //  GUI 
            std::this_thread::sleep_for(std::chrono::milliseconds(300)); // 
        }
        emit highlightStepInPanelSignal(-1); //  
        LOG_INFO(MODULE_NAME, "Mock program run finished.");
    }
}

void Mock_Adapter::onGuiPauseResumeProgramExecutionRequested() {
    LOG_INFO(MODULE_NAME, "SLOT: PauseResumeProgramExecutionRequested");
}
void Mock_Adapter::onGuiStopProgramExecutionRequested() {
    LOG_INFO(MODULE_NAME, "SLOT: StopProgramExecutionRequested");
    if (teach_panel_ptr_) emit highlightStepInPanelSignal(-1); //    
}
void Mock_Adapter::onGuiProgramSpeedOverrideChanged(int speed_percent) {
    LOG_INFO_F(MODULE_NAME, "SLOT: ProgramSpeedOverrideChanged: %d%%", speed_percent);
}
void Mock_Adapter::onGuiActiveToolSelectionChanged(const QString& toolName) {
    LOG_INFO_F(MODULE_NAME, "SLOT: ActiveToolSelectionChanged: %s", toolName.toStdString().c_str());
    //       StateData   
}
void Mock_Adapter::onGuiActiveBaseSelectionChanged(const QString& baseName) {
    LOG_INFO_F(MODULE_NAME, "SLOT: ActiveBaseSelectionChanged: %s", baseName.toStdString().c_str());
}

void Mock_Adapter::simulateHighlightStep(int index) {
    emit highlightStepInPanelSignal(index);
}

//  ()
RDT::ToolFrame Mock_Adapter::getToolFrameByName(const QString& name) const {
    if (name.compare("Gripper", Qt::CaseInsensitive) == 0)
        return ToolFrame(CartPose{0.0_m, 0.0_m, 0.20_m}, "Gripper");
    if (name.compare("Welder", Qt::CaseInsensitive) == 0)
        return ToolFrame(CartPose{0.05_m, 0.0_m, 0.15_m}, "Welder");
    if (name.compare("ThinTool", Qt::CaseInsensitive) == 0)
        return ToolFrame(CartPose{0.0_m, 0.0_m, 0.1_m}, "ThinTool");
    return ToolFrame(CartPose{}, name.toStdString().empty() ? "DefaultTool" : name.toStdString());
}

RDT::BaseFrame Mock_Adapter::getBaseFrameByName(const QString& name) const {
    if (name.compare("UserBase1", Qt::CaseInsensitive) == 0)
        return BaseFrame(CartPose{0.1_m, 0.1_m, 0.1_m}, "UserBase1");
    if (name.compare("FixtureA", Qt::CaseInsensitive) == 0)
        return BaseFrame(CartPose{0.1_m, 0.1_m, 0.0_m}, "FixtureA");
    if (name.compare("TableSurface", Qt::CaseInsensitive) == 0)
        return BaseFrame(CartPose{0.0_m, 0.0_m, -0.05_m}, "TableSurface");
    return BaseFrame(CartPose{}, name.toStdString().empty() ? "RobotBase" : name.toStdString());
}

}; // namespace RDT