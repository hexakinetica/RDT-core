// Panel_RobotView3D_UnitTest_main.cpp
#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QDebug>
#include <array> // For std::array

#include "Panel_RobotView3D.h" // Your Panel_RobotView3D class
#include "DataTypes.h" // For SixDofJointPose and M_PI (if defined there)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Forward declaration of the function that will update the robot
void updateRobotJointsFromUI(Panel_RobotView3D* robotView, const std::array<QDoubleSpinBox*, 6>& spinBoxes);

// Helper function to create a joint control group
QGroupBox* createJointControlGroup(
    const QString& title,
    Panel_RobotView3D* robotView, // Pass robotView
    const std::array<QDoubleSpinBox*, 6>& allSpinBoxes, // Pass all spinboxes for collective update
    QSlider*& slider,      // Output parameter for slider
    QLabel*& valueLabel,  // Output parameter for label
    QDoubleSpinBox*& spinBox // Output parameter for this joint's spinbox
    ) {
    QGroupBox *groupBox = new QGroupBox(title);
    QVBoxLayout *vLayout = new QVBoxLayout();

    QHBoxLayout *sliderLayout = new QHBoxLayout();
    slider = new QSlider(Qt::Horizontal);
    slider->setRange(-180, 180); // Degrees
    slider->setValue(0);
    valueLabel = new QLabel("0.0°");
    valueLabel->setFixedWidth(50);

    sliderLayout->addWidget(slider);
    sliderLayout->addWidget(valueLabel);
    vLayout->addLayout(sliderLayout);

    spinBox = new QDoubleSpinBox();
    spinBox->setRange(-360.0, 360.0); // Degrees
    spinBox->setDecimals(2);
    spinBox->setSuffix("°");
    spinBox->setValue(0.0);
    vLayout->addWidget(spinBox);

    groupBox->setLayout(vLayout);

    // Connect slider to update label, spinbox, AND the robot view
    QObject::connect(slider, &QSlider::valueChanged,
                     [valueLabel, spinBox, robotView, &allSpinBoxes](int value) {
                         double doubleValue = static_cast<double>(value);
                         valueLabel->setText(QString::number(doubleValue, 'f', 1) + "°");
                         // Temporarily block signals from spinBox to prevent infinite loop if spinBox also updates slider
                         bool oldSignalState = spinBox->blockSignals(true);
                         spinBox->setValue(doubleValue);
                         spinBox->blockSignals(oldSignalState);

                         updateRobotJointsFromUI(robotView, allSpinBoxes);
                     });

    // Connect spinbox to update label, slider, AND the robot view
    QObject::connect(spinBox, qOverload<double>(&QDoubleSpinBox::valueChanged),
                     [valueLabel, slider, robotView, &allSpinBoxes](double value) {
                         valueLabel->setText(QString::number(value, 'f', 1) + "°");
                         // Temporarily block signals from slider to prevent infinite loop
                         bool oldSignalState = slider->blockSignals(true);
                         slider->setValue(qRound(value));
                         slider->blockSignals(oldSignalState);

                         updateRobotJointsFromUI(robotView, allSpinBoxes);
                     });

    return groupBox;
}

// Function to read all joint values from UI and update Panel_RobotView3D
void updateRobotJointsFromUI(Panel_RobotView3D* robotView, const std::array<QDoubleSpinBox*, 6>& spinBoxes) {
    if (!robotView) return;

    RDT::AxisSet currentPose; // Expects angles in DEG
    currentPose.at(0).angle = RDT::Radians(spinBoxes[0]->value()*toRad);
    currentPose.at(1).angle = RDT::Radians(spinBoxes[1]->value()*toRad);
    currentPose.at(2).angle = RDT::Radians(spinBoxes[2]->value()*toRad);
    currentPose.at(3).angle = RDT::Radians(spinBoxes[3]->value()*toRad);
    currentPose.at(4).angle = RDT::Radians(spinBoxes[4]->value()*toRad);
    currentPose.at(5).angle = RDT::Radians(spinBoxes[5]->value()*toRad);

    // qDebug() << "Auto-updating joint angles (rad):"
    //          << currentPose.A1 << currentPose.A2 << currentPose.A3
    //          << currentPose.A4 << currentPose.A5 << currentPose.A6;
    robotView->update_jointpos(currentPose);
}


int main(int argc, char *argv[]) {
    QApplication::setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);
    QApplication app(argc, argv);

    QMainWindow mainWindow;
    mainWindow.setWindowTitle("Panel_RobotView3D - Interactive Joint Control Test");

    QWidget *centralWidget = new QWidget(&mainWindow);
    QHBoxLayout *mainLayout = new QHBoxLayout(centralWidget);

    QVBoxLayout *controlsLayout = new QVBoxLayout();
    controlsLayout->setSpacing(10);

    Panel_RobotView3D *robotView = new Panel_RobotView3D(centralWidget);
    robotView->setMinimumSize(800, 600);
    robotView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QPushButton *btnLoadModel = new QPushButton("Load Robot Model");
    QObject::connect(btnLoadModel, &QPushButton::clicked, robotView, &Panel_RobotView3D::loadmodel_r900);
    controlsLayout->addWidget(btnLoadModel);

    QPushButton *btnInitRobotVisuals = new QPushButton("Initialize Robot Visuals");
    QObject::connect(btnInitRobotVisuals, &QPushButton::clicked, robotView, &Panel_RobotView3D::Init_robot);
    controlsLayout->addWidget(btnInitRobotVisuals);

    controlsLayout->addSpacing(15);

    std::array<QSlider*, 6> jointSliders;       // Not strictly needed in main scope anymore
    std::array<QLabel*, 6> jointValueLabels;   // Same here
    std::array<QDoubleSpinBox*, 6> jointSpinBoxes; // This one is important to keep track of

    QGridLayout* jointControlsGrid = new QGridLayout();
    jointControlsGrid->setSpacing(5);

    for (int i = 0; i < 6; ++i) {
        QGroupBox* groupBox = createJointControlGroup(
            QString("Joint A%1").arg(i + 1),
            robotView,
            jointSpinBoxes, // Pass the array of all spinboxes
            jointSliders[i],    // These are output params for createJointControlGroup
            jointValueLabels[i],
            jointSpinBoxes[i]   // This specific spinbox is assigned here
            );
        jointControlsGrid->addWidget(groupBox, i / 2, i % 2);
    }
    controlsLayout->addLayout(jointControlsGrid);

    QPushButton *btnResetJoints = new QPushButton("Reset Joints to Zero");
    QObject::connect(btnResetJoints, &QPushButton::clicked,
                     [robotView, &jointSpinBoxes]() { // Removed jointSliders from capture as they are updated via spinboxes
                         for(int i=0; i<6; ++i) {
                             jointSpinBoxes[i]->setValue(0.0); // This will trigger valueChanged and auto-update
                         }
                         // The updateRobotJointsFromUI will be called by the last spinbox signal
                     });
    controlsLayout->addWidget(btnResetJoints);

    controlsLayout->addStretch();

    QWidget *controlsContainer = new QWidget();
    controlsContainer->setLayout(controlsLayout);
    controlsContainer->setFixedWidth(350);

    mainLayout->addWidget(controlsContainer);
    mainLayout->addWidget(robotView, 1);

    centralWidget->setLayout(mainLayout);
    mainWindow.setCentralWidget(centralWidget);
    mainWindow.resize(1200, 700);
    mainWindow.show();

    robotView->Set_perspective();
    robotView->set_view_top();
    robotView->Redraw();

    // Initial update to render robot at zero position if not already handled by Init_robot/loadmodel
    // Or after model is loaded by button click.
    // For safety, explicitly set to zero after UI is shown and model potentially loaded.
    // We can call reset joints logic once after setup.
    QTimer::singleShot(100, [robotView, &jointSpinBoxes](){ // Slight delay to ensure UI is ready
        for(int i=0; i<6; ++i) {
            if(jointSpinBoxes[i]) jointSpinBoxes[i]->setValue(0.0);
        }
        // updateRobotJointsFromUI will be called due to spinbox valueChanged signals
    });


    return app.exec();
};
