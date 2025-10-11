
#ifndef Panel_RobotView3D_H
#define Panel_RobotView3D_H
#pragma once
#include <QWidget>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QApplication>
#include <QTimer>

#include <AIS_InteractiveContext.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <V3d_View.hxx>
#include <Aspect_Handle.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Graphic3d_GraphicDriver.hxx>
#ifdef __linux__
#include <X11/Xlib.h>
#undef Bool
#undef None
#undef event
#undef True
#undef False
#undef Status
#undef CursorShape // example, add others if conflicts arise
#endif
#include <QOpenGLWidget> // Make sure this is used instead of QGLWidget
#include <AIS_InteractiveContext.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <V3d_View.hxx>
#include <Aspect_Handle.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Graphic3d_GraphicDriver.hxx>
#ifdef _WIN32
#include <WNT_Window.hxx>
#else
#undef None // Keep this if needed for conflicts
#include <Xw_Window.hxx> // This header *uses* the Window type
#endif
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <AIS_Shape.hxx>
#include <TDF_Label.hxx>
// #include "defines.h" // Assuming this is a custom header, might need to be adapted or included

// show xyz axis
#include <Geom_Axis2Placement.hxx>
#include <AIS_Trihedron.hxx>

// #include <variable.h> // Assuming this is a custom header, might need to be adapted or included
// #include <draw_primitives.h> // Assuming this is a custom header, might need to be adapted or included
#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp_Explorer.hxx>
#include <BRep_Tool.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <Quantity_Color.hxx>
#include <AIS_ViewCube.hxx>
#include <Graphic3d_TransformPers.hxx>
//#include <Aspect_TOTP.hxx>
//#include <Graphic3d_Vec2i.hxx>
//#include <V3d_Camera.hxx>
#include <Graphic3d_Camera.hxx>
#include <Aspect_GradientFillMethod.hxx>
#include <Prs3d_Drawer.hxx>
#include <Aspect_TypeOfHighlightMethod.hxx>
#include <Quantity_Color.hxx>
#include <Aspect_GridType.hxx>
//#include <Aspect_GridDrawingMode.hxx>
//#include <V3d_TypeOfAisObject.hxx>
#include <Prs3d_DatumAspect.hxx>
#include <Prs3d_LineAspect.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <AIS_StatusOfPick.hxx>
#include <Font_BRepFont.hxx>
#include <Font_BRepTextBuilder.hxx>
#include <NCollection_String.hxx>
#include <sstream>


//! Make conversion's easy:
 #define toRad M_PI/180.0
 #define toDeg 180.0/M_PI

#include "DataTypes.h"

// Assuming SixDofJointPose is defined elsewhere, potentially in defines.h or kinematic.h
//struct SixDofJointPose {
//    double A1, A2, A3, A4, A5, A6;
//};


struct POINT{
    double x=0,y=0,z=0;
};

struct SEGMENT{
    std::vector<Handle(AIS_Shape)> Ais_ShapeVec={}; ///each stepfile can contain multiple shapes, we need the vector.
    gp_Trsf MyTrsf={};
};


// Used as universal toolset.
struct gp_Euler_t {
    double x, y, z;
};
#define gp_Euler gp_Euler_t


class Panel_RobotView3D: public QOpenGLWidget
{
    Q_OBJECT
public:
    explicit Panel_RobotView3D(QWidget *parent = nullptr);
    ~Panel_RobotView3D();

    struct io {
        std::string halcommand={""};
    };

    struct bucket {
        std::string primitivetype;      // line,wire,arc,circle,spline.
        std::vector<gp_Pnt> pointvec;
        std::vector<gp_Euler_t> eulervec;
        Handle(AIS_Shape) Ais_shape;
        std::string info;

        // Hal io
        std::vector<io> iovec;

        double vo=0,ve=0,velmax=0,accmax=0;

        // Store gp_Trsf for euler orientations
        std::vector<gp_Trsf> eulervec_gp_Trsf;
    };
    std::vector<bucket> bucketvec;

    std::vector<Handle(AIS_Shape)> previewbucketvec;
    std::vector<SEGMENT> SegmentVec;


    bool Readstepfile(const std::string& theStepName);
    void Visit(const TDF_Label& theLabel);
    void Init_robot(); // This method was declared but not defined in the original files
    void setup_tcp_origin();

    void show_shape(Handle(AIS_Shape) shape);
    void Redraw();
    void update_jointpos(RDT::AxisSet axis);
    // void update_jointpos_arr(std::array<double, 6> pose){
    //     SixDofJointPose pose_;
    //     pose_.A1=pose[0];pose_.A2=pose[1];pose_.A3=pose[2];pose_.A4=pose[3];pose_.A5=pose[4];pose_.A6=pose[5];
    //     update_jointpos(pose_);
    // }
    // Draw primitives and tool orientation and store them with extra info.
    void draw_line_store_into_bucketvec(bucket b);

    // Preview line
    void draw_preview_cone(std::string type, gp_Trsf trsf);
    void empty_preview_bucket();

    // View
    void Set_orthographic();
    void Set_perspective();
    void set_view_front();
    void set_view_back();
    void set_view_left();
    void set_view_right();
    void set_view_top();
    void set_view_bottom();

    // Selection
    void get_selections();
    void delete_selections();

    // Load robot model
    bool loadmodel_r900();

private:
    void m_initialize_context();
    Handle(AIS_InteractiveContext) m_context;
    Handle(V3d_Viewer) m_viewer;
    Handle(V3d_View) m_view;
    Handle(Graphic3d_GraphicDriver) m_graphic_driver;
    Handle(AIS_InteractiveObject) m_aisViewCube;

    // Xyz axis sign.
    Handle(Geom_Axis2Placement) axis;
    Handle(AIS_Trihedron) aisTrihedron;
    std::vector<Handle(AIS_Trihedron)> aisTrihedrons;

protected:
    void paintEvent(QPaintEvent *);
    void resizeEvent(QResizeEvent *);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

protected:
    enum CurrentAction3d
    {
        CurAction3d_Nothing,
        CurAction3d_DynamicPanning,
        CurAction3d_DynamicZooming,
        CurAction3d_DynamicRotation
    };

private:
    Standard_Integer m_x_max;
    Standard_Integer m_y_max;
    CurrentAction3d m_current_mode;
    //gp_Trsf current_tcp;

    Handle(AIS_Shape) aisBody_tcp_xaxis, aisBody_tcp_yaxis, aisBody_tcp_zaxis;

    // Create the euler lines
    double toollenght=105;
    double linelenght=25;
    double coneheight=25;
    double conetopdiameter=1;
    double conebottomdiameter=5;
    double textheight=25;

    gp_Trsf level0x1x2x3x4x5x6;


    TopoDS_Edge edge_linepreview;
    Handle(AIS_Shape) aisBody_linepreview;

signals:

public slots:
    void mainloop();
private:
    int ready = 0;

};

#endif // Panel_RobotView3D_H
