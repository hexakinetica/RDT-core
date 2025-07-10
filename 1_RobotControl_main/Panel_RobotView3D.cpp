#include "Panel_RobotView3D.h"
// #include "kinematic.h" // Assuming this is a custom header
// #include "halio.h" // Assuming this is a custom header

#include <OSD.hxx>
#include <AIS_Trihedron.hxx>
#include <AIS_ViewCube.hxx>
#include <AIS_Selection.hxx>
#include <AIS_ColoredShape.hxx>
#include <AIS_ColoredDrawer.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepTools_ReShape.hxx>
#include <BRepTools.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <gp_Trsf.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>
#include <TPrsStd_AISPresentation.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Controller.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_CartesianPoint.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeArcOfEllipse.hxx>
#include <GC_MakeCircle.hxx>
#include <GC_MakeEllipse.hxx>
#include <GC_MakeSegment.hxx>
#include <gce_MakeRotation.hxx>
#include <TopExp.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Edge.hxx>
#include <TDocStd_Document.hxx>
#include <TDocStd_Application.hxx>
#include <TDF_Label.hxx>
#include <TDF_AttributeIterator.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TDataStd_TreeNode.hxx>
#include <TDataStd_UAttribute.hxx>
#include <TNaming_NamedShape.hxx>
#include <TopTools.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include "Geom_Axis2Placement.hxx"
#include <GCE2d_MakeSegment.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <XCAFDoc_Area.hxx>
#include <XCAFDoc_Centroid.hxx>
#include <XCAFDoc_Datum.hxx>
#include <XCAFDoc_Dimension.hxx>
#include <XCAFDoc_Location.hxx>
#include <XCAFDoc_Volume.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <Quantity_Color.hxx>
#include <Quantity_ColorRGBA.hxx>

#include "XCAFPrs_DocumentExplorer.hxx" // Assuming this is a custom header
#include <TDataStd_Name.hxx>
#include <XCAFDoc_AssemblyItemId.hxx>
#include <XCAFDoc_AssemblyItemRef.hxx>

#include <BRepTools.hxx>
#include <Font_BRepFont.hxx>
#include <Font_BRepTextBuilder.hxx>
#include <Bnd_Box.hxx>

#include "gp_Elips.hxx" // Assuming this is a custom header or typo for gp_Elips
#include <NCollection_Mat4.hxx>
#include <gp_Quaternion.hxx>

#include <QDebug> // For qWarning

// strore the data from the stepfile.
std::vector<TopoDS_Shape> Shape; // Consider making this a member of the class
std::vector<Handle(AIS_Shape)> ais_Shape; // Consider making this a member of the class
std::vector<Quantity_Color> colv; // Consider making this a member of the class

Handle(XCAFDoc_ColorTool) aColorTool; // Consider making this a member of the class
Handle(XCAFDoc_ShapeTool) aShapeTool; // Consider making this a member of the class


Panel_RobotView3D::Panel_RobotView3D(QWidget *parent) : QOpenGLWidget(parent)
{
    setBackgroundRole( QPalette::NoRole );
    setMouseTracking( true );

    QTimer *timer = new QTimer(this);
   // connect(timer, &QTimer::timeout, this, &Panel_RobotView3D::mainloop);
    timer->start(1000); //1000 milliseond.
}

Panel_RobotView3D::~Panel_RobotView3D()
{
    // Clean up OpenCASCADE resources if necessary
    if (!m_context.IsNull()) {
        m_context->RemoveAll(true);
    }
    if (!m_view.IsNull()) {
        m_view->Remove();
    }
    if (!m_viewer.IsNull()) {
        //m_viewer->SetDisplayConnection(Handle(Aspect_DisplayConnection)());
    }
}


bool Panel_RobotView3D::Readstepfile(const std::string& theStepName)
{
    SEGMENT segment; // Use a local segment
    SegmentVec.push_back(segment); // Add a new segment for each file

    STEPCAFControl_Controller::Init();

    Handle(TDocStd_Document) aDoc;
    Handle(XCAFApp_Application) anApp = XCAFApp_Application::GetApplication();
    anApp->NewDocument("MDTV-XCAF", aDoc);

    STEPCAFControl_Reader aStepReader;
    aStepReader.SetColorMode(true);
    aStepReader.SetNameMode(true);
    aStepReader.SetLayerMode(true);
    aStepReader.SetPropsMode(true);

    IFSelect_ReturnStatus status = aStepReader.ReadFile (theStepName.c_str());
    if (status != IFSelect_RetDone) {
        qWarning() << "Failed to read step file:" << QString::fromStdString(theStepName);
        SegmentVec.pop_back(); // Remove the empty segment
        return false;
    }


    if (!aStepReader.Transfer (aDoc)) {
        qWarning() << "Failed to transfer step file:" << QString::fromStdString(theStepName);
        SegmentVec.pop_back(); // Remove the empty segment
        return false;
    }


    TDF_Label aRootLabel = aDoc->Main();

    aShapeTool = XCAFDoc_DocumentTool::ShapeTool(aRootLabel);
    aColorTool = XCAFDoc_DocumentTool::ColorTool(aRootLabel);

    Visit(aRootLabel);

    if(!m_view.IsNull()){
        m_view->FitAll();
    }


    return true;
}

void Panel_RobotView3D::Visit(const TDF_Label& theLabel)
{
    Quantity_Color aColor;
    TopoDS_Shape aShape;

    Handle(TDataStd_Name) aName;
    if (theLabel.FindAttribute(TDataStd_Name::GetID(), aName))
    {
        //std::cout << "  Name: " << aName->Get() << std::endl;
    }

    bool skip=0;
    if (aShapeTool->IsShape(theLabel))
    {
        if(aShapeTool->GetShape(theLabel, aShape)){
            if(aColorTool->GetColor(aShape,XCAFDoc_ColorSurf,aColor)){
                skip=0;
            } else { skip=1;}
        }

        if(skip==0){
            if (!SegmentVec.empty()) {
                SegmentVec.back().Ais_ShapeVec.push_back(new AIS_Shape(aShape));
                SegmentVec.back().Ais_ShapeVec.back()->SetColor(aColor);
                SegmentVec.back().Ais_ShapeVec.back()->SetDisplayMode(AIS_Shaded);
                SegmentVec.back().Ais_ShapeVec.back()->Attributes()->SetFaceBoundaryDraw(true);
                SegmentVec.back().Ais_ShapeVec.back()->Attributes()->SetFaceBoundaryAspect(
                    new Prs3d_LineAspect(Quantity_NOC_BLACK, Aspect_TOL_SOLID, 1.));
                SegmentVec.back().Ais_ShapeVec.back()->Attributes()->SetIsoOnTriangulation(true);
                if(!m_context.IsNull()){
                    m_context->Display(SegmentVec.back().Ais_ShapeVec.back(),Standard_False);
                }
            }
        }
    }

    /*! Repeat the visit function for each childmember. */
    for (TDF_ChildIterator c(theLabel); c.More(); c.Next())
    {
        Visit(c.Value());
    }
}

void Panel_RobotView3D::Init_robot(){
    // This method was declared but not defined in the original files.
    // Add implementation if needed, or remove if not used.
}

void Panel_RobotView3D::setup_tcp_origin(){
    //double toollenght=105; //conus
    double current_toollenght=0; //flange; // Using a local variable to avoid conflict with member

    TopoDS_Edge edge = BRepBuilderAPI_MakeEdge({525+current_toollenght,0,890},{525+current_toollenght+100,0,890});
    aisBody_tcp_xaxis = new AIS_Shape(edge);
    if(!m_context.IsNull()){
        m_context->SetColor(aisBody_tcp_xaxis,Quantity_NOC_RED,Standard_False);
        m_context->SetMaterial(aisBody_tcp_xaxis,Graphic3d_NOM_PLASTIC,Standard_False);
        m_context->Display(aisBody_tcp_xaxis,Standard_False);
    }


    edge= BRepBuilderAPI_MakeEdge({525+current_toollenght,0,890},{525+current_toollenght,0+100,890});
    aisBody_tcp_yaxis = new AIS_Shape(edge);
    if(!m_context.IsNull()){
        m_context->SetColor(aisBody_tcp_yaxis,Quantity_NOC_GREEN,Standard_False);
        m_context->SetMaterial(aisBody_tcp_yaxis,Graphic3d_NOM_PLASTIC,Standard_False);
        m_context->Display(aisBody_tcp_yaxis,Standard_False);
    }


    edge= BRepBuilderAPI_MakeEdge({525+current_toollenght,0,890},{525+current_toollenght,0,890+100});
    aisBody_tcp_zaxis = new AIS_Shape(edge);
    if(!m_context.IsNull()){
        m_context->SetColor(aisBody_tcp_zaxis,Quantity_NOC_BLUE,Standard_False);
        m_context->SetMaterial(aisBody_tcp_zaxis,Graphic3d_NOM_PLASTIC,Standard_False);
        m_context->Display(aisBody_tcp_zaxis,Standard_False);
    }

}

void Panel_RobotView3D::update_jointpos(RDT::AxisSet axis){

    double j0 = static_cast<double>(axis.at(0).angle.value()); // Assuming .value() returns double
    double j1 = static_cast<double>(axis.at(1).angle.value());
    double j2 = static_cast<double>(axis.at(2).angle.value());
    double j3 = static_cast<double>(axis.at(3).angle.value());
    double j4 = static_cast<double>(axis.at(4).angle.value());
    double j5 = static_cast<double>(axis.at(5).angle.value());

    double DH_param[7]={25, -35, 0, 400, 455, 420, 80};

    double a1 = DH_param[0];
    double a2 = DH_param[1];
    double d1 = DH_param[3];
    double d2 = DH_param[4];
    double d3 = DH_param[5];
    double d4 = DH_param[6];


    if(SegmentVec.size() > 1)
        SegmentVec.at(1).MyTrsf.SetRotation(gp_Ax1(gp_Pnt(
                                                       0,           // Framevector  x
                                                       0,           //              y
                                                       0), gp_Dir(  //              z
                                                       0,           // Rotationflag x
                                                       0,           //              y
                                                       1)), j0);
    if(SegmentVec.size() > 2)
        SegmentVec.at(2).MyTrsf.SetRotation(gp_Ax1(gp_Pnt(
                                                       a1,          // 25
                                                       0,
                                                       d1), gp_Dir( // 400
                                                       0,
                                                       1,
                                                       0)), j1);
    if(SegmentVec.size() > 3)
        SegmentVec.at(3).MyTrsf.SetRotation(gp_Ax1(gp_Pnt(
                                                       a1,          // 25
                                                       0,
                                                       d1 + d2), gp_Dir( // 855
                                                       0,
                                                       1,
                                                       0)), j2);
    if(SegmentVec.size() > 4)
        SegmentVec.at(4).MyTrsf.SetRotation(gp_Ax1(gp_Pnt(
                                                       a1, // 25
                                                       0,
                                                       d1 + d2 - a2), gp_Dir( // 890
                                                       1,
                                                       0,
                                                       0)), j3);
    if(SegmentVec.size() > 5)
        SegmentVec.at(5).MyTrsf.SetRotation(gp_Ax1(gp_Pnt(
                                                       d2, //445
                                                       0,
                                                       d1 + d2 - a2), gp_Dir( // 890
                                                       0,
                                                       1,
                                                       0)), j4);
    if(SegmentVec.size() > 6)
        // Robot mount flange axis 6.
        SegmentVec.at(6).MyTrsf.SetRotation(gp_Ax1(gp_Pnt(
                                                       a1 + d3 +d4, // 525
                                                       0,
                                                       d1 + d2 - a2), gp_Dir( // 890
                                                       1,
                                                       0,
                                                       0)), j5);


    gp_Trsf level0 = (SegmentVec.size() > 0) ? SegmentVec.at(0).MyTrsf : gp_Trsf();
    gp_Trsf level1 = (SegmentVec.size() > 1) ? SegmentVec.at(1).MyTrsf : gp_Trsf();
    gp_Trsf level2 = (SegmentVec.size() > 2) ? SegmentVec.at(2).MyTrsf : gp_Trsf();
    gp_Trsf level3 = (SegmentVec.size() > 3) ? SegmentVec.at(3).MyTrsf : gp_Trsf();
    gp_Trsf level4 = (SegmentVec.size() > 4) ? SegmentVec.at(4).MyTrsf : gp_Trsf();
    gp_Trsf level5 = (SegmentVec.size() > 5) ? SegmentVec.at(5).MyTrsf : gp_Trsf();
    gp_Trsf level6 = (SegmentVec.size() > 6) ? SegmentVec.at(6).MyTrsf : gp_Trsf();


    gp_Trsf level0x1;
    level0x1.Multiply(level0);
    level0x1.Multiply(level1);

    gp_Trsf level0x1x2 = level0x1;
    level0x1x2.Multiply(level2);

    gp_Trsf level0x1x2x3 = level0x1x2;
    level0x1x2x3.Multiply(level3);

    gp_Trsf level0x1x2x3x4 = level0x1x2x3;
    level0x1x2x3x4.Multiply(level4);

    gp_Trsf level0x1x2x3x4x5 = level0x1x2x3x4;
    level0x1x2x3x4x5.Multiply(level5);

    level0x1x2x3x4x5x6 = level0x1x2x3x4x5;
    level0x1x2x3x4x5x6.Multiply(level6);

    if (!m_context.IsNull()) {
        // Apply multipied transformation, works if each stepfile represents one joint.
        for(unsigned int i=0; i<SegmentVec.size(); ++i){
            gp_Trsf current_trsf;
            if (i == 0) current_trsf = level0;
            else if (i == 1) current_trsf = level0x1;
            else if (i == 2) current_trsf = level0x1x2;
            else if (i == 3) current_trsf = level0x1x2x3;
            else if (i == 4) current_trsf = level0x1x2x3x4;
            else if (i == 5) current_trsf = level0x1x2x3x4x5;
            else if (i == 6) current_trsf = level0x1x2x3x4x5x6;

            for(unsigned int j=0; j<SegmentVec.at(i).Ais_ShapeVec.size(); j++){
                if(!SegmentVec.at(i).Ais_ShapeVec.at(j).IsNull()){
                    m_context->SetLocation(SegmentVec.at(i).Ais_ShapeVec.at(j), current_trsf);
                }
            }
        }

        // Move x-axis tcp origin along with the machine.
        if(!aisBody_tcp_xaxis.IsNull()) m_context->SetLocation(aisBody_tcp_xaxis, level0x1x2x3x4x5x6);
        if(!aisBody_tcp_yaxis.IsNull()) m_context->SetLocation(aisBody_tcp_yaxis, level0x1x2x3x4x5x6);
        if(!aisBody_tcp_zaxis.IsNull()) m_context->SetLocation(aisBody_tcp_zaxis, level0x1x2x3x4x5x6);


        // Look for mouse selections.
        get_selections();

        m_context->CurrentViewer()->Redraw();
    }
}

void Panel_RobotView3D::show_shape(Handle(AIS_Shape) shape){
    if(!m_context.IsNull()){
        m_context->Display(shape,Standard_False);
    }
}

void Panel_RobotView3D::draw_preview_cone(std::string type, gp_Trsf trsf){

    double current_toollenght=0; // Using a local variable

    gp_Pnt point={525+current_toollenght-coneheight,0,890};
    gp_Dir xDirection(1,0,0); // Direction is auto normalized by occ.
    gp_Dir normalDirection(0,0,1);
    gp_Ax2 aplace(point,normalDirection,xDirection);

    TopoDS_Shape t_topo_cone = BRepPrimAPI_MakeCone(aplace,conebottomdiameter,conetopdiameter,coneheight).Shape();

    // Draw toolpos cone at startpoint
    Handle(AIS_Shape) cone_shape = new AIS_Shape(t_topo_cone);
    previewbucketvec.push_back(cone_shape);


    gp_Trsf MyTrsf_Local_Rot;
    MyTrsf_Local_Rot.SetRotation(gp_Ax1(gp_Pnt(
                                            525+current_toollenght-coneheight,
                                            0,
                                            890), gp_Dir(
                                            0,                         //rotation flag x
                                            1,                         //rotation flag y
                                            0)), 90 * toRad);

    if(!previewbucketvec.back().IsNull()){
        previewbucketvec.back()->SetLocalTransformation(trsf*MyTrsf_Local_Rot);

        if(type=="startpoint"){
            m_context->SetColor(previewbucketvec.back(),Quantity_NOC_GREEN,Standard_False);
        }
        if(type=="io"){
            m_context->SetColor(previewbucketvec.back(),Quantity_NOC_RED,Standard_False);
        }
        if(type=="waypoint"){
            m_context->SetColor(previewbucketvec.back(),Quantity_NOC_BLUE,Standard_False);
        }
        if(type=="endpoint"){
            m_context->SetColor(previewbucketvec.back(),Quantity_NOC_BLACK,Standard_False);
        }

        m_context->SetTransparency(previewbucketvec.back(),0.5,false);
        m_context->SetMaterial(previewbucketvec.back(),Graphic3d_NOM_PLASTIC,Standard_False);
        m_context->Display(previewbucketvec.back(),Standard_False);
    }

}

void Panel_RobotView3D::empty_preview_bucket(){
    if(!m_context.IsNull()){
        for(unsigned int i=0; i<previewbucketvec.size(); i++){
            if(!previewbucketvec.at(i).IsNull()){
                m_context->Remove(previewbucketvec.at(i), Standard_False);
            }
        }
        previewbucketvec.clear();
        m_context->CurrentViewer()->Redraw();
    }
}

void Panel_RobotView3D::get_selections(){ // Updated by jointpos function from mainwindow.
    if(!m_context.IsNull()){
        for(m_context->InitSelected(); m_context->MoreSelected(); m_context->NextSelected()){

            const TopoDS_Shape& aSelShape = m_context->SelectedShape();
            std::cout<<"selected shape type:"<<aSelShape.ShapeType()<<std::endl;
            // Shapetype 6=line.

            for(unsigned int i=0; i<bucketvec.size(); i++){
                if(!bucketvec.at(i).Ais_shape.IsNull() && m_context->SelectedShape()==bucketvec.at(i).Ais_shape->Shape()){
                    std::cout<<"match found at Ais_bucket i:"<<i<<std::endl;

                    // Print some extra content:
                    std::cout<<"primitivetype:"<<bucketvec.at(i).primitivetype<<std::endl;

                    // Print euler content:
                    for(unsigned int j=0; j<bucketvec.at(i).eulervec.size(); j++){
                        std::cout<<"index:"<<j<<"euler_z:"<<bucketvec.at(i).eulervec.at(j).z<<std::endl;
                        std::cout<<"index:"<<j<<"euler_y:"<<bucketvec.at(i).eulervec.at(j).y<<std::endl;
                        std::cout<<"index:"<<j<<"euler_x:"<<bucketvec.at(i).eulervec.at(j).x<<std::endl;
                    }
                }
            }

            TopExp_Explorer explorer;

            for(explorer.Init(aSelShape, TopAbs_EDGE); explorer.More(); explorer.Next()){

                const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());

                TopoDS_Vertex v1,v2;
                TopExp::Vertices(edge,v1,v2);
                gp_Pnt p1= BRep_Tool::Pnt(v1);
                gp_Pnt p2= BRep_Tool::Pnt(v2);

                std::cout<<"edge p1 x: "<<p1.X()<<" y:"<<p1.Y()<<" z:"<<p1.Z()<<std::endl;
                std::cout<<"edge p2 x: "<<p2.X()<<" y:"<<p2.Y()<<" z:"<<p2.Z()<<std::endl;

                std::cout<<"NEXT-SHAPE"<<std::endl;
            }
        }
    }
}

void Panel_RobotView3D::delete_selections(){
    if(!m_context.IsNull()){
        std::cout<<"bucketvecsize before:"<<bucketvec.size()<<std::endl;
        std::vector<int> indices_to_remove;
        for(m_context->InitSelected(); m_context->MoreSelected(); m_context->NextSelected()){
            Handle(AIS_InteractiveObject) selected_obj = m_context->SelectedInteractive();
            if(!selected_obj.IsNull()){
                // Find the match of selected item in the Ais_databucket.
                for(unsigned int i=0; i<bucketvec.size(); i++){
                    if(!bucketvec.at(i).Ais_shape.IsNull() && bucketvec.at(i).Ais_shape == selected_obj){
                        indices_to_remove.push_back(i);
                    }
                }
            }
        }

        // Sort in descending order to avoid index issues when erasing
        std::sort(indices_to_remove.rbegin(), indices_to_remove.rend());

        for(int index : indices_to_remove){
            if(!bucketvec.at(index).Ais_shape.IsNull()){
                m_context->Remove(bucketvec.at(index).Ais_shape, Standard_False);
            }
            bucketvec.erase(bucketvec.begin()+index);
        }

        m_context->CurrentViewer()->Redraw();
        std::cout<<"bucketvecsize after:"<<bucketvec.size()<<std::endl;
    }
}

void Panel_RobotView3D::Redraw(){
    if(!m_view.IsNull()){
        m_view->Redraw();
    }
}

void Panel_RobotView3D::m_initialize_context()
{
    if (m_context.IsNull())
    {
        Handle(Aspect_DisplayConnection) m_display_donnection = new Aspect_DisplayConnection();

        if (m_graphic_driver.IsNull())
        {
            m_graphic_driver = new OpenGl_GraphicDriver(m_display_donnection);
        }

        WId window_handle = (WId) winId();
#ifdef _WIN32
        Handle(WNT_Window) wind = new WNT_Window((Aspect_Handle) window_handle);
#else
        Handle(Xw_Window) wind = new Xw_Window(m_display_donnection, (Window) window_handle);
#endif
        m_viewer = new V3d_Viewer(m_graphic_driver);

        m_view = m_viewer->CreateView();
        m_view->SetWindow(wind);

        /*! Set the camera in perspective mode */
        m_view->Camera()->SetProjectionType (Graphic3d_Camera::Projection_Perspective);

        if (!wind->IsMapped())
        {
            wind->Map();
        }
        m_context = new AIS_InteractiveContext(m_viewer);

        m_viewer->SetDefaultLights();
        m_viewer->SetLightOn();

        /// This is the interactivve 3d box
        //  View : top, bottom, side, 3d, etc.

        Handle(AIS_ViewCube) aisViewCube = new AIS_ViewCube;
        aisViewCube->SetBoxColor(Quantity_NOC_GRAY75);
        //aisViewCube->SetFixedAnimationLoop(false);
        aisViewCube->SetDrawAxes(false);
        aisViewCube->SetSize(55);
        aisViewCube->SetFontHeight(12);
        aisViewCube->SetTransformPersistence(
            new Graphic3d_TransformPers(
                Graphic3d_TMF_TriedronPers,
                Aspect_TOTP_LEFT_UPPER,
                Graphic3d_Vec2i(85, 85)));
        m_context->Display(aisViewCube, false);
        m_aisViewCube = aisViewCube; // Assign to member variable


        /// Set background with gradient stylesheet
        Quantity_Color cola,colb;
        cola.SetValues(0.3,0.3,0.3,Quantity_TOC_RGB);
        colb.SetValues(0.6,0.6,0.6,Quantity_TOC_RGB);
        m_view->SetBgGradientColors(cola,colb,Aspect_GFM_DIAG2 , false);

        /// View as wireframe or shaded
        m_context->SetDisplayMode(AIS_Shaded, Standard_False);

        Handle(Prs3d_Drawer) t_hilight_style = m_context->HighlightStyle();
        t_hilight_style->SetMethod(Aspect_TOHM_COLOR);
        t_hilight_style->SetColor(Quantity_NOC_LIGHTYELLOW);
        t_hilight_style->SetDisplayMode(1);
        t_hilight_style->SetTransparency(0.2f);

        Handle(Prs3d_Drawer) t_select_style = m_context->SelectionStyle();
        t_select_style->SetMethod(Aspect_TOHM_COLOR);
        t_select_style->SetColor(Quantity_NOC_LIGHTSEAGREEN);
        t_select_style->SetDisplayMode(1);
        t_select_style->SetTransparency(0.4f);

        /// Show grid
        m_viewer->SetRectangularGridValues(0,0,1,1,0);
        m_viewer->SetRectangularGridGraphicValues(2.01,2.01,0);
        m_viewer->ActivateGrid(Aspect_GT_Rectangular,Aspect_GDM_Lines);

        /// Show triedron. This is the 3d axis cross at the lower left of the screen.
        m_view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_GOLD, 0.08, V3d_ZBUFFER);

        m_view->MustBeResized();
    }
}

void Panel_RobotView3D::Set_orthographic(){
    if(!m_view.IsNull()){
        m_view->Camera()->SetProjectionType (Graphic3d_Camera::Projection_Orthographic);
        m_view->Update();
    }
}

void Panel_RobotView3D::Set_perspective(){
    if(!m_view.IsNull()){
        m_view->Camera()->SetProjectionType (Graphic3d_Camera::Projection_Perspective);
        m_view->Redraw();
    }
}

void Panel_RobotView3D::paintEvent(QPaintEvent *)
{
    if (m_context.IsNull())
    {
        m_initialize_context();
    }
    if(!m_view.IsNull()){
        m_view->Redraw();
    }
}

void Panel_RobotView3D::resizeEvent(QResizeEvent *)
{
    if( !m_view.IsNull() )
    {
        m_view->MustBeResized();
    }
}

void Panel_RobotView3D::mousePressEvent(QMouseEvent *event)
{
    if((event->buttons()&Qt::LeftButton) && (event->buttons()&Qt::RightButton))
    {
        m_x_max=event->x();
        m_y_max=event->y();
    }
    else if(event->buttons()&Qt::LeftButton)
    {
        if(!m_context.IsNull() && !m_view.IsNull()){
            m_context->MoveTo(event->pos().x(),event->pos().y(),m_view,Standard_True);

            AIS_StatusOfPick t_pick_status = AIS_SOP_NothingSelected;
            if(qApp->keyboardModifiers()==Qt::ControlModifier)
            {
                t_pick_status = m_context->ShiftSelect(true);
            }
            else
            {
                t_pick_status = m_context->Select(true);
            }
        }

    }
    else if(event->buttons()&Qt::MiddleButton)
    {
        m_x_max=event->x();
        m_y_max=event->y();
        if(!m_view.IsNull()){
            m_view->StartRotation(event->x(),event->y());
        }

    }
}

void Panel_RobotView3D::mouseReleaseEvent(QMouseEvent *event)
{
    if(!m_context.IsNull() && !m_view.IsNull()){
        m_context->MoveTo(event->pos().x(),event->pos().y(),m_view,Standard_True);
    }
}

void Panel_RobotView3D::mouseMoveEvent(QMouseEvent *event)
{
    if((event->buttons()&Qt::LeftButton) && (event->buttons()&Qt::RightButton))
    {
        if(!m_view.IsNull()){
            m_view->Pan(event->pos().x()-m_x_max,m_y_max-event->pos().y());
            m_x_max=event->x();
            m_y_max=event->y();
        }

    }
    else if(event->buttons()&Qt::MiddleButton)
    {
        if(qApp->keyboardModifiers()==Qt::ShiftModifier)
        {
            if(!m_view.IsNull()){
                m_view->Pan(event->pos().x()-m_x_max,m_y_max-event->pos().y());
                m_x_max=event->x();
                m_y_max=event->y();
            }
        }
        else
        {
            if(!m_view.IsNull()){
                m_view->Rotation(event->x(),event->y());
            }
        }
    }
    else
    {
        if(!m_context.IsNull() && !m_view.IsNull()){
            m_context->MoveTo(event->pos().x(),event->pos().y(),m_view,Standard_True);
        }
    }
}

void Panel_RobotView3D::wheelEvent(QWheelEvent *event)
{
    if(!m_view.IsNull()){
        m_view->StartZoomAtPoint(event->position().x(),event->position().y());
        m_view->ZoomAtPoint(0, 0, event->angleDelta().y(), 0);
    }
}

void Panel_RobotView3D::set_view_front()
{
    if(!m_view.IsNull()){
        m_view->SetProj( V3d_Yneg );
        m_view->FitAll();
    }
}

void Panel_RobotView3D::set_view_back()
{
    if(!m_view.IsNull()){
        m_view->SetProj( V3d_Ypos );
        m_view->FitAll();
    }
}

void Panel_RobotView3D::set_view_top()
{
    if(!m_view.IsNull()){
        m_view->SetProj( V3d_Zpos );
        m_view->FitAll();
    }
}

void Panel_RobotView3D::set_view_bottom()
{
    if(!m_view.IsNull()){
        m_view->SetProj( V3d_Zneg );
        m_view->FitAll();
    }
}

void Panel_RobotView3D::set_view_left()
{
    if(!m_view.IsNull()){
        m_view->SetProj( V3d_Xneg );
        m_view->FitAll();
    }
}
void Panel_RobotView3D::set_view_right()
{
    if(!m_view.IsNull()){
        m_view->SetProj( V3d_Xpos );
        m_view->FitAll();
    }
}

void Panel_RobotView3D::draw_line_store_into_bucketvec(bucket b){

    bucketvec.push_back(b); // Create a new place at the end of the bucketvector.

    if(bucketvec.back().pointvec.size() >= 2){
        TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(bucketvec.back().pointvec.front(),bucketvec.back().pointvec.back()); // Create a edge from point1 & point2.
        bucketvec.back().Ais_shape=new AIS_Shape(edge); // Create a interactive shape and save it.

        if(!m_context.IsNull()){
            // Show the line primitive.
            m_context->SetColor(bucketvec.back().Ais_shape,Quantity_NOC_BLACK,Standard_False);
            m_context->SetMaterial(bucketvec.back().Ais_shape,Graphic3d_NOM_PLASTIC,Standard_False);
            m_context->Display(bucketvec.back().Ais_shape,Standard_False);
        }

        // Ensure eulervec_gp_Trsf is populated
        if (bucketvec.back().eulervec_gp_Trsf.empty() && !bucketvec.back().eulervec.empty()) {
            for(const auto& euler : bucketvec.back().eulervec) {
                gp_Trsf trsf;
                // Assuming euler angles are Z(Yaw), Y(Pitch), X(Roll)
                gp_Trsf rotationZ, rotationY, rotationX;
                rotationZ.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,0,1)), euler.z * toRad);
                rotationY.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,1,0)), euler.y * toRad);
                rotationX.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(1,0,0)), euler.x * toRad);

                trsf = rotationX;      // Start with the last rotation
                trsf.Multiply(rotationY); // trsf becomes Rx * Ry
                trsf.Multiply(rotationZ);

                //trsf = rotationZ;
                //trsf.Multiply(rotationY); // trsf = Rz * Ry
                //trsf.Multiply(rotationX); // trsf = Rz * Ry * Rx
                size_t euler_index = &euler - &bucketvec.back().eulervec[0];
                if(euler_index < bucketvec.back().pointvec.size()){
                    gp_Vec translation(bucketvec.back().pointvec.at(euler_index).X(),
                                       bucketvec.back().pointvec.at(euler_index).Y(),
                                       bucketvec.back().pointvec.at(euler_index).Z());
                    trsf.SetTranslationPart(translation);
                }
                bucketvec.back().eulervec_gp_Trsf.push_back(trsf);
            }
        }

        if (bucketvec.back().eulervec_gp_Trsf.size() >= 2 && !bucketvec.back().Ais_shape.IsNull()) {
            double current_toollenght = 0; // Using a local variable

            // First euler origin.
            TopoDS_Edge edge_euler_begin_x = BRepBuilderAPI_MakeEdge({525+current_toollenght,0,890},{525+current_toollenght+linelenght,0,890});
            Handle(AIS_Shape) aisBody_euler_begin_x_axis = new AIS_Shape(edge_euler_begin_x);
            aisBody_euler_begin_x_axis->SetLocalTransformation(bucketvec.back().eulervec_gp_Trsf.front());
            bucketvec.back().Ais_shape->AddChild(aisBody_euler_begin_x_axis);
            if(!m_context.IsNull()){
                m_context->SetColor(aisBody_euler_begin_x_axis,Quantity_NOC_RED,Standard_False);
                m_context->SetMaterial(aisBody_euler_begin_x_axis,Graphic3d_NOM_PLASTIC,Standard_False);
                m_context->Display(aisBody_euler_begin_x_axis,Standard_False);
            }


            TopoDS_Edge edge_euler_begin_y = BRepBuilderAPI_MakeEdge({525+current_toollenght,0,890},{525+current_toollenght,0+linelenght,890});
            Handle(AIS_Shape) aisBody_euler_begin_y_axis = new AIS_Shape(edge_euler_begin_y);
            aisBody_euler_begin_y_axis->SetLocalTransformation(bucketvec.back().eulervec_gp_Trsf.front());
            bucketvec.back().Ais_shape->AddChild(aisBody_euler_begin_y_axis);
            if(!m_context.IsNull()){
                m_context->SetColor(aisBody_euler_begin_y_axis,Quantity_NOC_GREEN,Standard_False);
                m_context->SetMaterial(aisBody_euler_begin_y_axis,Graphic3d_NOM_PLASTIC,Standard_False);
                m_context->Display(aisBody_euler_begin_y_axis,Standard_False);
            }


            TopoDS_Edge edge_euler_begin_z = BRepBuilderAPI_MakeEdge({525+current_toollenght,0,890},{525+current_toollenght,0,890+linelenght});
            Handle(AIS_Shape) aisBody_euler_begin_z_axis = new AIS_Shape(edge_euler_begin_z);
            aisBody_euler_begin_z_axis->SetLocalTransformation(bucketvec.back().eulervec_gp_Trsf.front());
            bucketvec.back().Ais_shape->AddChild(aisBody_euler_begin_z_axis);
            if(!m_context.IsNull()){
                m_context->SetColor(aisBody_euler_begin_z_axis,Quantity_NOC_BLUE,Standard_False);
                m_context->SetMaterial(aisBody_euler_begin_z_axis,Graphic3d_NOM_PLASTIC,Standard_False);
                m_context->Display(aisBody_euler_begin_z_axis,Standard_False);
            }


            // Second euler origin.
            TopoDS_Edge edge_euler_end_x = BRepBuilderAPI_MakeEdge({525+current_toollenght,0,890},{525+current_toollenght+linelenght,0,890});
            Handle(AIS_Shape) aisBody_euler_end_x_axis = new AIS_Shape(edge_euler_end_x);
            aisBody_euler_end_x_axis->SetLocalTransformation(bucketvec.back().eulervec_gp_Trsf.back());
            bucketvec.back().Ais_shape->AddChild(aisBody_euler_end_x_axis);
            if(!m_context.IsNull()){
                m_context->SetColor(aisBody_euler_end_x_axis,Quantity_NOC_RED,Standard_False);
                m_context->SetMaterial(aisBody_euler_end_x_axis,Graphic3d_NOM_PLASTIC,Standard_False);
                m_context->Display(aisBody_euler_end_x_axis,Standard_False);
            }


            TopoDS_Edge edge_euler_end_y = BRepBuilderAPI_MakeEdge({525+current_toollenght,0,890},{525+current_toollenght,0+linelenght,890});
            Handle(AIS_Shape) aisBody_euler_end_y_axis = new AIS_Shape(edge_euler_end_y);
            aisBody_euler_end_y_axis->SetLocalTransformation(bucketvec.back().eulervec_gp_Trsf.back());
            bucketvec.back().Ais_shape->AddChild(aisBody_euler_end_y_axis);
            if(!m_context.IsNull()){
                m_context->SetColor(aisBody_euler_end_y_axis,Quantity_NOC_GREEN,Standard_False);
                m_context->SetMaterial(aisBody_euler_end_y_axis,Graphic3d_NOM_PLASTIC,Standard_False);
                m_context->Display(aisBody_euler_end_y_axis,Standard_False);
            }

            TopoDS_Edge edge_euler_end_z = BRepBuilderAPI_MakeEdge({525+current_toollenght,0,890},{525+current_toollenght,0,890+linelenght});
            Handle(AIS_Shape) aisBody_euler_end_z_axis = new AIS_Shape(edge_euler_end_z);
            aisBody_euler_end_z_axis->SetLocalTransformation(bucketvec.back().eulervec_gp_Trsf.back());
            bucketvec.back().Ais_shape->AddChild(aisBody_euler_end_z_axis);
            if(!m_context.IsNull()){
                m_context->SetColor(aisBody_euler_end_z_axis,Quantity_NOC_BLUE,Standard_False);
                m_context->SetMaterial(aisBody_euler_end_z_axis,Graphic3d_NOM_PLASTIC,Standard_False);
                m_context->Display(aisBody_euler_end_z_axis,Standard_False);
            }


            // Draw toolpos cone at startpoint.
            gp_Pnt point={525+current_toollenght-coneheight,0,890};
            gp_Dir xDirection(1,0,0); // Direction is auto normalized by occ.
            gp_Dir normalDirection(0,0,1);
            gp_Ax2 aplace(point,normalDirection,xDirection);

            TopoDS_Shape t_topo_cone = BRepPrimAPI_MakeCone(aplace,conebottomdiameter,conetopdiameter,coneheight).Shape();

            Handle(AIS_Shape) t_ais_cone_start = new AIS_Shape(t_topo_cone);

            gp_Trsf MyTrsf_Local_Rot;
            MyTrsf_Local_Rot.SetRotation(gp_Ax1(gp_Pnt(
                                                    525+current_toollenght-coneheight,
                                                    0,
                                                    890), gp_Dir(
                                                    0,                         //rotation flag x
                                                    1,                         //rotation flag y
                                                    0)), 90 * toRad);

            t_ais_cone_start->SetLocalTransformation(bucketvec.back().eulervec_gp_Trsf.front()*MyTrsf_Local_Rot);
            bucketvec.back().Ais_shape->AddChild(t_ais_cone_start);

            if(!m_context.IsNull()){
                m_context->SetColor(t_ais_cone_start,Quantity_NOC_GREEN,Standard_False);
                m_context->SetTransparency(t_ais_cone_start,0.5,false);
                m_context->SetMaterial(t_ais_cone_start,Graphic3d_NOM_PLASTIC,Standard_False);
                m_context->Display(t_ais_cone_start,Standard_False);
            }


            // Draw toolpos cone at endpoint.
            Handle(AIS_Shape) t_ais_cone_endpoint = new AIS_Shape(t_topo_cone);
            t_ais_cone_endpoint->SetLocalTransformation(bucketvec.back().eulervec_gp_Trsf.back()*MyTrsf_Local_Rot);
            bucketvec.back().Ais_shape->AddChild(t_ais_cone_endpoint);

            if(!m_context.IsNull()){
                m_context->SetColor(t_ais_cone_endpoint,Quantity_NOC_BLACK,Standard_False);
                m_context->SetTransparency(t_ais_cone_endpoint,0.5,false);
                m_context->SetMaterial(t_ais_cone_endpoint,Graphic3d_NOM_PLASTIC,Standard_False);
                m_context->Display(t_ais_cone_endpoint,Standard_False);
            }


            // Draw io status cone's.
            for(unsigned int i=1; i<bucketvec.back().eulervec_gp_Trsf.size()-1; i++){

                Handle(AIS_Shape) t_ais_cone_io = new AIS_Shape(t_topo_cone);

                t_ais_cone_io->SetLocalTransformation(bucketvec.back().eulervec_gp_Trsf.at(i)*MyTrsf_Local_Rot);
                bucketvec.back().Ais_shape->AddChild(t_ais_cone_io);

                if(!m_context.IsNull()){
                    m_context->SetColor(t_ais_cone_io,Quantity_NOC_RED,Standard_False);
                    m_context->SetTransparency(t_ais_cone_io,0.5,false);
                    m_context->SetMaterial(t_ais_cone_io,Graphic3d_NOM_PLASTIC,Standard_False);
                    m_context->Display(t_ais_cone_io,Standard_False);
                }

                // Draw io text.
                if (i - 1 < bucketvec.back().iovec.size()) {
                    std::stringstream ss;
                    ss<<bucketvec.back().iovec.at(i-1).halcommand; // Assuming iovec is 0-indexed for points 1 to size-2
                    // Add boolvalue if available in io struct
                    // ss<<" :";
                    // ss<<bucketvec.back().iovec.at(i-1).boolvalue;


                    std::string text=ss.str();
                    const char *chartext=text.c_str();

                    double current_textheight=25; // Using a local variable
                    if(current_textheight==0 || current_textheight<0){current_textheight=1;}

                }
            }

            // Draw text line info.
            std::string id=QString::number(bucketvec.size()).toStdString();
            std::string text="line-id:"+id;
            const char *chartext=text.c_str();

            double current_textheight=25; // Using a local variable
            if(current_textheight==0 || current_textheight<0){current_textheight=1;}


        }
    }
}

bool Panel_RobotView3D::loadmodel()
{
    SegmentVec.clear(); // Clear previous segments

    bool ok=Readstepfile("/home/ui/Desktop/Robot_UI/RobotControl_MVP/1_RobotControl_main/kuka_r900/kuka_base.step");
    if(!ok){ qWarning() << "Failed to load kuka_base.step"; return false;}
    ok=Readstepfile("/home/ui/Desktop/Robot_UI/RobotControl_MVP/1_RobotControl_main/kuka_r900/kuka_joint_1.step");
    if(!ok){ qWarning() << "Failed to load Link1.STEP"; return false;}
    ok=Readstepfile("/home/ui/Desktop/Robot_UI/RobotControl_MVP/1_RobotControl_main/kuka_r900/kuka_joint_2.step");
    if(!ok){ qWarning() << "Failed to load Link2.STEP"; return false;}
    ok=Readstepfile("/home/ui/Desktop/Robot_UI/RobotControl_MVP/1_RobotControl_main/kuka_r900/kuka_joint_3.step");
    if(!ok){ qWarning() << "Failed to load Link3.STEP"; return false;}
    ok=Readstepfile("/home/ui/Desktop/Robot_UI/RobotControl_MVP/1_RobotControl_main/kuka_r900/kuka_joint_4.step");
    if(!ok){ qWarning() << "Failed to load Link4.STEP"; return false;}
    ok=Readstepfile("/home/ui/Desktop/Robot_UI/RobotControl_MVP/1_RobotControl_main/kuka_r900/kuka_joint_5.step");
    if(!ok){ qWarning() << "Failed to load Link5.STEP"; return false;}
   // ok=Readstepfile("/home/ui/Desktop/Robot_UI/RobotControl_MVP/1_RobotControl_main/kuka_r900/kuka_joint_6.step");
   // if(!ok){ qWarning() << "Failed to load Link6.STEP"; return false;}

    qDebug()<<"Robot model loaded successfully.";

    setup_tcp_origin();
    //Redraw();
    return true;
}

void Panel_RobotView3D::mainloop()
{
    if(!ready){
        bool ok=loadmodel();
        if(ok){
            ready=1;
            // Initial pose if needed
                RDT::AxisSet initial_pose;
                std::array<RDT::Radians, RDT::ROBOT_AXES_COUNT> initial_angles = {
                    RDT::Radians(0.0),
                    RDT::Radians(1.5708),
                    RDT::Radians(0.0),
                    RDT::Radians(0.0),
                    RDT::Radians(0.0),
                    RDT::Radians(0.0)
                };
                initial_pose.fromAngleArray(initial_angles);
                update_jointpos(initial_pose);
        }
    }

    // Add other mainloop logic here if necessary, e.g., animation updates
};
