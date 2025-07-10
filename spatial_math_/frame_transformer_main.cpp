// frame_transformer_main.cpp
#include "FrameTransformer.h" // RDT::FrameTransformer (header-only)
#include "DataTypes.h"        // RDT::CartPose, RDT::ToolFrame, RDT::BaseFrame
#include "Units.h"            // RDT::Meters, RDT::Radians, RDT::literals, RDT::UnitConstants
#include "Logger.h"           // RDT::Logger

#include <iostream>
#include <iomanip>   // For std::fixed, std::setprecision
#include <cmath>     // For std::abs in comparisons, M_PI (though we use UnitConstants::PI)
#include <stdexcept> // For try-catch (if any methods could throw)

// For convenience
using namespace RDT;
using namespace RDT::literals;

// Helper function to compare two CartPoses with a tolerance
bool arePosesApproximatelyEqual(const CartPose& p1, const CartPose& p2,
                                Meters pos_tol = 0.00001_m,
                                Radians ang_tol = (0.001_deg).toRadians()) {
    bool pos_equal = ((p1.x - p2.x).abs() <= pos_tol) &&
                     ((p1.y - p2.y).abs() <= pos_tol) &&
                     ((p1.z - p2.z).abs() <= pos_tol);

    // For angles, it's better to compare the shortest angle between them.
    // (a - b).normalized() handles wrapping around PI.
    bool ang_equal = ((p1.rx - p2.rx).normalized().abs() <= ang_tol) &&
                     ((p1.ry - p2.ry).normalized().abs() <= ang_tol) &&
                     ((p1.rz - p2.rz).normalized().abs() <= ang_tol);

    if (!(pos_equal && ang_equal)) {
        LOG_DEBUG_F("PoseCompare", "Poses differ significantly:");
        LOG_DEBUG_F("PoseCompare", "  P1: %s", p1.toDescriptiveString().c_str());
        LOG_DEBUG_F("PoseCompare", "  P2: %s", p2.toDescriptiveString().c_str());
        if (!pos_equal) {
            LOG_DEBUG_F("PoseCompare", "  Pos Diff: dx=%s, dy=%s, dz=%s",
                        (p1.x - p2.x).abs().toString(7).c_str(), // Increased precision for diff
                        (p1.y - p2.y).abs().toString(7).c_str(),
                        (p1.z - p2.z).abs().toString(7).c_str());
        }
        if (!ang_equal) {
             LOG_DEBUG_F("PoseCompare", "  Ang Diff (abs norm): drx=%s, dry=%s, drz=%s",
                        (p1.rx - p2.rx).normalized().abs().toDegrees().toString(5).c_str(), // Increased precision
                        (p1.ry - p2.ry).normalized().abs().toDegrees().toString(5).c_str(),
                        (p1.rz - p2.rz).normalized().abs().toDegrees().toString(5).c_str());
        }
        return false;
    }
    return true;
}

void testBasicPointTransforms() {
    LOG_INFO("FrameTransTest", "--- Testing Basic Point Transforms (Local <-> World) ---");

    CartPose point_in_local_A = {1.0_m, 0.5_m, 0.2_m, 0.0_rad, 0.0_rad, 0.0_rad};
    CartPose frameA_in_world  = {0.1_m, 0.2_m, 0.3_m, 0.0_rad, 0.0_rad, (90.0_deg).toRadians()};

    LOG_DEBUG_F("FrameTransTest", "Point in Local A: %s", point_in_local_A.toDescriptiveString().c_str());
    LOG_DEBUG_F("FrameTransTest", "Frame A in World: %s", frameA_in_world.toDescriptiveString().c_str());

    CartPose point_in_world = FrameTransformer::transformPoseToWorld(point_in_local_A, frameA_in_world);
    LOG_INFO_F("FrameTransTest", "Point_local_A to World: %s", point_in_world.toDescriptiveString().c_str());
    
    // Manual calculation for T_world_local * P_local
    // T_world_local: Translate(0.1, 0.2, 0.3) * RotateZ(90deg)
    // P_local: Translate(1.0, 0.5, 0.2)
    // Rotated P_local' (coords of P_local in frameA axes expressed in World axes if frameA origin was World origin):
    // x' = 1.0*cos(90) - 0.5*sin(90) = -0.5
    // y' = 1.0*sin(90) + 0.5*cos(90) =  1.0
    // z' = 0.2
    // P_world.translation = frameA_in_world.translation + Rot(frameA_in_world) * P_local.translation
    // P_world.x = 0.1 + (-0.5) = -0.4
    // P_world.y = 0.2 + ( 1.0) =  1.2
    // P_world.z = 0.3 + ( 0.2) =  0.5
    // P_world.rotation = frameA_in_world.rotation * P_local.rotation (here P_local.rotation is identity)
    // So P_world.rz = 90_deg
    CartPose expected_p_world = {-0.4_m, 1.2_m, 0.5_m, 0.0_rad, 0.0_rad, (90.0_deg).toRadians()};
    if (arePosesApproximatelyEqual(point_in_world, expected_p_world)) {
        LOG_INFO("FrameTransTest", "  transformPoseToWorld matches expected value.");
    } else {
        LOG_ERROR_F("FrameTransTest", "  transformPoseToWorld MISMATCH! Expected: %s", expected_p_world.toDescriptiveString().c_str());
    }

    CartPose point_in_local_A_check = FrameTransformer::transformPoseFromWorld(point_in_world, frameA_in_world);
    LOG_INFO_F("FrameTransTest", "Point_world back to Local A: %s", point_in_local_A_check.toDescriptiveString().c_str());

    if (arePosesApproximatelyEqual(point_in_local_A, point_in_local_A_check)) {
        LOG_INFO("FrameTransTest", "SUCCESS: Basic ToWorld/FromWorld transform consistency.");
    } else {
        LOG_ERROR("FrameTransTest", "FAILURE: Basic ToWorld/FromWorld transform inconsistency.");
    }
}

void testTcpAndFlangeCalculation() {
    LOG_INFO("FrameTransTest", "--- Testing TCP/Flange Calculation Transforms ---");

    CartPose flange_in_world    = {1.0_m, 2.0_m, 0.5_m, (10.0_deg).toRadians(), (20.0_deg).toRadians(), (30.0_deg).toRadians()};
    CartPose tool_tx_on_flange  = {0.0_m, 0.0_m, 0.25_m, 0.0_rad, 0.0_rad, 0.0_rad}; 

    LOG_DEBUG_F("FrameTransTest", "Flange in World: %s", flange_in_world.toDescriptiveString().c_str());
    LOG_DEBUG_F("FrameTransTest", "Tool transform on Flange: %s", tool_tx_on_flange.toDescriptiveString().c_str());

    CartPose tcp_in_world = FrameTransformer::calculateTcpInWorld(flange_in_world, tool_tx_on_flange);
    LOG_INFO_F("FrameTransTest", "Calculated TCP in World: %s", tcp_in_world.toDescriptiveString().c_str());

    CartPose flange_in_world_check = FrameTransformer::calculateFlangeInWorld(tcp_in_world, tool_tx_on_flange);
    LOG_INFO_F("FrameTransTest", "Calculated Flange from TCP in World: %s", flange_in_world_check.toDescriptiveString().c_str());

    if (arePosesApproximatelyEqual(flange_in_world, flange_in_world_check)) {
        LOG_INFO("FrameTransTest", "SUCCESS: TCP/Flange calculation consistency.");
    } else {
        LOG_ERROR("FrameTransTest", "FAILURE: TCP/Flange calculation inconsistency.");
    }
}

void testTransformCombinationAndInversion() {
    LOG_INFO("FrameTransTest", "--- Testing Transform Combination and Inversion ---");
    CartPose T_world_base  = {1.0_m, 0.0_m, 0.0_m, 0.0_rad, 0.0_rad, (90.0_deg).toRadians()}; // Base at (1,0,0) in World, rotated 90deg about World Z
    CartPose T_base_object = {0.0_m, 0.5_m, 0.0_m, 0.0_rad, 0.0_rad, 0.0_rad};                 // Object at (0,0.5,0) in Base frame

    LOG_DEBUG_F("FrameTransTest", "T_world_base: %s", T_world_base.toDescriptiveString().c_str());
    LOG_DEBUG_F("FrameTransTest", "T_base_object: %s", T_base_object.toDescriptiveString().c_str());

    CartPose T_world_object = FrameTransformer::combineTransforms(T_world_base, T_base_object);
    LOG_INFO_F("FrameTransTest", "Combined T_world_object: %s", T_world_object.toDescriptiveString().c_str());
    
    // Manual calculation for T_world_base * T_base_object:
    // T_world_base: Translate(1,0,0) then RotateZ(90)
    // T_base_object: Translate(0,0.5,0) in Base frame coordinates
    // Point (0,0.5,0) in Base frame, when Base is rotated 90deg about Z_world:
    //   Base_X_world = World_Y
    //   Base_Y_world = -World_X
    //   Base_Z_world = World_Z
    // So, (0,0.5,0)_base becomes:
    //   dx_world_from_base_object = 0 * World_Y + 0.5 * (-World_X_unit) = -0.5 along World_X
    //   dy_world_from_base_object = 0 * (-World_X) + 0.5 * (World_Y_unit) = 0.5 along World_Y (mistake here in my previous manual calc)
    //   Let's re-evaluate using Eigen's multiplication logic:
    //   T_wb_matrix = Translation(1,0,0) * RotationZ(90) -> This is incorrect order for Affine3d.
    //   Correct for Affine3d: T_wb_matrix.translation = (1,0,0); T_wb_matrix.linear = Rz(90)
    //   T_bo_matrix.translation = (0,0.5,0); T_bo_matrix.linear = Identity
    //   T_wo_matrix = T_wb_matrix * T_bo_matrix
    //   T_wo_matrix.translation = T_wb_matrix.translation + T_wb_matrix.linear * T_bo_matrix.translation
    //                         = [1,0,0]^T + [0 -1 0; 1 0 0; 0 0 1] * [0, 0.5, 0]^T
    //                         = [1,0,0]^T + [-0.5, 0, 0]^T
    //                         = [0.5, 0, 0]^T
    //   T_wo_matrix.linear = T_wb_matrix.linear * T_bo_matrix.linear = Rz(90) * Identity = Rz(90)
    // So, X=0.5, Y=0.0, Z=0.0, Rz=90deg.
    
    // BEGIN CORRECTION
    CartPose expected_T_world_object = {0.5_m, 0.0_m, 0.0_m, 0.0_rad, 0.0_rad, (90.0_deg).toRadians()};
    // END CORRECTION

     if (arePosesApproximatelyEqual(T_world_object, expected_T_world_object)) {
        LOG_INFO("FrameTransTest", "  combineTransforms matches expected value.");
    } else {
        LOG_ERROR_F("FrameTransTest", "  combineTransforms MISMATCH! Expected: %s", expected_T_world_object.toDescriptiveString().c_str());
    }

    CartPose T_object_base = FrameTransformer::invertTransform(T_base_object);
    LOG_INFO_F("FrameTransTest", "Inverted T_base_object (is T_object_base): %s", T_object_base.toDescriptiveString().c_str());

    CartPose T_world_base_check = FrameTransformer::combineTransforms(T_world_object, T_object_base);
    LOG_INFO_F("FrameTransTest", "Checked T_world_base (T_world_object * T_object_base): %s", T_world_base_check.toDescriptiveString().c_str());

    if (arePosesApproximatelyEqual(T_world_base, T_world_base_check)) {
        LOG_INFO("FrameTransTest", "SUCCESS: Combine/Invert transform consistency (A = B * inv(C) where B = A * C).");
    } else {
        LOG_ERROR("FrameTransTest", "FAILURE: Combine/Invert transform consistency (A = B * inv(C)).");
    }

    CartPose Identity = {0.0_m, 0.0_m, 0.0_m, 0.0_rad, 0.0_rad, 0.0_rad};
    CartPose T_base_world_inv = FrameTransformer::invertTransform(T_world_base); // This is T_base_world
    CartPose I_check = FrameTransformer::combineTransforms(T_world_base, T_base_world_inv);
    LOG_INFO_F("FrameTransTest", "Identity Check (T_world_base * T_base_world): %s", I_check.toDescriptiveString().c_str());
    if (arePosesApproximatelyEqual(I_check, Identity, 0.000001_m, (0.0001_deg).toRadians())) { // Tighter tolerance for identity
         LOG_INFO("FrameTransTest", "SUCCESS: Transform multiplied by its inverse yields Identity.");
    } else {
        LOG_ERROR("FrameTransTest", "FAILURE: Transform * Inverse != Identity.");
    }
}


int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
    Logger::setLogLevel(LogLevel::Debug); // Set to Debug to see PoseCompare logs on mismatch
    LOG_INFO("MainApp", "FrameTransformer Test Application Started.");

    try {
        testBasicPointTransforms();
        testTcpAndFlangeCalculation();
        testTransformCombinationAndInversion();
    } catch (const std::exception& e) {
        LOG_CRITICAL_F("MainApp", "An unhandled exception occurred: %s", e.what());
        return 1;
    }

    LOG_INFO("MainApp", "FrameTransformer Test Application Finished Successfully.");
    return 0;
}