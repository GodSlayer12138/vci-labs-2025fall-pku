#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include "Labs/4-Animation/tasks.h"
#include "IKSystem.h"
#include "CustomFunc.inl"


namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        
        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
            ik.JointGlobalRotation[i] = ik.JointGlobalRotation[i - 1] * ik.JointLocalRotation[i];
            glm::vec4 p = glm::vec4(ik.JointLocalOffset[i], 1.0f);
            p = glm::mat4_cast(ik.JointGlobalRotation[i - 1]) * p;
            ik.JointGlobalPosition[i] = ik.JointGlobalPosition[i - 1] + glm::vec3(p.x, p.y, p.z) / p.w;
        }
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        // printf("Iteration number: %d\n", maxCCDIKIteration);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
            int nJoints = ik.NumJoints();
            for (int i = nJoints - 2; i >= 0; i--) {
                glm::vec3 to_end_effector = glm::normalize(ik.EndEffectorPosition() - ik.JointGlobalPosition[i]);
                glm::vec3 to_target       = glm::normalize(EndPosition - ik.JointGlobalPosition[i]);
                glm::quat rotation_update = glm::rotation(to_end_effector, to_target);
                ik.JointLocalRotation[i]  = rotation_update * ik.JointLocalRotation[i];
                ForwardKinematics(ik, i);
            }
        }
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        // printf("Iteration number: %d\n", maxFABRIKIteration);
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                // your code here
                backward_positions[i] = next_position + glm::normalize(ik.JointGlobalPosition[i] - next_position) * ik.JointOffsetLength[i + 1];
                next_position = backward_positions[i];
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0] = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints - 1; i++) {
                // your code here
                forward_positions[i + 1] = now_position + glm::normalize(backward_positions[i + 1] - now_position) * ik.JointOffsetLength[i + 1];
                now_position = forward_positions[i + 1];
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
        }

        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }

    float my_custom_x(float t) {
        return ((-2.0 / 5 * sin(11.0 / 7 - 6 * t) + 443.0 / 5 * sin(t + 11.0 / 7) + 5.0 / 11 * sin(2 * t + 17.0 / 11) + 76.0 / 9 * sin(3 * t + 11.0 / 7) + 1.0 / 2 * sin(4 * t + 14.0 / 9) + 27.0 / 7 * sin(5 * t + 11.0 / 7) - 1159.0 / 7) * theta(71 * EIGEN_PI - t) * theta(t - 67 * EIGEN_PI) + (-76.0 / 15 * sin(11.0 / 7 - 5 * t) - 115.0 / 7 * sin(11.0 / 7 - 3 * t) - 1667.0 / 8 * sin(11.0 / 7 - t) + 244.0 / 5 * sin(2 * t + 11.0 / 7) + 32.0 / 5 * sin(4 * t + 11.0 / 7) + 13.0 / 4 * sin(6 * t + 11.0 / 7) + 10.0 / 7 * sin(7 * t + 33.0 / 7) + 11.0 / 5 * sin(8 * t + 8.0 / 5) + 4.0 / 7 * sin(9 * t + 14.0 / 3) + 20.0 / 19 * sin(10 * t + 11.0 / 7) + 5.0 / 7 * sin(11 * t + 33.0 / 7) + 7.0 / 6 * sin(12 * t + 8.0 / 5) + 4737.0 / 7) * theta(67 * EIGEN_PI - t) * theta(t - 63 * EIGEN_PI) + (-7.0 / 5 * sin(11.0 / 7 - 6 * t) - 1.0 / 11 * sin(7.0 / 5 - 5 * t) - 7.0 / 2 * sin(11.0 / 7 - 4 * t) - 87.0 / 8 * sin(11.0 / 7 - 2 * t) + 72.0 / 7 * sin(t + 11.0 / 7) + 7.0 / 5 * sin(3 * t + 11.0 / 7) - 391.0 / 4) * theta(63 * EIGEN_PI - t) * theta(t - 59 * EIGEN_PI) + (-5.0 / 2 * sin(11.0 / 7 - 3 * t) - 95.0 / 9 * sin(11.0 / 7 - t) + 11.0 / 4 * sin(2 * t + 11.0 / 7) + 4.0 / 3 * sin(4 * t + 11.0 / 7) - 1164.0 / 7) * theta(59 * EIGEN_PI - t) * theta(t - 55 * EIGEN_PI) + (-6 * sin(14.0 / 9 - 10 * t) - 1.0 / 5 * sin(3.0 / 2 - 8 * t) - 231.0 / 23 * sin(17.0 / 11 - 7 * t) - 49.0 / 4 * sin(14.0 / 9 - 6 * t) - 59.0 / 9 * sin(14.0 / 9 - 5 * t) - 29.0 / 9 * sin(11.0 / 7 - 3 * t) + 4.0 / 7 * sin(t + 37.0 / 8) + 8.0 / 5 * sin(2 * t + 14.0 / 3) + 17.0 / 4 * sin(4 * t + 11.0 / 7) + 8 * sin(9 * t + 8.0 / 5) + 35.0 / 8 * sin(11 * t + 8.0 / 5) + 1.0 / 18 * sin(12 * t + 13.0 / 5) - 1794.0 / 11) * theta(55 * EIGEN_PI - t) * theta(t - 51 * EIGEN_PI) + (-13.0 / 7 * sin(14.0 / 9 - 7 * t) - 43.0 / 7 * sin(14.0 / 9 - 6 * t) - 18.0 / 19 * sin(10.0 / 7 - 5 * t) - 3 * sin(17.0 / 11 - 4 * t) - 20.0 / 3 * sin(14.0 / 9 - 3 * t) - 67.0 / 3 * sin(11.0 / 7 - t) + 16.0 / 5 * sin(2 * t + 14.0 / 9) + 39.0 / 19 * sin(8 * t + 8.0 / 5) + 7.0 / 4 * sin(9 * t + 13.0 / 8) + 3.0 / 5 * sin(10 * t + 7.0 / 4) + 6.0 / 7 * sin(11 * t + 12.0 / 7) + 12.0 / 5 * sin(12 * t + 5.0 / 3) - 1769.0 / 7) * theta(51 * EIGEN_PI - t) * theta(t - 47 * EIGEN_PI) + (-24.0 / 7 * sin(4.0 / 3 - 6 * t) + 56.0 / 19 * sin(t + 21.0 / 5) + 139.0 / 10 * sin(2 * t + 6.0 / 5) + 169.0 / 7 * sin(3 * t + 10.0 / 7) + 187.0 / 7 * sin(4 * t + 9.0 / 2) + 26.0 / 5 * sin(5 * t + 21.0 / 11) + 2 * sin(7 * t + 18.0 / 5) + 19.0 / 13 * sin(8 * t + 9.0 / 10) + 7.0 / 5 * sin(9 * t + 1.0 / 2) + 1.0 / 4 * sin(10 * t + 1.0 / 4) + 18.0 / 17 * sin(11 * t + 7.0 / 6) + 9.0 / 7 * sin(12 * t + 17.0 / 4) + 787.0 / 2) * theta(47 * EIGEN_PI - t) * theta(t - 43 * EIGEN_PI) + (-1.0 / 5 * sin(14.0 / 9 - 5 * t) - 81.0 / 20 * sin(11.0 / 7 - 2 * t) + 59.0 / 3 * sin(t + 11.0 / 7) + 10.0 / 7 * sin(3 * t + 11.0 / 7) + 3.0 / 5 * sin(4 * t + 33.0 / 7) + 1001.0 / 5) * theta(43 * EIGEN_PI - t) * theta(t - 39 * EIGEN_PI) + (-1.0 / 3 * sin(1.0 / 6 - 10 * t) - 27.0 / 7 * sin(9.0 / 7 - 8 * t) - 100.0 / 9 * sin(6.0 / 5 - 7 * t) - 57.0 / 8 * sin(3.0 / 5 - 4 * t) + 57.0 / 2 * sin(t + 1.0 / 4) + 49.0 / 2 * sin(2 * t + 13.0 / 7) + 135.0 / 7 * sin(3 * t + 15.0 / 4) + 7.0 / 3 * sin(5 * t + 35.0 / 9) + 47.0 / 6 * sin(6 * t + 3.0 / 8) + 4.0 / 5 * sin(9 * t + 22.0 / 5) + 5.0 / 4 * sin(11 * t + 33.0 / 7) + 11.0 / 9 * sin(12 * t + 5.0 / 6) - 2821.0 / 9) * theta(39 * EIGEN_PI - t) * theta(t - 35 * EIGEN_PI) + (-4.0 / 5 * sin(3.0 / 5 - 12 * t) + 82.0 / 5 * sin(4 * t) + 97.0 / 6 * sin(t + 6.0 / 7) + 148.0 / 5 * sin(2 * t + 11.0 / 5) + 155.0 / 6 * sin(3 * t + 40.0 / 9) + 37.0 / 8 * sin(5 * t + 24.0 / 7) + 31.0 / 6 * sin(6 * t + 23.0 / 11) + 142.0 / 13 * sin(7 * t + 23.0 / 9) + 104.0 / 7 * sin(8 * t + 3.0 / 2) + 18.0 / 7 * sin(9 * t + 25.0 / 6) + 18.0 / 5 * sin(10 * t + 14.0 / 5) + 8.0 / 5 * sin(11 * t + 7.0 / 5) - 97.0 / 5) * theta(35 * EIGEN_PI - t) * theta(t - 31 * EIGEN_PI) + (-32.0 / 9 * sin(10.0 / 7 - 19 * t) - 27.0 / 8 * sin(4.0 / 3 - 18 * t) - 19.0 / 4 * sin(1.0 / 32 - 15 * t) - 124.0 / 13 * sin(7.0 / 8 - 11 * t) - 107.0 / 12 * sin(17.0 / 11 - 2 * t) - 25.0 / 2 * sin(8.0 / 9 - t) + 201.0 / 5 * sin(3 * t + 32.0 / 13) + 407.0 / 12 * sin(4 * t + 1.0 / 22) + 361.0 / 6 * sin(5 * t + 11.0 / 5) + 637.0 / 11 * sin(6 * t + 23.0 / 8) + 21.0 / 2 * sin(7 * t + 21.0 / 5) + 133.0 / 5 * sin(8 * t + 11.0 / 5) + 226.0 / 9 * sin(9 * t + 1.0 / 6) + 21.0 / 4 * sin(10 * t + 34.0 / 9) + 43.0 / 4 * sin(12 * t + 16.0 / 5) + 117.0 / 29 * sin(13 * t + 9.0 / 7) + 21.0 / 11 * sin(14 * t + 26.0 / 9) + 83.0 / 6 * sin(16 * t + 18.0 / 5) + 26.0 / 7 * sin(17 * t + 1.0 / 13) - 277.0 / 6) * theta(31 * EIGEN_PI - t) * theta(t - 27 * EIGEN_PI) + (-1.0 / 6 * sin(3.0 / 4 - 19 * t) - 13.0 / 8 * sin(5.0 / 4 - 17 * t) - 8.0 / 7 * sin(11.0 / 10 - 11 * t) - 134.0 / 5 * sin(11.0 / 10 - 8 * t) - 307.0 / 10 * sin(7.0 / 6 - 7 * t) - 100.0 / 9 * sin(3.0 / 8 - t) + 29.0 / 8 * sin(2 * t + 49.0 / 11) + 111.0 / 11 * sin(3 * t + 30.0 / 7) + 47.0 / 3 * sin(4 * t + 13.0 / 8) + 61.0 / 4 * sin(5 * t + 37.0 / 8) + 179.0 / 6 * sin(6 * t + 6.0 / 5) + 72.0 / 7 * sin(9 * t + 3.0 / 8) + 37.0 / 9 * sin(10 * t + 24.0 / 23) + 11.0 / 8 * sin(12 * t + 35.0 / 9) + 3.0 / 5 * sin(13 * t + 13.0 / 5) + 3.0 / 4 * sin(14 * t + 31.0 / 15) + 5.0 / 7 * sin(15 * t + 4) + 9.0 / 5 * sin(16 * t + 6.0 / 5) + 1.0 / 8 * sin(18 * t + 7.0 / 3) - 3079.0 / 9) * theta(27 * EIGEN_PI - t) * theta(t - 23 * EIGEN_PI) + (-904.0 / 9 * sin(11.0 / 7 - t) + 16.0 / 7 * sin(2 * t + 8.0 / 3) + 9 * sin(3 * t + 23.0 / 5) + 8.0 / 7 * sin(4 * t + 23.0 / 9) + 27.0 / 11 * sin(5 * t + 37.0 / 8) + 2 * sin(6 * t + 25.0 / 9) + 19.0 / 10 * sin(7 * t + 17.0 / 4) + 7.0 / 6 * sin(8 * t + 20.0 / 7) + 8.0 / 9 * sin(9 * t + 9.0 / 2) + 4.0 / 5 * sin(10 * t + 11.0 / 4) + 5.0 / 8 * sin(11 * t + 81.0 / 20) + 3.0 / 7 * sin(12 * t + 53.0 / 18) - 517.0 / 3) * theta(23 * EIGEN_PI - t) * theta(t - 19 * EIGEN_PI) + (-26.0 / 7 * sin(1.0 / 4 - 11 * t) + 4643.0 / 7 * sin(t + 11.0 / 10) + 77.0 / 2 * sin(2 * t + 7.0 / 3) + 192.0 / 5 * sin(3 * t + 1.0 / 34) + 144.0 / 7 * sin(4 * t + 8.0 / 5) + 39.0 / 19 * sin(5 * t + 13.0 / 4) + 19.0 / 2 * sin(6 * t + 3.0 / 5) + 71.0 / 10 * sin(7 * t + 13.0 / 7) + 8.0 / 3 * sin(8 * t + 1.0 / 5) + 97.0 / 16 * sin(9 * t + 5.0 / 6) + 9.0 / 5 * sin(10 * t + 6.0 / 5) + 17.0 / 7 * sin(12 * t + 8.0 / 7) + 7278.0 / 19) * theta(19 * EIGEN_PI - t) * theta(t - 15 * EIGEN_PI) + (-19.0 / 7 * sin(9.0 / 7 - 9 * t) - 17.0 / 6 * sin(3.0 / 5 - 8 * t) - 22.0 / 5 * sin(11.0 / 10 - 7 * t) - 107.0 / 12 * sin(1.0 / 9 - 5 * t) + 1562.0 / 5 * sin(t + 5.0 / 4) + 41.0 / 3 * sin(2 * t + 5.0 / 6) + 94.0 / 3 * sin(3 * t + 6.0 / 11) + 29.0 / 5 * sin(4 * t + 3.0 / 5) + 64.0 / 13 * sin(6 * t + 1.0 / 45) + 3.0 / 2 * sin(10 * t + 9.0 / 2) + 2 * sin(11 * t + 37.0 / 8) + 8.0 / 9 * sin(12 * t + 19.0 / 6) + 2588.0 / 13) * theta(15 * EIGEN_PI - t) * theta(t - 11 * EIGEN_PI) + (-33.0 / 13 * sin(9.0 / 8 - 12 * t) - 14.0 / 9 * sin(4.0 / 3 - 11 * t) - 25.0 / 7 * sin(7.0 / 13 - 10 * t) - 17.0 / 6 * sin(2.0 / 9 - 9 * t) - 82.0 / 11 * sin(3.0 / 2 - 4 * t) + 898.0 / 3 * sin(t + 5.0 / 4) + 207.0 / 4 * sin(2 * t + 24.0 / 7) + 77.0 / 3 * sin(3 * t + 31.0 / 15) + 58.0 / 5 * sin(5 * t + 24.0 / 7) + 118.0 / 9 * sin(6 * t + 45.0 / 11) + 33.0 / 5 * sin(7 * t + 9.0 / 4) + 4.0 / 3 * sin(8 * t + 7.0 / 2) + 1774.0 / 9) * theta(11 * EIGEN_PI - t) * theta(t - 7 * EIGEN_PI) + (-41.0 / 8 * sin(3.0 / 7 - 6 * t) - 41.0 / 4 * sin(3.0 / 2 - 4 * t) + 2098.0 / 5 * sin(t + 13.0 / 3) + 419.0 / 9 * sin(2 * t + 40.0 / 9) + 104.0 / 5 * sin(3 * t + 11.0 / 5) + 40.0 / 7 * sin(5 * t + 5.0 / 2) + 16.0 / 9 * sin(7 * t + 91.0 / 23) + 29.0 / 10 * sin(8 * t + 13.0 / 3) + 3.0 / 2 * sin(9 * t + 19.0 / 18) + 11.0 / 4 * sin(10 * t + 27.0 / 7) + 13.0 / 8 * sin(11 * t + 3.0 / 4) + 5.0 / 11 * sin(12 * t + 57.0 / 14) - 47.0 / 5) * theta(7 * EIGEN_PI - t) * theta(t - 3 * EIGEN_PI) + (-16.0 / 5 * sin(5.0 / 6 - 15 * t) - 89.0 / 15 * sin(5.0 / 11 - 7 * t) - 117.0 / 8 * sin(4.0 / 3 - 6 * t) - 65.0 / 2 * sin(7.0 / 9 - 2 * t) + 3097.0 / 7 * sin(t + 2) + 67.0 / 9 * sin(3 * t + 3.0 / 4) + 428.0 / 13 * sin(4 * t + 65.0 / 16) + 111.0 / 7 * sin(5 * t + 9.0 / 2) + 11.0 / 3 * sin(8 * t + 22.0 / 5) + 13.0 / 6 * sin(9 * t + 17.0 / 8) + 33.0 / 8 * sin(10 * t + 43.0 / 11) + 33.0 / 7 * sin(11 * t + 30.0 / 7) + 11.0 / 4 * sin(12 * t + 32.0 / 7) + 39.0 / 20 * sin(13 * t + 32.0 / 7) + 13.0 / 6 * sin(14 * t + 17.0 / 4) + 14.0 / 9 * sin(16 * t + 10.0 / 11) + 237.0 / 14) * theta(3 * EIGEN_PI - t) * theta(t + EIGEN_PI)) * theta(sqrt(sgn(sin(t/2))));
    }
    float my_custom_y(float t) {
        return ((-1.0 / 3 * sin(11.0 / 7 - 6 * t) - 5.0 / 7 * sin(11.0 / 7 - 4 * t) - 8.0 / 3 * sin(11.0 / 7 - 2 * t) + 36.0 / 7 * sin(t + 11.0 / 7) + sin(3 * t + 11.0 / 7) + 4.0 / 5 * sin(5 * t + 11.0 / 7) - 575.0 / 7) * theta(71 * EIGEN_PI - t) * theta(t - 67 * EIGEN_PI) + (-13.0 / 5 * sin(11.0 / 7 - 12 * t) - 10.0 / 3 * sin(11.0 / 7 - 10 * t) - 1.0 / 4 * sin(13.0 / 9 - 9 * t) - 5 * sin(11.0 / 7 - 8 * t) - 5.0 / 7 * sin(10.0 / 7 - 7 * t) - 177.0 / 22 * sin(11.0 / 7 - 6 * t) - 19.0 / 18 * sin(16.0 / 11 - 5 * t) - 67.0 / 4 * sin(11.0 / 7 - 4 * t) - 29.0 / 8 * sin(14.0 / 9 - 3 * t) - 273.0 / 4 * sin(11.0 / 7 - 2 * t) - 1085.0 / 6 * sin(11.0 / 7 - t) + 3.0 / 7 * sin(11 * t + 16.0 / 11) - 8933.0 / 11) * theta(67 * EIGEN_PI - t) * theta(t - 63 * EIGEN_PI) + (-5.0 / 4 * sin(11.0 / 7 - 4 * t) - 106.0 / 21 * sin(11.0 / 7 - 2 * t) - 103.0 / 3 * sin(11.0 / 7 - t) + 1.0 / 20 * sin(3 * t + 37.0 / 8) + 1.0 / 5 * sin(5 * t + 11.0 / 7) + 1.0 / 4 * sin(6 * t + 33.0 / 7) + 56) * theta(63 * EIGEN_PI - t) * theta(t - 59 * EIGEN_PI) + (-1.0 / 2 * sin(11.0 / 10 - 4 * t) - 9.0 / 4 * sin(3.0 / 2 - 3 * t) - sin(11.0 / 7 - 2 * t) - 152.0 / 5 * sin(11.0 / 7 - t) + 1315.0 / 7) * theta(59 * EIGEN_PI - t) * theta(t - 55 * EIGEN_PI) + (-5.0 / 3 * sin(3.0 / 2 - 12 * t) - 2.0 / 5 * sin(10.0 / 7 - 11 * t) - 8.0 / 7 * sin(20.0 / 13 - 10 * t) - 17.0 / 9 * sin(17.0 / 11 - 8 * t) - 19.0 / 5 * sin(14.0 / 9 - 7 * t) - 1.0 / 5 * sin(11.0 / 7 - 3 * t) + 11.0 / 10 * sin(t + 8.0 / 5) + 11.0 / 8 * sin(2 * t + 8.0 / 5) + 2.0 / 3 * sin(4 * t + 8.0 / 5) + 13.0 / 4 * sin(5 * t + 8.0 / 5) + 2.0 / 5 * sin(6 * t + 11.0 / 7) + 8.0 / 9 * sin(9 * t + 11.0 / 7) + 185.0 / 7) * theta(55 * EIGEN_PI - t) * theta(t - 51 * EIGEN_PI) + (-719.0 / 8 * sin(11.0 / 7 - t) + 315.0 / 8 * sin(2 * t + 11.0 / 7) + 9.0 / 2 * sin(3 * t + 33.0 / 7) + 19.0 / 10 * sin(4 * t + 8.0 / 5) + 4 * sin(5 * t + 33.0 / 7) + 17.0 / 2 * sin(6 * t + 8.0 / 5) + 13.0 / 3 * sin(7 * t + 5.0 / 3) + 54.0 / 11 * sin(8 * t + 13.0 / 8) + 12.0 / 13 * sin(9 * t + 9.0 / 5) + 13.0 / 4 * sin(10 * t + 13.0 / 8) + 15.0 / 7 * sin(11 * t + 5.0 / 3) + 13.0 / 7 * sin(12 * t + 13.0 / 8) + 356.0 / 5) * theta(51 * EIGEN_PI - t) * theta(t - 47 * EIGEN_PI) + (425.0 / 7 * sin(t + 37.0 / 8) + 81.0 / 7 * sin(2 * t + 16.0 / 11) + 25.0 / 4 * sin(3 * t + 9.0 / 2) + 118.0 / 5 * sin(4 * t + 22.0 / 5) + 59.0 / 5 * sin(5 * t + 49.0 / 12) + 51.0 / 5 * sin(6 * t + 22.0 / 5) + 20.0 / 3 * sin(7 * t + 7.0 / 6) + 21.0 / 8 * sin(8 * t + 3.0 / 5) + 12.0 / 5 * sin(9 * t + 43.0 / 11) + 10.0 / 7 * sin(10 * t + 11.0 / 3) + 1.0 / 2 * sin(11 * t + 26.0 / 7) + 4.0 / 7 * sin(12 * t + 31.0 / 7) + 1627.0 / 7) * theta(47 * EIGEN_PI - t) * theta(t - 43 * EIGEN_PI) + (299.0 / 4 * sin(t + 11.0 / 7) + 5.0 / 4 * sin(2 * t + 8.0 / 5) + 42.0 / 5 * sin(3 * t + 11.0 / 7) + 1.0 / 24 * sin(4 * t + 17.0 / 4) + 23.0 / 7 * sin(5 * t + 11.0 / 7) - 328) * theta(43 * EIGEN_PI - t) * theta(t - 39 * EIGEN_PI) + (-44.0 / 9 * sin(9.0 / 7 - 6 * t) - 46.0 / 9 * sin(4.0 / 5 - t) + 20.0 / 3 * sin(2 * t + 3.0 / 4) + 32.0 / 7 * sin(3 * t + 23.0 / 11) + 21.0 / 11 * sin(4 * t + 33.0 / 8) + 21.0 / 5 * sin(5 * t + 13.0 / 8) + 37.0 / 7 * sin(7 * t + 26.0 / 7) + 5.0 / 6 * sin(8 * t + 9.0 / 4) + 27.0 / 26 * sin(9 * t + 30.0 / 7) + 7.0 / 5 * sin(10 * t + 32.0 / 11) + 1.0 / 18 * sin(11 * t + 11.0 / 6) + 6.0 / 13 * sin(12 * t + 16.0 / 7) + 601.0 / 3) * theta(39 * EIGEN_PI - t) * theta(t - 35 * EIGEN_PI) + (-111.0 / 16 * sin(1 - 4 * t) + 57.0 / 8 * sin(t + 1.0 / 2) + 85.0 / 9 * sin(2 * t + 5.0 / 4) + 33.0 / 4 * sin(3 * t + 7.0 / 2) + 17.0 / 6 * sin(5 * t + 8.0 / 5) + 16.0 / 5 * sin(6 * t + 23.0 / 12) + 70.0 / 23 * sin(7 * t + 4.0 / 3) + 41.0 / 9 * sin(8 * t + 1.0 / 4) + 5.0 / 2 * sin(9 * t + 79.0 / 20) + 32.0 / 13 * sin(10 * t + 8.0 / 5) + 3.0 / 2 * sin(11 * t + 1.0 / 33) + 2.0 / 7 * sin(12 * t + 13.0 / 8) + 1007.0 / 4) * theta(35 * EIGEN_PI - t) * theta(t - 31 * EIGEN_PI) + (-35.0 / 12 * sin(14.0 / 9 - 10 * t) - 17.0 / 4 * sin(1.0 / 4 - 9 * t) - 44.0 / 7 * sin(18.0 / 17 - 4 * t) - 29.0 / 15 * sin(13.0 / 12 - t) + 53.0 / 8 * sin(11 * t) + 55.0 / 7 * sin(2 * t + 22.0 / 5) + 82.0 / 7 * sin(3 * t + 9.0 / 5) + 52.0 / 7 * sin(5 * t + 18.0 / 17) + 168.0 / 13 * sin(6 * t + 17.0 / 7) + 25.0 / 6 * sin(7 * t + 13.0 / 4) + 17.0 / 3 * sin(8 * t + 13.0 / 12) + 21.0 / 8 * sin(12 * t + 13.0 / 5) + 25.0 / 7 * sin(13 * t + 3.0 / 5) + 15.0 / 7 * sin(14 * t + 11.0 / 3) + 17.0 / 18 * sin(15 * t + 1.0 / 3) + 7.0 / 3 * sin(16 * t + 21.0 / 8) + 16.0 / 7 * sin(17 * t + 13.0 / 3) + 9.0 / 4 * sin(18 * t + 23.0 / 6) + 13.0 / 8 * sin(19 * t + 6.0 / 5) + 2009.0 / 6) * theta(31 * EIGEN_PI - t) * theta(t - 27 * EIGEN_PI) + (-5.0 / 4 * sin(9.0 / 7 - 16 * t) - 3.0 / 4 * sin(1.0 / 4 - 15 * t) - 2.0 / 3 * sin(9.0 / 7 - 13 * t) - 38.0 / 11 * sin(7.0 / 6 - 8 * t) + 29.0 / 4 * sin(t + 5.0 / 2) + 81.0 / 7 * sin(2 * t + 17.0 / 11) + 61.0 / 15 * sin(3 * t + 29.0 / 7) + 49.0 / 9 * sin(4 * t + 15.0 / 14) + 5.0 / 6 * sin(5 * t + 21.0 / 8) + 117.0 / 29 * sin(6 * t + 14.0 / 3) + 21.0 / 4 * sin(7 * t + 45.0 / 23) + 14.0 / 5 * sin(9 * t + 10.0 / 9) + 8.0 / 3 * sin(10 * t + 37.0 / 8) + 21.0 / 11 * sin(11 * t + 4.0 / 3) + 1.0 / 6 * sin(12 * t + 37.0 / 12) + 5.0 / 7 * sin(14 * t + 32.0 / 7) + 11.0 / 7 * sin(17 * t + 1) + 2 * sin(18 * t + 17.0 / 4) + 5.0 / 4 * sin(19 * t + 7.0 / 6) + 278) * theta(27 * EIGEN_PI - t) * theta(t - 23 * EIGEN_PI) + (-2.0 / 5 * sin(3.0 / 7 - 11 * t) - 2.0 / 3 * sin(1.0 / 5 - 10 * t) - 7.0 / 6 * sin(1.0 / 5 - 6 * t) - 13.0 / 6 * sin(6.0 / 5 - 5 * t) + 1108.0 / 27 * sin(t + 10.0 / 3) + 6.0 / 5 * sin(2 * t + 26.0 / 9) + 11.0 / 4 * sin(3 * t + 14.0 / 5) + 29.0 / 8 * sin(4 * t + 49.0 / 11) + 12.0 / 13 * sin(7 * t + 1.0 / 5) + 7.0 / 6 * sin(8 * t + 12.0 / 5) + 3.0 / 4 * sin(9 * t + 11.0 / 3) + 1.0 / 4 * sin(12 * t + 6.0 / 5) - 353.0 / 4) * theta(23 * EIGEN_PI - t) * theta(t - 19 * EIGEN_PI) + (-12.0 / 5 * sin(5.0 / 6 - 8 * t) - 48.0 / 7 * sin(4.0 / 5 - 6 * t) - 2749.0 / 8 * sin(1.0 / 3 - t) + 591.0 / 7 * sin(2 * t + 35.0 / 8) + 323.0 / 7 * sin(3 * t + 17.0 / 4) + 134.0 / 9 * sin(4 * t + 5.0 / 4) + 77.0 / 3 * sin(5 * t + 53.0 / 13) + 90.0 / 7 * sin(7 * t + 28.0 / 9) + 11.0 / 3 * sin(9 * t + 30.0 / 7) + 11.0 / 5 * sin(10 * t + 22.0 / 7) + 17.0 / 4 * sin(11 * t + 71.0 / 18) + 17.0 / 6 * sin(12 * t + 9.0 / 10) - 4437.0 / 5) * theta(19 * EIGEN_PI - t) * theta(t - 15 * EIGEN_PI) + (-44.0 / 3 * sin(19.0 / 20 - 3 * t) + 1180.0 / 7 * sin(t + 14.0 / 15) + 20.0 / 3 * sin(2 * t + 7.0 / 3) + 41.0 / 4 * sin(4 * t + 1) + 76.0 / 5 * sin(5 * t + 30.0 / 7) + 35.0 / 6 * sin(6 * t + 6.0 / 7) + 121.0 / 15 * sin(7 * t + 29.0 / 8) + 31.0 / 9 * sin(8 * t + 3.0 / 4) + 14.0 / 3 * sin(9 * t + 19.0 / 6) + 11.0 / 5 * sin(10 * t + 8.0 / 7) + 1.0 / 2 * sin(11 * t + 17.0 / 6) + 11.0 / 7 * sin(12 * t + 23.0 / 12) - 2876.0 / 5) * theta(15 * EIGEN_PI - t) * theta(t - 11 * EIGEN_PI) + (-23.0 / 9 * sin(4.0 / 5 - 9 * t) - 841.0 / 12 * sin(10.0 / 7 - 3 * t) + 1518.0 / 5 * sin(t + 3.0 / 8) + 292.0 / 9 * sin(2 * t + 2.0 / 7) + 19.0 / 3 * sin(4 * t + 9.0 / 5) + 59.0 / 7 * sin(5 * t + 14.0 / 5) + 13.0 / 3 * sin(6 * t + 8.0 / 7) + 36.0 / 5 * sin(7 * t + 1.0 / 3) + 9.0 / 5 * sin(8 * t + 20.0 / 7) + 5.0 / 2 * sin(10 * t + 12.0 / 7) + 15.0 / 8 * sin(11 * t + 19.0 / 7) + 4.0 / 3 * sin(12 * t + 1.0 / 14) - 2920.0 / 9) * theta(11 * EIGEN_PI - t) * theta(t - 7 * EIGEN_PI) + (-153.0 / 19 * sin(19.0 / 20 - 10 * t) - 6 * sin(1.0 / 25 - 8 * t) + 866.0 / 5 * sin(t + 17.0 / 7) + 138 * sin(2 * t + 25.0 / 6) + 47 * sin(3 * t + 2) + 45.0 / 4 * sin(4 * t + 27.0 / 7) + 37.0 / 2 * sin(5 * t + 19.0 / 9) + 58.0 / 9 * sin(6 * t + 6.0 / 7) + 53.0 / 7 * sin(7 * t + 13.0 / 9) + 23.0 / 7 * sin(9 * t + 3.0 / 7) + 41.0 / 9 * sin(11 * t + 32.0 / 9) + 19.0 / 8 * sin(12 * t + 41.0 / 9) + 2911.0 / 5) * theta(7 * EIGEN_PI - t) * theta(t - 3 * EIGEN_PI) + (-29.0 / 8 * sin(1.0 / 8 - 16 * t) - 9.0 / 10 * sin(2.0 / 7 - 15 * t) - 23.0 / 6 * sin(1.0 / 2 - 11 * t) - 158.0 / 7 * sin(6.0 / 7 - 5 * t) + 3279.0 / 8 * sin(t + 25.0 / 7) + 93 * sin(2 * t + 9.0 / 7) + 109.0 / 7 * sin(3 * t + 7.0 / 4) + 89.0 / 6 * sin(4 * t + 30.0 / 7) + 169.0 / 6 * sin(6 * t + 1.0 / 3) + 79.0 / 5 * sin(7 * t + 4.0 / 7) + 46.0 / 5 * sin(8 * t + 8.0 / 7) + 29.0 / 7 * sin(9 * t + 5.0 / 2) + 9.0 / 4 * sin(10 * t + 1.0 / 12) + 29.0 / 4 * sin(12 * t + 1.0 / 4) + 9.0 / 2 * sin(13 * t + 11.0 / 9) + 9.0 / 5 * sin(14 * t + 8.0 / 7) + 629.0 / 4) * theta(3 * EIGEN_PI - t) * theta(t + EIGEN_PI)) * theta(sqrt(sgn(sin(t/2))));
    }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        int nums = 5000;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
        int index = 0;
        for (int i = 0; i < nums; i++) {
            float x_val = 1.5e-3f * my_custom_x(72 * glm::pi<float>() * i / nums);
            float y_val = 1.5e-3f * my_custom_y(72 * glm::pi<float>() * i / nums);
            if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
            (*custom)[index++] = glm::vec3(0.4f - x_val, 0.0f, y_val + 0.4f);
        }
        custom->resize(index);
        return custom;
    }

    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size())
        );
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const & b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }

    static Eigen::VectorXf f_ext(MassSpringSystem const & system) {
        std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0, -system.Gravity, 0));
        return glm2eigen(forces);
    }

    static Eigen::VectorXf x_minus_y(MassSpringSystem const & system, Eigen::VectorXf const & v, float const dt) {
        return -(dt * v + dt * dt * f_ext(system) / system.Mass);
    }

    static Eigen::VectorXf grad_E(MassSpringSystem const & system, float const dt) {
        std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
            glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
            glm::vec3 const e01 = glm::normalize(x01);
            glm::vec3 f = system.Stiffness * (glm::length(x01) - spring.RestLength) * e01;
            forces[p0] -= f;
            forces[p1] += f;
        }
        return glm2eigen(forces);
    }

    static Eigen::VectorXf grad_g(MassSpringSystem const & system, Eigen::VectorXf const & v, float const dt) {
        return (system.Mass / (dt * dt)) * x_minus_y(system, v, dt) + grad_E(system, dt);
    }

    static Eigen::SparseMatrix<float> Hessian_g(MassSpringSystem const & system, float const dt) {
        auto const & x = system.Positions;
        std::vector<Eigen::Triplet<float>> triplets;
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = x[p1] - x[p0];
            float const l = glm::length(x01);
            glm::vec3 const e01 = glm::normalize(x01);
            Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
            Eigen::Matrix3f outer_e01 = Eigen::Map<Eigen::Vector3f const>(reinterpret_cast<float const *>(&e01)) * Eigen::Map<Eigen::Vector3f const>(reinterpret_cast<float const *>(&e01)).transpose();
            Eigen::Matrix3f H_local = system.Stiffness * ((1 - spring.RestLength / l) * (I - outer_e01) + outer_e01);
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    triplets.emplace_back(3 * p0 + row, 3 * p0 + col, H_local(row, col));
                    triplets.emplace_back(3 * p1 + row, 3 * p1 + col, H_local(row, col));
                    triplets.emplace_back(3 * p0 + row, 3 * p1 + col, -H_local(row, col));
                    triplets.emplace_back(3 * p1 + row, 3 * p0 + col, -H_local(row, col));
                }
            }
        }
        Eigen::SparseMatrix<float> H = CreateEigenSparseMatrix(system.Positions.size() * 3, triplets);
        Eigen::SparseMatrix<float> M(system.Positions.size() * 3, system.Positions.size() * 3);
        for (std::size_t i = 0; i < system.Positions.size(); i++) {
            for (int j = 0; j < 3; j++) {
                M.insert(3 * i + j, 3 * i + j) = system.Mass;
            }
        }
        return (1 / (dt * dt)) * M + H;
    }

    static Eigen::VectorXf f(MassSpringSystem const & system, Eigen::VectorXf const & x, Eigen::VectorXf const & v, float const dt) {
        std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0, -system.Gravity, 0));
        auto x_glm = eigen2glm(x);
        auto v_glm = eigen2glm(v);
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = x_glm[p1] - x_glm[p0];
            glm::vec3 const v01 = v_glm[p1] - v_glm[p0];
            glm::vec3 const e01 = glm::normalize(x01);
            glm::vec3 f = system.Stiffness * (glm::length(x01) - spring.RestLength) * e01;
            forces[p0] += f;
            forces[p1] -= f;
        }
        return glm2eigen(forces);
    }

    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        // your code here: rewrite following code
        // int const steps = 1000;
        // float const ddt = dt / steps; 

        // for (std::size_t s = 0; s < steps; s++) {
        //     std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
        //     for (auto const spring : system.Springs) {
        //         auto const p0 = spring.AdjIdx.first;
        //         auto const p1 = spring.AdjIdx.second;
        //         glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
        //         glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
        //         glm::vec3 const e01 = glm::normalize(x01);
        //         glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
        //         forces[p0] += f;
        //         forces[p1] -= f;
        //     }
        //     for (std::size_t i = 0; i < system.Positions.size(); i++) {
        //         if (system.Fixed[i]) continue;
        //         system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
        //         system.Positions[i] += system.Velocities[i] * ddt;
        //     }
        // }
        
        auto x = glm2eigen(system.Positions);
        auto v = glm2eigen(system.Velocities);
        auto delta_x = ComputeSimplicialLLT(
            Hessian_g(system, dt),
            -grad_g(system, v, dt)
        );
        x += delta_x;
        v += (dt / system.Mass) * f(system, x, v, dt);
        auto x_glm = eigen2glm(x);
        auto v_glm = eigen2glm(v);
        for (std::size_t i = 0; i < system.Positions.size(); i++) {
            if (system.Fixed[i]) continue;
            system.Positions[i] = x_glm[i];
            system.Velocities[i] = v_glm[i];
        }
    }
}
