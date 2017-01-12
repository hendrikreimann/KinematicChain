%     This file is part of the KinematicChain library
%     Copyright (C) 2017 Hendrik Reimann <hendrikreimann@gmail.com>
% 
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
% 
%     You should have received a copy of the GNU General Public License
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.% compare the kinematic tree against the kinematic chain

classdef GeneralKinematicTreeTest < matlab.unittest.TestCase
    % Unit Test for GeneralKinematicTree
    %   to test, instantiate and call run
    
    properties
        testChain;
        testTree
    end

    methods
        function this = GeneralKinematicTreeTest
            % chain
            joint_positions = {[0; 2; 0], [0; 2; 2], [0; 2; 4], [0; 2; 6]};
            joint_axes = {[0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0]};
            joint_types = [1 1 1 1];
            branch_matrix = [1 1 1 1];
            end_effector = {[0; 2; 8]};
            link_centers = {[0; 2; 1], [0; 2; 3], [0; 2; 5], [0; 2; 7]};
            link_orientations = {eye(3), eye(3), eye(3), eye(3)};
            link_masses = [1 1 1 1];
            link_moments_of_inertia = [1 1 1; 1 1 1; 1 1 1; 1 1 1];

            this.testChain = GeneralKinematicTree ...
            ( ...
              joint_positions, ...
              joint_axes, ...
              joint_types, ...
              branch_matrix, ...
              end_effector, ...
              link_centers, ...
              link_orientations, ...
              link_masses, ...
              link_moments_of_inertia ...
            );
        
            % tree
            joint_positions = {[0; 0; 0]; [0.5; 0; 0]; [0.5; 0; -1]; [-0.5; 0; 0]; [-0.5; 0; -1]; };
            joint_axes = {[0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0]};
            joint_types = [1 1 1 1 1];
            end_effectors = {[0.5; 0; -2], [-0.5; 0; -2]};
            link_centers = {[0; 0; 0]; [0.5; 0; -0.5]; [0.5; 0; -1.5]; [-0.5; 0; -0.5]; [-0.5; 0; -1.5];};
            branch_matrix = [1 1 1 0 0; 1 0 0 1 1]; % each row is a branch, listing the joints that move the end-effector of that branch
            link_orientations = {eye(3), eye(3), eye(3), eye(3), eye(3)};
            link_masses = ones(1, 5);
            link_moments_of_inertia = ones(5, 3)*0.01;
            this.testTree = GeneralKinematicTree ...
            ( ...
              joint_positions, ...
              joint_axes, ...
              joint_types, ...
              branch_matrix, ...
              end_effectors, ...
              link_centers, ...
              link_orientations, ...
              link_masses, ...
              link_moments_of_inertia ...
            );
%             this.testTree.updateConfiguration();
        end
    end
    methods (Test)
        function testForwardKinematicsChain(this)
            this.testChain.jointAngles = [pi/4; pi/4; -3*pi/4; -pi/4];
            this.testChain.updateConfiguration();
            
            % test the angles
            actualSolution = this.testChain.jointAngles;
            expectedSolution = [pi/4; pi/4; -3*pi/4; -pi/4];
            this.verifyEqual(actualSolution, expectedSolution);
            
            % test the joint positions
            joint_two_position_actual = this.testChain.jointPositions{2};
            joint_two_position_expexted = [sqrt(2); 2; sqrt(2)];
            this.verifyEqual(joint_two_position_actual, joint_two_position_expexted, 'AbsTol', sqrt(eps));
            joint_three_position_actual = this.testChain.jointPositions{3};
            joint_three_position_expexted = [sqrt(2)+2; 2; sqrt(2)];
            this.verifyEqual(joint_three_position_actual, joint_three_position_expexted, 'AbsTol', sqrt(eps));
            joint_four_position_actual = this.testChain.jointPositions{4};
            joint_four_position_expexted = [2; 2; 2*sqrt(2)];
            this.verifyEqual(joint_four_position_actual, joint_four_position_expexted, 'AbsTol', sqrt(eps));
            
            % test the joint orientations
            joint_one_orientation_actual = this.testChain.jointTransformations{1}(1:3, 1:3);
            joint_one_orientation_expected = [sqrt(1/2) 0 sqrt(1/2); 0 1 0; -sqrt(1/2) 0 sqrt(1/2)];
            this.verifyEqual(joint_one_orientation_actual, joint_one_orientation_expected, 'AbsTol', sqrt(eps));
            joint_two_orientation_actual = this.testChain.jointTransformations{2}(1:3, 1:3);
            joint_two_orientation_expected = [0 0 1; 0 1 0; -1 0 0];
            this.verifyEqual(joint_two_orientation_actual, joint_two_orientation_expected, 'AbsTol', sqrt(eps));
            joint_three_orientation_actual = this.testChain.jointTransformations{3}(1:3, 1:3);
            joint_three_orientation_expected = [sqrt(1/2) 0 -sqrt(1/2); 0 1 0; sqrt(1/2) 0 sqrt(1/2)];
            this.verifyEqual(joint_three_orientation_actual, joint_three_orientation_expected, 'AbsTol', sqrt(eps));
            joint_four_orientation_actual = this.testChain.jointTransformations{4}(1:3, 1:3);
            joint_four_orientation_expected = [0 0 -1; 0 1 0; 1 0 0];
            this.verifyEqual(joint_four_orientation_actual, joint_four_orientation_expected, 'AbsTol', sqrt(eps));

            % test the end-effector transformation
            eef_transformation_actual = this.testChain.endEffectorTransformations{1};
            eef_transformation_expected = [0 0 -1 0; 0 1 0 2; 1 0 0 2*sqrt(2); 0 0 0 1];
            this.verifyEqual(eef_transformation_actual, eef_transformation_expected, 'AbsTol', sqrt(eps));
        end
        function testJacobiansChain(this)
            this.testChain.jointAngles = [pi/4; pi/4; -3*pi/4; -pi/4];
            this.testChain.updateConfiguration();
            this.testChain.updateJacobians();
            this.testChain.calculateLinkJacobians();
            
            % test spatial Jacobian
            spatial_jacobian_actual = this.testChain.spatialJacobian;
            spatial_jacobian_expected = [0 -sqrt(2) -sqrt(2) -2*sqrt(2); 0 0 0 0; 0 sqrt(2) sqrt(2)+2 2; 0 0 0 0; 1 1 1 1; 0 0 0 0];
            this.verifyEqual(spatial_jacobian_actual, spatial_jacobian_expected, 'AbsTol', sqrt(eps));
            
            % test Cartesian Jacobian
            eef_jacobian_actual = this.testChain.endEffectorJacobians{1};
            eef_jacobian_expected = [2*sqrt(2) sqrt(2) sqrt(2) 0; 0 0 0 0; 0 sqrt(2) sqrt(2)+2 2];
            this.verifyEqual(eef_jacobian_actual, eef_jacobian_expected, 'AbsTol', sqrt(eps));
            
            % test body Jacobian
            body_jacobian_actual = this.testChain.bodyJacobians{1};
            body_jacobian_expected = [0 sqrt(2) sqrt(2)+2 2; 0 0 0 0; -2*sqrt(2) -sqrt(2) -sqrt(2) 0; 0 0 0 0; 1 1 1 1; 0 0 0 0];
            this.verifyEqual(body_jacobian_actual, body_jacobian_expected, 'AbsTol', sqrt(eps));
            
            % test link Jacobians
            link_one_jacobian_actual = this.testChain.linkJacobians{1};
            link_one_jacobian_expected = [sqrt(1/2) 0 0 0; 0 0 0 0; -sqrt(1/2) 0 0 0];
            this.verifyEqual(link_one_jacobian_actual, link_one_jacobian_expected, 'AbsTol', sqrt(eps));
            
            link_two_jacobian_actual = this.testChain.linkJacobians{2};
            link_two_jacobian_expected = [sqrt(2) 0 0 0; 0 0 0 0; -1-sqrt(2) -1 0 0];
            this.verifyEqual(link_two_jacobian_actual, link_two_jacobian_expected, 'AbsTol', sqrt(eps));
            
            link_three_jacobian_actual = this.testChain.linkJacobians{3};
            link_three_jacobian_expected = [3*sqrt(1/2) sqrt(1/2) sqrt(1/2) 0; 0 0 0 0; -2-sqrt(1/2) -2+sqrt(1/2) sqrt(1/2) 0];
            this.verifyEqual(link_three_jacobian_actual, link_three_jacobian_expected, 'AbsTol', sqrt(eps));
            
            link_four_jacobian_actual = this.testChain.linkJacobians{4};
            link_four_jacobian_expected = [2*sqrt(2) sqrt(2) sqrt(2) 0; 0 0 0 0; -1 -1+sqrt(2) 1+sqrt(2) 1];
            this.verifyEqual(link_four_jacobian_actual, link_four_jacobian_expected, 'AbsTol', sqrt(eps));
            
            % test CoM Jacobian
            theta_0 = [pi/4; pi/4; -3*pi/4; -pi/4];
            this.testChain.jointAngles = theta_0;
            this.testChain.updateConfiguration();
            this.testChain.updateJacobians();
            this.testChain.calculateLinkJacobians();
            com_0 = this.testChain.calculateCenterOfMassPosition;
            com_jacobian_actual = zeros(3, this.testChain.numberOfJoints);
            
            delta_angle = 1e-8;
            for i_joint = 1 : this.testChain.numberOfJoints
                theta_delta = theta_0;
                theta_delta(i_joint) = theta_0(i_joint) + delta_angle;
                this.testChain.jointAngles = theta_delta;
                this.testChain.updateConfiguration();
                this.testChain.updateJacobians();
                this.testChain.calculateLinkJacobians();
                com_delta = this.testChain.calculateCenterOfMassPosition;
                com_jacobian_actual(:, i_joint) = (com_delta - com_0) * delta_angle^(-1);
            end
            com_jacobian_expected = this.testChain.calculateCenterOfMassJacobian;
            this.verifyEqual(com_jacobian_actual, com_jacobian_expected, 'AbsTol', sqrt(sqrt(eps)));
            
            
        end
        function testDynamicMatricesChain(this)
            this.testChain.jointAngles = [pi/4; pi/4; -3*pi/4; -pi/4];
            this.testChain.jointVelocities = ones(4, 1);
            this.testChain.updateInternals();
            
            % test inertia matrix
            inertia_matrix_actual = this.testChain.inertiaMatrix;
            inertia_matrix_expected = [33.656854249492369 14 3.171572875253812 0; 14 8.343145750507624 4.585786437626910 1.414213562373099; 3.171572875253816 4.585786437626910 10.828427124746195 3.414213562373098; 0 1.414213562373096 3.414213562373095 2];
            this.verifyEqual(inertia_matrix_actual, inertia_matrix_expected, 'AbsTol', sqrt(eps));
            
            % test coriolis matrix
            coriolis_matrix_actual = this.testChain.coriolisMatrix;
            coriolis_matrix_expected = [14.828427124746188 15.171572875253814 37.798989873223334 11.313708498984756; 5.313708498984761 5.656854249492382 14.142135623730951 5.656854249492383; -14.485281374238571 -7.071067811865478 1.414213562373095 5.656854249492379; -5.656854249492378 -4.242640687119286 -4.242640687119286 0];
            this.verifyEqual(coriolis_matrix_actual, coriolis_matrix_expected, 'AbsTol', sqrt(eps));
            
            % test gravitation matrix
            gravitation_matrix_actual = this.testChain.gravitationalTorqueMatrix;
            gravitation_matrix_expected = [-66.963994862892221; -18.423553852830835; 30.609696147169160; 9.806649999999999];
            this.verifyEqual(gravitation_matrix_actual, gravitation_matrix_expected, 'AbsTol', sqrt(eps));
        end
        function testDerivativesChain(this)
            this.testChain.jointAngles = [pi/4; pi/4; -3*pi/4; -pi/4];
            this.testChain.jointVelocities = ones(4, 1);
            this.testChain.jointAccelerations = [1; 2; 3; 4];
            this.testChain.updateInternals();
            
            % store old values
            joint_angles = this.testChain.jointAngles;
            joint_velocities = this.testChain.jointVelocities;
            eef_position = this.testChain.endEffectorPositions{1};
            eef_velocity = this.testChain.endEffectorVelocities{1};
            eef_acceleration = this.testChain.endEffectorAccelerations{1};
            spatial_jacobian = this.testChain.spatialJacobian;
            body_jacobian = this.testChain.bodyJacobians{1};
            spatial_jacobian_dot = this.testChain.spatialJacobianTemporalDerivative;
            body_jacobian_dot = this.testChain.bodyJacobianTemporalDerivatives{1};
            
            % delta step
            delta_t = 1e-08;
            this.testChain.jointAngles = joint_angles + delta_t*joint_velocities + delta_t^2*this.testChain.jointAccelerations;
            this.testChain.jointVelocities = joint_velocities + delta_t*this.testChain.jointAccelerations;
            this.testChain.updateKinematics();
            
            % get new values
            joint_angles_new = this.testChain.jointAngles;
            joint_velocities_new = this.testChain.jointVelocities;
            eef_position_new = this.testChain.endEffectorPositions{1};
            eef_velocity_new = this.testChain.endEffectorVelocities{1};
            spatial_jacobian_new = this.testChain.spatialJacobian;
            body_jacobian_new = this.testChain.bodyJacobians{1};
            
            % calculate finite differences
            joint_velocities_numerical = (joint_angles_new - joint_angles) * delta_t^(-1);
            joint_acceleration_numerical = (joint_velocities_new - joint_velocities) * delta_t^(-1);
            eef_velocity_numerical = (eef_position_new - eef_position) * delta_t^(-1);
            eef_acceleration_numerical = (eef_velocity_new - eef_velocity) * delta_t^(-1);
            spatial_jacobian_dot_numerical = (spatial_jacobian_new - spatial_jacobian) * delta_t^(-1);
            body_jacobian_dot_numerical = (body_jacobian_new - body_jacobian) * delta_t^(-1);
            
            % compare
            this.verifyEqual(joint_velocities_numerical, joint_velocities, 'AbsTol', 1e-6);
            this.verifyEqual(joint_acceleration_numerical, this.testChain.jointAccelerations, 'AbsTol', 1e-6);
            this.verifyEqual(eef_velocity_numerical, eef_velocity, 'AbsTol', 1e-6);
            this.verifyEqual(eef_acceleration_numerical, eef_acceleration, 'AbsTol', 1e-4);
            
            this.verifyEqual(spatial_jacobian_dot_numerical, spatial_jacobian_dot, 'AbsTol', 1e-4);
            this.verifyEqual(body_jacobian_dot_numerical, body_jacobian_dot, 'AbsTol', 1e-4);
            
            
        end
        
        function testDerivativesTree(this)
            this.testTree.jointAngles = [pi/4; pi/4; -3*pi/4; -pi/4; pi/3];
            this.testTree.jointVelocities = ones(5, 1);
            this.testTree.jointAccelerations = [1; 2; 3; 4; 5];
            this.testTree.updateInternals();
            
            % store old values
            joint_angles = this.testTree.jointAngles;
            joint_velocities = this.testTree.jointVelocities;
            eef_one_position = this.testTree.endEffectorPositions{1};
            eef_two_position = this.testTree.endEffectorPositions{2};
            eef_one_velocity = this.testTree.endEffectorVelocities{1};
            eef_two_velocity = this.testTree.endEffectorVelocities{2};
            eef_one_acceleration = this.testTree.endEffectorAccelerations{1};
            eef_two_acceleration = this.testTree.endEffectorAccelerations{2};
            spatial_jacobian = this.testTree.spatialJacobian;
            body_jacobian_one = this.testTree.bodyJacobians{1};
            body_jacobian_two = this.testTree.bodyJacobians{2};
            spatial_jacobian_dot = this.testTree.spatialJacobianTemporalDerivative;
            body_jacobian_one_dot = this.testTree.bodyJacobianTemporalDerivatives{1};
            body_jacobian_two_dot = this.testTree.bodyJacobianTemporalDerivatives{2};
            
            % delta step
            delta_t = 1e-08;
            this.testTree.jointAngles = joint_angles + delta_t*joint_velocities + delta_t^2*this.testTree.jointAccelerations;
            this.testTree.jointVelocities = joint_velocities + delta_t*this.testTree.jointAccelerations;
            this.testTree.updateKinematics();
            
            % get new values
            joint_angles_new = this.testTree.jointAngles;
            joint_velocities_new = this.testTree.jointVelocities;
            eef_one_position_new = this.testTree.endEffectorPositions{1};
            eef_two_position_new = this.testTree.endEffectorPositions{2};
            eef_one_velocity_new = this.testTree.endEffectorVelocities{1};
            eef_two_velocity_new = this.testTree.endEffectorVelocities{2};
            spatial_jacobian_new = this.testTree.spatialJacobian;
            body_jacobian_one_new = this.testTree.bodyJacobians{1};
            body_jacobian_two_new = this.testTree.bodyJacobians{2};
            
            % calculate finite differences
            joint_velocities_numerical = (joint_angles_new - joint_angles) * delta_t^(-1);
            joint_acceleration_numerical = (joint_velocities_new - joint_velocities) * delta_t^(-1);
            eef_one_velocity_numerical = (eef_one_position_new - eef_one_position) * delta_t^(-1);
            eef_two_velocity_numerical = (eef_two_position_new - eef_two_position) * delta_t^(-1);
            eef_one_acceleration_numerical = (eef_one_velocity_new - eef_one_velocity) * delta_t^(-1);
            eef_two_acceleration_numerical = (eef_two_velocity_new - eef_two_velocity) * delta_t^(-1);
            spatial_jacobian_dot_numerical = (spatial_jacobian_new - spatial_jacobian) * delta_t^(-1);
            body_jacobian_one_dot_numerical = (body_jacobian_one_new - body_jacobian_one) * delta_t^(-1);
            body_jacobian_two_dot_numerical = (body_jacobian_two_new - body_jacobian_two) * delta_t^(-1);
            
            % compare
            this.verifyEqual(joint_velocities_numerical, joint_velocities, 'AbsTol', 1e-6);
            this.verifyEqual(joint_acceleration_numerical, this.testTree.jointAccelerations, 'AbsTol', 1e-6);
            this.verifyEqual(eef_one_velocity_numerical, eef_one_velocity, 'AbsTol', 1e-6);
            this.verifyEqual(eef_two_velocity_numerical, eef_two_velocity, 'AbsTol', 1e-6);
            this.verifyEqual(eef_one_acceleration_numerical, eef_one_acceleration, 'AbsTol', 1e-4);
            this.verifyEqual(eef_two_acceleration_numerical, eef_two_acceleration, 'AbsTol', 1e-4);
            this.verifyEqual(spatial_jacobian_dot_numerical, spatial_jacobian_dot, 'AbsTol', 1e-4);
            this.verifyEqual(body_jacobian_one_dot_numerical, body_jacobian_one_dot, 'AbsTol', 1e-4);
            this.verifyEqual(body_jacobian_two_dot_numerical, body_jacobian_two_dot, 'AbsTol', 1e-4);
            
            
        end
        function testComJacobian(this)
            this.testTree.jointAngles = [pi/4; pi/4; -3*pi/4; -pi/4; pi/3];
            this.testTree.updateInternals();
            
            % store old values
            joint_angles_0 = this.testTree.jointAngles;
            com_0 = this.testTree.calculateCenterOfMassPosition();
            
            % calculate Jacobian
            delta_theta = 1e-08;
            J_com = this.testTree.calculateCenterOfMassJacobian();
            J_com_numerical = zeros(size(J_com));
            for i_joint = 1 : this.testTree.numberOfJoints
                joint_angles_delta = joint_angles_0;
                joint_angles_delta(i_joint) = joint_angles_delta(i_joint) + delta_theta;
                this.testTree.jointAngles = joint_angles_delta;
                this.testTree.updateInternals();
                com_delta = this.testTree.calculateCenterOfMassPosition();
                J_com_numerical(:, i_joint) = (com_delta - com_0) * delta_theta^(-1);
            end
            this.verifyEqual(J_com_numerical, J_com, 'AbsTol', 1e-6);
        end
    end
    
end

