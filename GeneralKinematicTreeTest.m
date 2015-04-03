classdef GeneralKinematicTreeTest < matlab.unittest.TestCase
    % Unit Test for GeneralKinematicTree
    %   to test, instantiate and call run
    
    properties
        testChain;
    end

    methods
        function this = GeneralKinematicTreeTest
            joint_positions = {[0; 2; 0], [0; 2; 2], [0; 2; 4], [0; 2; 6]};
            joint_axes = {[0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0]};
            joint_types = [1 1 1 1];
            branch_matrix = [1 1 1 1];
            eef = {[0; 2; 8]};
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
              eef, ...
              link_centers, ...
              link_orientations, ...
              link_masses, ...
              link_moments_of_inertia ...
            );            
        end
    end
    methods (Test)
        function testForwardKinematics(this)
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
        function testJacobians(this)
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
        end
        function testDynamicMatrices(this)
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
        function testDerivatives(this)
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
    end
    
end

