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
            end_effector_transformation_actual = this.testChain.endEffectorTransformations{1};
            end_effector_transformation_expected = [0 0 -1 0; 0 1 0 2; 1 0 0 2*sqrt(2); 0 0 0 1];
            this.verifyEqual(end_effector_transformation_actual, end_effector_transformation_expected, 'AbsTol', sqrt(eps));
        end
        function testJacobians(this)
            this.testChain.jointAngles = [pi/4; pi/4; -3*pi/4; -pi/4];
            this.testChain.updateConfiguration();
            this.testChain.updateJacobians();
            
            % test spatial Jacobian
            spatial_jacobian_actual = this.testChain.spatialJacobian;
            spatial_jacobian_expected = [0 -sqrt(2) -sqrt(2) -2*sqrt(2); 0 0 0 0; 0 sqrt(2) sqrt(2)+2 2; 0 0 0 0; 1 1 1 1; 0 0 0 0];
            this.verifyEqual(spatial_jacobian_actual, spatial_jacobian_expected, 'AbsTol', sqrt(eps));
            
            % test Cartesian Jacobian
            end_effector_jacobian_actual = this.testChain.endEffectorJacobians{1};
            end_effector_jacobian_expected = [2*sqrt(2) sqrt(2) sqrt(2) 0; 0 0 0 0; 0 sqrt(2) sqrt(2)+2 2];
            this.verifyEqual(end_effector_jacobian_actual, end_effector_jacobian_expected, 'AbsTol', sqrt(eps));
            
            % test body Jacobian
            body_jacobian_actual = this.testChain.bodyJacobians{1};
            body_jacobian_expected = [0 sqrt(2) sqrt(2)+2 2; 0 0 0 0; -2*sqrt(2) -sqrt(2) -sqrt(2) 0; 0 0 0 0; 1 1 1 1; 0 0 0 0];
            this.verifyEqual(body_jacobian_actual, body_jacobian_expected, 'AbsTol', sqrt(eps));
            
            
        end
        function testDynamicMatrices(this)
            this.testChain.jointAngles = [pi/4; pi/4; -3*pi/4; -pi/4];
            this.testChain.updateInternals();
            
            % test inertia matrix
            inertia_matrix_actual = this.testChain.inertiaMatrix;
            inertia_matrix_expected = [33.656854249492369 14 3.171572875253812 0; 14 8.343145750507624 4.585786437626910 1.414213562373099; 3.171572875253816 4.585786437626910 10.828427124746195 3.414213562373098; 0 1.414213562373096 3.414213562373095 2];
            this.verifyEqual(inertia_matrix_actual, inertia_matrix_expected, 'AbsTol', sqrt(eps));
            
            % test coriolis matrix
            coriolis_matrix_actual = this.testChain.coriolisMatrix;
            coriolis_matrix_expected = [33.656854249492369 14 3.171572875253812 0; 14 8.343145750507624 4.585786437626910 1.414213562373099; 3.171572875253816 4.585786437626910 10.828427124746195 3.414213562373098; 0 1.414213562373096 3.414213562373095 2];
            this.verifyEqual(coriolis_matrix_actual, coriolis_matrix_expected, 'AbsTol', sqrt(eps));
            
            % test gravitation matrix
            gravitation_matrix_actual = this.testChain.gravitationMatrix;
            gravitation_matrix_expected = [33.656854249492369 14 3.171572875253812 0; 14 8.343145750507624 4.585786437626910 1.414213562373099; 3.171572875253816 4.585786437626910 10.828427124746195 3.414213562373098; 0 1.414213562373096 3.414213562373095 2];
            this.verifyEqual(gravitation_matrix_actual, gravitation_matrix_expected, 'AbsTol', sqrt(eps));
        end        
    end
    
end

