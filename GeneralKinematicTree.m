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

classdef GeneralKinematicTree < KinematicTree
    properties
        % reference data
        referenceJointTwists;
        referenceJointTransformations;
        referenceEndEffectorTransformations;
        referenceLinkTransformations;
        transformedLinkInertiaMatrices;
        generalizedInertiaMatrices;
        markerReferencePositions;
        jointAxes;
        
        % screw geometry data
        twistExponentials
        productsOfExponentials
        jointTransformations
        endEffectorTransformations
        linkTransformations
        interTransformations
        interAdjoints
        inverseInterAdjoints
        inertiaMatrixPartialDerivatives
        spatialJacobian
        spatialJacobianTemporalDerivative
        linkJacobians
        bodyJacobians
        bodyJacobianTemporalDerivatives
        
        % visualization data
        linkVisualizationReferenceData
        markerConnectionLineIndices
    end
    methods
        function obj = GeneralKinematicTree(jointPositions, jointAxes, jointTypes, branchMatrix, endEffectorData, linkCenters, linkOrientations, varargin)
            if nargin == 0
                jointPositions = {[0; 0; 0]};
                jointAxes = {[0; 0; 1]};
                jointTypes = 1;
                branchMatrix = 1;
                endEffectorData = {[2; 0; 0]};
                linkCenters = {[1; 0; 0]};
                linkOrientations = {eye(3)};
                varargin = {1, [1 1 1]};
            end
            degrees_of_freedom = length(jointPositions);
            obj = obj@KinematicTree(degrees_of_freedom, branchMatrix);
            
            % generate references
            obj.jointAxes = jointAxes;
            for i_joint = 1 : degrees_of_freedom
                obj.referenceJointTransformations{i_joint} = [eye(3), jointPositions{i_joint}; 0 0 0 1];
                obj.referenceJointTwists{i_joint} = generateTwistCoordinates(jointPositions{i_joint}, jointAxes{i_joint}, jointTypes(i_joint));
            end
            for i_eef = 1 : obj.numberOfBranches
                if numel(endEffectorData{i_eef}) == 3
                    obj.referenceEndEffectorTransformations{i_eef} = [eye(3) endEffectorData{i_eef}; 0 0 0 1];
                elseif numel(endEffectorData{i_eef}) == 16
                    obj.referenceEndEffectorTransformations{i_eef} = endEffectorData{i_eef};
                else
                    error('end-effectors must be provided either by position (3x1) or transformation (4x4 rigid)')
                end
                obj.bodyJacobians{i_eef} = zeros(6, obj.numberOfJoints);
                obj.bodyJacobianTemporalDerivatives{i_eef} = zeros(6, obj.numberOfJoints);
            end
            
            % links
            if size(varargin, 2) == 2
                obj.linkMasses = varargin{1};
                link_moments_of_inertia = varargin{2};
                
                for i_joint = 1 : degrees_of_freedom
                    obj.referenceLinkTransformations{i_joint} = [linkOrientations{i_joint} linkCenters{i_joint}; 0 0 0 1];
                    generalized_inertia_matrix = zeros(6, 6);
                    generalized_inertia_matrix(1, 1) = obj.linkMasses(i_joint);
                    generalized_inertia_matrix(2, 2) = obj.linkMasses(i_joint);
                    generalized_inertia_matrix(3, 3) = obj.linkMasses(i_joint);
                    generalized_inertia_matrix(4, 4) = link_moments_of_inertia(i_joint, 1);
                    generalized_inertia_matrix(5, 5) = link_moments_of_inertia(i_joint, 2);
                    generalized_inertia_matrix(6, 6) = link_moments_of_inertia(i_joint, 3);
                    obj.generalizedInertiaMatrices{i_joint} = generalized_inertia_matrix;
                end
                
            elseif size(varargin, 2) == 1
                obj.linkMasses = zeros(1, degrees_of_freedom);
                obj.generalizedInertiaMatrices = varargin{1};
                for i_joint = 1 : degrees_of_freedom
                    obj.referenceLinkTransformations{i_joint} = [linkOrientations{i_joint} linkCenters{i_joint}; 0 0 0 1];
                    obj.linkMasses(i_joint) = obj.generalizedInertiaMatrices{i_joint}(1, 1);
                end
            else
                error('inertial properties not specified');
            end
            
            obj.transformedLinkInertiaMatrices = cell(1, degrees_of_freedom);
            for i_joint = 1 : degrees_of_freedom
                adjoint_transformation = rigidToAdjointTransformation(obj.referenceLinkTransformations{i_joint});
                inverse_adjoint = invertAdjointTransformation(adjoint_transformation);
                obj.transformedLinkInertiaMatrices{i_joint} = inverse_adjoint' * obj.generalizedInertiaMatrices{i_joint} * inverse_adjoint;
            end
            
            % generate marker data container
            obj.markerReferencePositions = cell(obj.numberOfJoints, 1);

            obj.updateInternals();
            
            % generate link visualization data
            obj.linkVisualizationReferenceData = struct([]);
            for i_joint = 1 : obj.numberOfJoints-1
                start_point = obj.jointTransformations{i_joint}(1:3, 4);
                end_point = obj.jointTransformations{i_joint+1}(1:3, 4);
                obj.linkVisualizationData(i_joint).startPoints(:, 1) = start_point;
                obj.linkVisualizationData(i_joint).endPoints(:, 1) = end_point;
                obj.linkVisualizationReferenceData(i_joint).startPoints(:, 1) = start_point;
                obj.linkVisualizationReferenceData(i_joint).endPoints(:, 1) = end_point;
            end
            
            % generate default labels
            for i_joint = 1 : obj.numberOfJoints
                obj.jointLabels{i_joint} = ['joint ' num2str(i_joint)];
            end
            
        end
        function updateConfiguration(obj)
            % update geometric transformations
            obj.calculateTwistExponentials();
            obj.calculateProductsOfExponentials();
            obj.calculateJointTransformations();
            obj.calculateEndEffectorTransformations();
            obj.calculateLinkTransformations();
            
            % update dependent variables
            for i_joint = 1 : obj.numberOfJoints
                obj.jointPositions{i_joint} = obj.jointTransformations{i_joint}(1:3, 4);
            end
            for i_eef = 1 : obj.numberOfBranches
                obj.endEffectorPositions{i_eef} = obj.endEffectorTransformations{i_eef}(1:3, 4);
            end
            
            % update marker positions
            for i_joint = 1 : obj.numberOfJoints
                joint_transformation = obj.productsOfExponentials{i_joint};
                for i_marker = 1 : size(obj.markerReferencePositions{i_joint}, 2)
                    current_position = joint_transformation * obj.markerReferencePositions{i_joint}(:, i_marker);
                    obj.markerPositions{i_joint}(:, i_marker) = current_position;
                end
            end
            
        end
        function updateJacobians(obj)
            % update Jacobians
            obj.calculateSpatialJacobian();
            obj.calculateEndEffectorJacobians();
%             obj.calculateEndEffectorOrientationJacobians();
            obj.calculateBodyJacobians();
        end
        function updateInternals(obj)
            obj.updateKinematics();
            
            % update dynamic matrices and derivatives
            obj.calculateInertiaMatrix();
            obj.calculateInertiaMatrixPartialDerivatives();
            obj.calculateCoriolisMatrix();
            obj.calculateLinkJacobians();
            obj.calculateGravitationMatrix();
            
            % update marker positions
            for i_joint = 1 : obj.numberOfJoints
                joint_transformation = obj.productsOfExponentials{i_joint};
                for i_marker = 1 : size(obj.markerReferencePositions{i_joint}, 2)
                    current_position = joint_transformation * obj.markerReferencePositions{i_joint}(:, i_marker);
                    obj.markerPositions{i_joint}(:, i_marker) = current_position;
                end
            end

            
        end
        function updateKinematics(obj)
            obj.updateConfiguration();
            obj.updateJacobians();
            
            obj.calculateEndEffectorVelocities();
            
            % calculate things needed for inertia and coriolis/centrifugal matrix
            obj.calculateInterTransformations();
            obj.calculateInterAdjoints();
            obj.calculateInverseInterAdjoints();
            
            % update second-order temporal derivatives
            obj.calculateSpatialJacobianTemporalDerivative();
            obj.calculateEndEffectorJacobianTemporalDerivative();
            obj.calculateBodyJacobianTemporalDerivatives();
            obj.calculateEndEffectorAccelerations();
            
        end
        function addMarker(obj, joint_index, marker_reference_position, visualization_color)
            if nargin < 4
                visualization_color = rand(1, 3);
            end
            if joint_index == 0
                obj.fixedMarkerPositions = [obj.fixedMarkerPositions marker_reference_position];
                obj.markerExportMap = [obj.markerExportMap [joint_index; size(obj.fixedMarkerPositions, 2)]];
                obj.fixedMarkerVisualizationColors = [obj.fixedMarkerVisualizationColors; visualization_color];
            else
                obj.markerReferencePositions{joint_index} = [obj.markerReferencePositions{joint_index} [marker_reference_position; 1]];
                obj.markerPositions{joint_index} = [obj.markerPositions{joint_index} zeros(4, 1)];
                obj.markerExportMap = [obj.markerExportMap [joint_index; size(obj.markerPositions{joint_index}, 2)]];
                obj.markerVisualizationColors{joint_index} = [obj.markerVisualizationColors{joint_index}; visualization_color];
            end
        end
        function color = getMarkerVisualizationColor(obj, jointIndex, markerIndex)
            if jointIndex == 0
                color = obj.fixedMarkerVisualizationColors(markerIndex, :);
            else
                color = obj.markerVisualizationColors{jointIndex}(markerIndex, :);
            end
        end
        function addMarkerConnectionLineBody(obj, marker_indices)
            number_of_markers = length(marker_indices);
            for i_start_marker = 1 : number_of_markers
                for i_target_marker = i_start_marker+1 : number_of_markers
                    obj.markerConnectionLineIndices = [obj.markerConnectionLineIndices; marker_indices(i_start_marker) marker_indices(i_target_marker)];
                end
            end
        end
        function calculateTwistExponentials(obj)
            % calculate the twist exponentials from the current joint
            % angles and the reference twists
            for i_joint = 1 : obj.numberOfJoints
                obj.twistExponentials{i_joint} = expTwist(obj.referenceJointTwists{i_joint}, obj.jointAngles(i_joint));
            end
        end
        function calculateProductsOfExponentials(obj)
        % calculates the products of exponentials from the current twist exponentials
            for i_joint = 1 : obj.numberOfJoints
                if obj.jointParents(i_joint) == 0
                    obj.productsOfExponentials{i_joint} = obj.twistExponentials{i_joint};
                else
                    obj.productsOfExponentials{i_joint} = obj.productsOfExponentials{obj.jointParents(i_joint)} * obj.twistExponentials{i_joint};
                end
            end
        end
        function calculateJointTransformations(obj)
        % calculates the transformations between world and joint coordinate frames
            for i_joint = 1 : obj.numberOfJoints
                obj.jointTransformations{i_joint} = obj.productsOfExponentials{i_joint} * obj.referenceJointTransformations{i_joint};
            end
        end
        function calculateEndEffectorTransformations(obj)
        % calculates the transformations between world and joint end-effector coordinate frames
            for i_eef = 1 : obj.numberOfBranches
                obj.endEffectorTransformations{i_eef} = obj.productsOfExponentials{obj.endEffectorParents(i_eef)} * obj.referenceEndEffectorTransformations{i_eef};
            end
        end
        function calculateSpatialJacobian(obj)
            % first column
            obj.spatialJacobian(1:6, 1) = obj.referenceJointTwists{1};
            % child columns
            for i_joint = 2 : obj.numberOfJoints
                adjoint = rigidToAdjointTransformation(obj.productsOfExponentials{obj.jointParents(i_joint)});
                obj.spatialJacobian(1:6, i_joint) = adjoint * obj.referenceJointTwists{i_joint};
            end
        end
        function calculateEndEffectorJacobians(obj)
            eef_position_local = [0; 0; 0; 1]; % end-effector position in local coordinates
            for i_eef = 1 : obj.numberOfBranches
                for i_joint = 1 : obj.numberOfJoints
                    joint_twist = wedgeTwist(obj.spatialJacobian(:, i_joint));
                    current_column = joint_twist * obj.endEffectorTransformations{i_eef} * eef_position_local;
                    obj.endEffectorJacobians{i_eef}(1:3, i_joint) = current_column(1:3) * obj.branchMatrix(i_eef, i_joint);
                end
            end
        end
        function calculateEndEffectorOrientationJacobians(obj)
            % define the relevant axes
            x_world = [1; 0; 0];
            y_world = [0; 1; 0];
            z_world = [0; 0; 1];
            
            for i_eef = 1 : obj.numberOfBranches
                end_effector_orientation = obj.endEffectorTransformations{i_eef}(1:3, 1:3);
                euler_angles = eulerAnglesZXY(end_effector_orientation);
                psi_z = euler_angles(1);
                phi_x = euler_angles(2);
                gamma_y = euler_angles(3);
                R_z = rodrigues(z_world, psi_z);
                R_x = rodrigues(x_world, phi_x);
                R_y = rodrigues(y_world, gamma_y);

                R = R_z * R_x * R_y;
                y_prime = R_z(:, 2)
                X_body = R(:, 1);
                Y_body = R(:, 2);
                Z_body = R(:, 3);
                
                xy_normal = z_world;
                XZ_normal = Y_body;
                N = cross(xy_normal, XZ_normal) * norm(cross(xy_normal, XZ_normal))^(-1)
                psi_check = subspace(x_world, N)
                phi_check = subspace(y_prime, Y_body)
                gamma_check = subspace(X_body, N)            
            
            end
        end
        function calculateBodyJacobians(obj)
            for i_eef = 1 : obj.numberOfBranches
                adjoint = rigidToAdjointTransformation(obj.endEffectorTransformations{i_eef});
                inverseAdjoint = adjoint^(-1);
                bodyJacobian = inverseAdjoint * obj.spatialJacobian;
                % remove columns that do not move this end-effector
                for i_joint = 1 : obj.numberOfJoints
                    bodyJacobian(:, i_joint) = bodyJacobian(:, i_joint) * obj.branchMatrix(i_eef, i_joint);
                end
                obj.bodyJacobians{i_eef} = bodyJacobian;
            end
        end
        function calculateEndEffectorVelocities(obj)
            for i_eef = 1 : obj.numberOfBranches
                obj.endEffectorVelocities{i_eef} = obj.endEffectorJacobians{i_eef} * obj.jointVelocities;
            end
        end
        function calculateEndEffectorAccelerations(obj)
            for i_eef = 1 : obj.numberOfBranches
                obj.endEffectorAccelerations{i_eef} ...
                    = obj.endEffectorJacobians{i_eef} * obj.jointAccelerations ...
                      + obj.endEffectorJacobianTemporalDerivatives{i_eef} * obj.jointVelocities;
            end
        end
        function calculateLinkTransformations(obj)
            for i_joint = 1 : obj.numberOfJoints
                obj.linkTransformations{i_joint} = obj.productsOfExponentials{i_joint} * obj.referenceLinkTransformations{i_joint};
            end
        end 
        function calculateInterTransformations(obj)
            % fill cell array with zeros and identities
            for i_joint = 1 : obj.numberOfJoints
                for j_joint = 1 : obj.numberOfJoints
                    if i_joint == j_joint
                        obj.interTransformations{i_joint, j_joint} = eye(4, 4);
                    else
                        obj.interTransformations{i_joint, j_joint} = zeros(4, 4);
                    end
                end
            end
        
            % calculate transformations that are non-zero and non-identity
            for j_joint = 1 : obj.numberOfJoints-1
                if obj.connectivityMatrix(j_joint, j_joint+1)
                    % first element of this column, i=j+1
                    obj.interTransformations{j_joint+1, j_joint} = obj.twistExponentials{j_joint+1};
                end
                % rest of the column
                for i_joint = j_joint+2 : obj.numberOfJoints
                    if obj.connectivityMatrix(j_joint, i_joint)
                        obj.interTransformations{i_joint, j_joint} = obj.interTransformations{obj.jointParents(i_joint), j_joint} * obj.twistExponentials{i_joint};
                    end
                end
            end
        end
        function calculateInterAdjoints(obj)
            % calculate transformations that are non-zero and non-identity
            for i_joint = 1 : obj.numberOfJoints
                for j_joint = 1 : obj.numberOfJoints
                    obj.interAdjoints{i_joint, j_joint} = rigidToAdjointTransformation(obj.interTransformations{i_joint, j_joint});
                end
            end
        end
        function calculateInverseInterAdjoints(obj)
            % calculate transformations that are non-zero and non-identity
            for i_joint = 1 : obj.numberOfJoints
                for j_joint = 1 : obj.numberOfJoints
                    obj.inverseInterAdjoints{i_joint, j_joint} = invertAdjointTransformation(obj.interAdjoints{i_joint, j_joint});
                end
            end
        end
        function calculateInertiaMatrix(obj)
            for i_joint = 1 : obj.numberOfJoints
                for j_joint = 1 : obj.numberOfJoints
                    % calculate M_{ij}
                    m_ij = 0;
                    parent_index = min(i_joint, j_joint);
                    child_index = max(i_joint, j_joint);
                    % check if the child is actually being moved by the parent
                    if (obj.connectivityMatrix(parent_index, child_index) || parent_index == child_index)
                        % first entry that actually contributes
                        m_ij = obj.referenceJointTwists{i_joint}' ...
                             * obj.inverseInterAdjoints{child_index, i_joint}' ...
                             * obj.transformedLinkInertiaMatrices{child_index} ...
                             * obj.inverseInterAdjoints{child_index, j_joint} ...
                             * obj.referenceJointTwists{j_joint};
                        for l_joint = child_index+1 : obj.numberOfJoints
                            if obj.connectivityMatrix(child_index, l_joint)
                                p = obj.referenceJointTwists{i_joint}' ...
                                    * obj.inverseInterAdjoints{l_joint, i_joint}' ...
                                    * obj.transformedLinkInertiaMatrices{l_joint} ...
                                    * obj.inverseInterAdjoints{l_joint, j_joint} ...
                                    * obj.referenceJointTwists{j_joint};
                                m_ij = m_ij + p;
                            end
                        end
                    end
                    obj.inertiaMatrix(i_joint, j_joint) = m_ij;
                end
            end
        end
        function calculateInertiaMatrixPartialDerivatives(obj)
            for k_joint = 1 : obj.numberOfJoints
                for i_joint = 1 : obj.numberOfJoints
                    for j_joint = 1 : obj.numberOfJoints
                        m_ijk = 0;
                        parent_index = min(i_joint, j_joint);
                        child_index = max(i_joint, j_joint);
                        % check if the child is actually being moved by the parent
                        if (obj.connectivityMatrix(parent_index, child_index) || parent_index == child_index)
                            % check if the changing joint (k) is being moved by the parent
                            if (obj.connectivityMatrix(parent_index, k_joint) || parent_index == k_joint)
                                % calculate $\partial M_{ij} \by \partial \theta_k$
                                A_kiTimesXi_i = obj.inverseInterAdjoints{k_joint, i_joint} * obj.referenceJointTwists{i_joint};
                                lieBracket1 = twistLieBracket(A_kiTimesXi_i, obj.referenceJointTwists{k_joint});
                                A_kjTimesXi_j = obj.inverseInterAdjoints{k_joint, j_joint} * obj.referenceJointTwists{j_joint};
                                lieBracket2 = twistLieBracket(A_kjTimesXi_j, obj.referenceJointTwists{k_joint});
                                % add up the derivatives
                                for l_joint = max(i_joint, j_joint) : obj.numberOfJoints
                                    if (obj.connectivityMatrix(child_index, l_joint) || child_index == l_joint)
                                        % only add this one if l_joint is actually in the current branch of
                                        % interest. the current branch of interest is the one starting at the
                                        % child (see Springer Handbook, eq. 2.85 on p. 55
                                        p1 = lieBracket1' ...
                                             * obj.inverseInterAdjoints{l_joint, k_joint}' ...
                                             * obj.transformedLinkInertiaMatrices{l_joint} ...
                                             * obj.inverseInterAdjoints{l_joint, j_joint} ...
                                             * obj.referenceJointTwists{j_joint};
                                        p2 = obj.referenceJointTwists{i_joint}' ...
                                             * obj.inverseInterAdjoints{l_joint, i_joint}' ...
                                             * obj.transformedLinkInertiaMatrices{l_joint} ...
                                             * obj.inverseInterAdjoints{l_joint, k_joint} ...
                                             * lieBracket2;
                                        m_ijk = m_ijk + p1 + p2;
                                    end
                                end
                            end
                        end
                        obj.inertiaMatrixPartialDerivatives{i_joint, j_joint, k_joint} = m_ijk;
                    end
                end
            end
        end
        function calculateCoriolisMatrix(obj)
            obj.coriolisMatrix = zeros(obj.numberOfJoints, obj.numberOfJoints);
            for i_joint = 1 : obj.numberOfJoints
                for j_joint = 1 : obj.numberOfJoints
                    % calculate ij-th entry of the matrix
                    c_ij = 0;
                    for k_joint = 1 : obj.numberOfJoints
                        c_ij = c_ij + (obj.inertiaMatrixPartialDerivatives{i_joint, j_joint, k_joint} ...
                                                   + obj.inertiaMatrixPartialDerivatives{i_joint, k_joint, j_joint} ...
                                                   - obj.inertiaMatrixPartialDerivatives{k_joint, j_joint, i_joint}) ...
                                                 * obj.jointVelocities(k_joint) ...
                                                 * 0.5;
                    end
                    obj.coriolisMatrix(i_joint, j_joint) = c_ij;
                end
            end
        end
        function calculateLinkJacobians(obj)
            for i_joint = 1 : obj.numberOfJoints
                position = obj.linkTransformations{i_joint}(1:4, 4);
                jacobian = zeros(3, obj.numberOfJoints);

                for j_joint = 1 : i_joint
                    % check whether the j-th joint actually moves the i-th joint (or rather link)
                    if (obj.connectivityMatrix(j_joint, i_joint) || j_joint == i_joint)
                        twist = wedgeTwist(obj.spatialJacobian(1:6, j_joint));
                        column = twist * position;
                        jacobian(1:3, j_joint) = column(1:3);
                    end
                end
                obj.linkJacobians{i_joint} = jacobian;
            end
        end
        function calculateGravitationMatrix(obj)
            for k_joint = 1 : obj.numberOfJoints
                N_k = 0;
                for j_joint = 1 : obj.numberOfJoints
                    N_k = N_k + obj.linkMasses(j_joint) * obj.linkJacobians{j_joint}(3, k_joint);
                end
                obj.gravitationalTorqueMatrix(k_joint) = obj.standardGravity * N_k;
            end
        end
        function calculateSpatialJacobianTemporalDerivative(obj)
            for j_joint = 1 : obj.numberOfJoints
                % g = exp(xiHat_1*theta_1)*...*exp(xiHat_{j-1}*theta_{j-1}) 
                %       * xiHat_j 
                %       * exp(-xiHat_{j-1}*theta_{j-1})*...*exp(xiHat_1*theta_1)
                % this is a product with 2 * (j-1) factors depending upon time
                % its derivative is a sum with 2 * (j-1) summands

                g_dot = zeros(4, 4);
                for k_joint = 1 : j_joint-1
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % the k-th summand, deriving the factor with positive sign theta_k
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    s_k = eye(4, 4); % the summand where the factor with the positive sign theta_k is derived

                    % factors before the k-th
                    for i_joint = 1 : k_joint-1
                        % i-th factor stays the same for i < k
                        if obj.connectivityMatrix(i_joint, j_joint)
                            s_k = s_k * obj.twistExponentials{i_joint};
                        end
                    end
                    % k-th factor is derived by time
                    if obj.connectivityMatrix(k_joint, j_joint)
                        s_k = s_k * wedgeTwist(obj.referenceJointTwists{k_joint}) * obj.jointVelocities(k_joint) * obj.twistExponentials{k_joint};
                    end
                    % factors after the k-th
                    for i_joint = k_joint+1 : j_joint-1
                        % i-th factor stays the same for i > k
                        if obj.connectivityMatrix(i_joint, j_joint)
                            s_k = s_k * obj.twistExponentials{i_joint};
                        end
                    end
                    s_k = s_k * wedgeTwist(obj.referenceJointTwists{j_joint}) * obj.productsOfExponentials{obj.jointParents(j_joint)}^(-1);

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % the (2*(j-1)-k)-th summand, deriving the factor with negative sign theta_k
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    t_k = obj.productsOfExponentials{obj.jointParents(j_joint)} * wedgeTwist(obj.referenceJointTwists{j_joint}); 
                    % the summand where the factor with the negative sign theta_k is derived
                    for i_joint = j_joint-1 : -1 : k_joint+1
                        if obj.connectivityMatrix(i_joint, j_joint)
                            t_k = t_k * obj.twistExponentials{i_joint}^(-1);
                        end
                    end
                    if obj.connectivityMatrix(k_joint, j_joint)
                        t_k = t_k * wedgeTwist(obj.referenceJointTwists{k_joint}) * obj.jointVelocities(k_joint) * obj.twistExponentials{k_joint}^(-1);
                    end
                    for i_joint = k_joint-1 : -1 : 1
                        % i-th factor stays the same for i < k
                        if obj.connectivityMatrix(i_joint, j_joint)
                            t_k = t_k * obj.twistExponentials{i_joint}^(-1);
                        end
                    end

                    % add this summand to the sum
                    parent_index = k_joint;
                    child_index = j_joint;
                    if (obj.connectivityMatrix(parent_index, child_index) || parent_index == child_index)
%                         g_dot = g_dot + obj.jointVelocities(k_joint)*(s_k - t_k);
                        g_dot = g_dot + (s_k - t_k);
                    end
                end

                obj.spatialJacobianTemporalDerivative(:, j_joint) = veeTwist(g_dot);
            end
        end
        function calculateBodyJacobianTemporalDerivatives(obj)
            for i_eef = 1 : obj.numberOfBranches
                J_s_dot = obj.spatialJacobianTemporalDerivative;
                for i_joint = 1 : obj.numberOfJoints
                    J_s_dot(:, i_joint) = J_s_dot(:, i_joint) * obj.branchMatrix(i_eef, i_joint);
                end
                % todo: check if this can be simplified
                end_effector_transformation = obj.endEffectorTransformations{i_eef};
                body_velocity = obj.bodyJacobians{i_eef} * obj.jointVelocities;
                eef_trafo_dot_analytical = end_effector_transformation * wedgeTwist(body_velocity);
                R_eef = end_effector_transformation(1:3, 1:3);
                p_eef = end_effector_transformation(1:3, 4);
                R_eef_dot = eef_trafo_dot_analytical(1:3, 1:3);
                p_eef_dot = eef_trafo_dot_analytical(1:3, 4);
                p_eef_skew = wedgeAxis(p_eef);
                p_eef_dot_skew = wedgeAxis(p_eef_dot);
                eef_trafo_adjoint_dot_analytical = [R_eef_dot p_eef_dot_skew*R_eef+p_eef_skew*R_eef_dot; zeros(3) R_eef_dot];
                
                obj.bodyJacobianTemporalDerivatives{i_eef} = ...
                    invertAdjointTransformation(rigidToAdjointTransformation(end_effector_transformation)) * ...
                      (J_s_dot - eef_trafo_adjoint_dot_analytical * obj.bodyJacobians{i_eef});
            end
        end
        function [bodyJacobian, bodyJacobianTemporalDerivative] = calculateArbitraryFrameBodyJacobian(obj, coordinateFrame, attachmentJoint)
            % calculate body Jacobian
            coordinate_frame_adjoint = rigidToAdjointTransformation(coordinateFrame);
            inverse_coordinate_frame_adjoint = coordinate_frame_adjoint^(-1);
            body_jacobian_full = inverse_coordinate_frame_adjoint * obj.spatialJacobian;
            bodyJacobian = zeros(size(body_jacobian_full));
            % remove columns that do not move this point
            for i_joint = 1 : obj.numberOfJoints
                if obj.connectivityMatrix(i_joint, attachmentJoint) || (i_joint == attachmentJoint)
                    bodyJacobian(:, i_joint) = body_jacobian_full(:, i_joint);
                end
            end
            
            % calculate temporal derivative
            if nargout == 2
                % get and adjust spatial Jacobian temporal derivative
                J_s_dot_full = obj.spatialJacobianTemporalDerivative;
                J_s_dot = zeros(size(obj.spatialJacobianTemporalDerivative));
                for i_joint = 1 : obj.numberOfJoints
                    if obj.connectivityMatrix(i_joint, attachmentJoint) || (i_joint == attachmentJoint)
                        J_s_dot(:, i_joint) = J_s_dot_full(:, i_joint);
                    end
                end
            end            
            body_velocity = bodyJacobian * obj.jointVelocities;
            coordinate_frame_dot = coordinateFrame * wedgeTwist(body_velocity);
            R = coordinateFrame(1:3, 1:3);
            p = coordinateFrame(1:3, 4);
            R_dot = coordinate_frame_dot(1:3, 1:3);
            p_dot = coordinate_frame_dot(1:3, 4);
            p_skew = wedgeAxis(p);
            p_dot_skew = wedgeAxis(p_dot);
            coordinate_frame_adjoint_dot = [R_dot p_dot_skew*R+p_skew*R_dot; zeros(3) R_dot];
            if nargout == 2
                bodyJacobianTemporalDerivative = ...
                  invertAdjointTransformation(rigidToAdjointTransformation(coordinateFrame)) * ...
                  (J_s_dot - coordinate_frame_adjoint_dot * bodyJacobian);
            end
        end
        
        
        
        function updateLinkVisualizationData(obj)
%             for i_joint = 1 : obj.numberOfJoints
%                 for i_line = 1 : size(obj.linkVisualizationReferenceData(i_joint).startPoints, 2)
%                     start_point_reference = [obj.linkVisualizationReferenceData(i_joint).startPoints(:, i_line); 1];
%                     start_point_current = obj.productsOfExponentials{i_joint} * start_point_reference;
%                     obj.linkVisualizationData(i_joint).startPoints(:, i_line) = start_point_current(1:3);
%                     end_point_reference = [obj.linkVisualizationReferenceData(i_joint).endPoints(:, i_line); 1];
%                     end_point_current = obj.productsOfExponentials{i_joint} * end_point_reference;
%                     obj.linkVisualizationData(i_joint).endPoints(:, i_line) = end_point_current(1:3);
%                 end
%             end
        end
        function calculateEndEffectorJacobianTemporalDerivative(obj)
            for i_eef = 1 : obj.numberOfBranches
                end_effector_jacobian_temporal_derivative = zeros(size(obj.endEffectorJacobians{i_eef}));
            
                end_effector_position = [obj.endEffectorPositions{i_eef}; 1];
                end_effector_velocity = [obj.endEffectorVelocities{i_eef}; 0];
                for i_joint = 1 : obj.numberOfJoints
                    if obj.branchMatrix(i_eef, i_joint);
                        S1 = wedgeTwist(obj.spatialJacobianTemporalDerivative(:, i_joint)) * end_effector_position;
                        S2 = wedgeTwist(obj.spatialJacobian(:, i_joint)) * end_effector_velocity;
                        column = S1 + S2;
                    else
                        column = zeros(3, 1);
                    end
                    end_effector_jacobian_temporal_derivative(:, i_joint) = column(1:3);                        
                end
                obj.endEffectorJacobianTemporalDerivatives{i_eef} = end_effector_jacobian_temporal_derivative;
            end
        end
        function com = calculateCenterOfMassPosition(obj)
            com = [0; 0; 0];
            for i_joint = 1 : obj.numberOfJoints
                com = com + obj.linkTransformations{i_joint}(1:3, 4) * obj.linkMasses(i_joint);
            end
            com = com * (1 / sum(obj.linkMasses));
        end
        function jacobian = calculateCenterOfMassJacobian(obj)
            jacobian = zeros(3, obj.numberOfJoints);
            for i_joint = 1 : obj.numberOfJoints
                jacobian = jacobian + obj.linkJacobians{i_joint} * obj.linkMasses(i_joint);
            end
            jacobian = jacobian * (1 / sum(obj.linkMasses));
        end
        function Jacobian = calculateArbitraryPointJacobian(obj, point, jointIndex, coordinateFrame)
            Jacobian = zeros(3, obj.numberOfJoints);
            if nargin < 4
                coordinateFrame = 'world';
            end
            if strcmp(coordinateFrame, 'world')
                point_local = obj.jointTransformations{jointIndex}^(-1) * point;
            elseif strcmp(coordinateFrame, 'local')
                point_local = point;
            else
                error('coordinate frame must be specified as "local" or "world"');
            end
            for i_joint = 1 : obj.numberOfJoints
                point_moved_by_this_joint = obj.connectivityMatrix(i_joint, jointIndex);
                if i_joint == jointIndex
                    point_moved_by_this_joint = 1;
                end
                joint_twist = wedgeTwist(obj.spatialJacobian(:, i_joint));
                current_column = joint_twist * obj.jointTransformations{jointIndex} * point_local;
                Jacobian(:, i_joint) = current_column(1:3) * point_moved_by_this_joint;
            end
        end
        
        
        function position = getPointOfInterestPosition(obj, label)
            % determine joint index
            index_in_list = find(strcmp(obj.pointsOfInterestLables, label), 1, 'first');
            joint_index = obj.pointsOfInterestJointIndices{index_in_list};
            
            position = eye(3, 4) * obj.productsOfExponentials{joint_index} * [obj.pointsOfInterest{index_in_list}; 1];
        end
        
    end
end


































