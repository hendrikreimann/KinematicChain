


classdef generalKinematicTree < kinematicTree
    properties
        % reference data
        referenceJointTwists;
        referenceJointTransformations;
        referenceEndEffectorTransformations;
        referenceLinkTransformations;
        transformedLinkInertiaMatrices;
        generalizedInertiaMatrices;
        
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
        linkJacobians
    end
    methods
        function obj = generalKinematicTree(jointPositions, jointAxes, branchMatrix, endEffectorPositions, linkCenters, linkMasses, linkMomentsOfInertia)
            degrees_of_freedom = length(jointPositions);
            obj = obj@kinematicTree(degrees_of_freedom, branchMatrix);
            obj.linkMasses = linkMasses;
            
            % generate references
            for i_joint = 1 : degrees_of_freedom
                obj.referenceJointTransformations{i_joint} = createReferenceTransformation(jointPositions{i_joint}, eye(3));
                obj.referenceJointTwists{i_joint} = createTwist(jointPositions{i_joint}, jointAxes{i_joint});
            end
            for i_eef = 1 : obj.numberOfBranches
                obj.referenceEndEffectorTransformations{i_eef} = createReferenceTransformation(endEffectorPositions{i_eef}, eye(3));
            end
            
            
            
            if nargin > 4
                for i_joint = 1 : degrees_of_freedom
                    obj.referenceLinkTransformations{i_joint} = createReferenceTransformation(linkCenters{i_joint}, eye(3));
                    obj.generalizedInertiaMatrices{i_joint} = generalizedInertiaMatrix(linkMasses(i_joint), linkMomentsOfInertia(i_joint, :));
                end
                obj.transformedLinkInertiaMatrices = ...                    % calculate transformed inertia matrices
                    calculateTransformedLinkInertiaMatrices(obj.generalizedInertiaMatrices, obj.referenceLinkTransformations);
            end
            
            
            
            
        end
        function updateInternals(obj)
            % update geometric transformations
            obj.calculateTwistExponentials();
            obj.calculateProductsOfExponentials();
            obj.calculateJointTransformations();
            obj.calculateEndEffectorTransformations();
            
            % update Jacobians
            obj.calculateSpatialJacobian();
            obj.calculateEndEffectorJacobians();
            
            % update dependent variables
            for i_joint = 1 : obj.numberOfJoints
                obj.jointPositions{i_joint} = obj.jointTransformations{i_joint}(1:3, 4);
            end
            for i_eef = 1 : obj.numberOfBranches
                obj.endEffectorPositions{i_eef} = obj.endEffectorTransformations{i_eef}(1:3, 4);
            end
            obj.calculateEndEffectorVelocities();
            
            
            % calculate things needed for inertia and coriolis/centrifugal matrix
            obj.calculateLinkTransformations();
            obj.calculateInterTransformations();
            obj.calculateInterAdjoints();
            obj.calculateInverseInterAdjoints();
            
            % update dynamic matrices and derivatives
            obj.calculateInertiaMatrix();
            obj.calculateInertiaMatrixPartialDerivatives();
            obj.calculateCoriolisMatrix();
            obj.calculateLinkJacobians();
            obj.calculateGravitationMatrix();
            
            
            
            return
            


            % update Jacobians
            obj.linkJacobians = calculateLinkJacobians(obj.spatialJacobian, obj.linkTransformations);
            

            % update coriolis and gravitation matrices
            obj.coriolisMatrix = calculateCoriolisMatrix(obj.inertiaMatrixPartialDerivatives, obj.jointVelocities);
            obj.gravitationalTorqueMatrix = calculateGravitationMatrix(obj.linkJacobians, obj.linkMasses);
            
            % update functionals
%             obj.endEffectorPosition = obj.jointTransformations{ obj.numberOfJoints + 1 }(1:3, 4);
            obj.endEffectorVelocity = obj.endEffectorJacobian * obj.jointVelocities;
            obj.endEffectorAcceleration = calculateEndEffectorAcceleration ...
                                            (...
                                                obj.jointTransformations, ...
                                                obj.referenceJointTwists, ...
                                                obj.productsOfExponentials, ...
                                                obj.twistExponentials, ...
                                                obj.referenceJointTwists, ...
                                                obj.jointVelocities, ...
                                                obj.jointAccelerations ...
                                           );
            
        end
        function calculateTwistExponentials(obj)
            % calculate the twist exponentials from the current joint
            % angles and the reference twists
            for i_joint = 1 : obj.numberOfJoints
                obj.twistExponentials{i_joint} = twistExponential(obj.referenceJointTwists{i_joint}, obj.jointAngles(i_joint));
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
                adjoint = createAdjointTransformation(obj.productsOfExponentials{obj.jointParents(i_joint)});
                obj.spatialJacobian(1:6, i_joint) = adjoint * obj.referenceJointTwists{i_joint};
            end
        end
        function calculateEndEffectorJacobians(obj)
            eef_position_local = [0; 0; 0; 1]; % end-effector position in local coordinates
            for i_eef = 1 : obj.numberOfBranches
                for i_joint = 1 : obj.numberOfJoints
                    joint_twist = twistCoordinatesToMatrix(obj.spatialJacobian(:, i_joint));
                    current_column = joint_twist * obj.endEffectorTransformations{i_eef} * eef_position_local;
                    obj.endEffectorJacobians{i_eef}(1:3, i_joint) = current_column(1:3) * obj.branchMatrix(i_eef, i_joint);
                end
            end
        end
        function calculateEndEffectorVelocities(obj)
            for i_eef = 1 : obj.numberOfBranches
                obj.endEffectorVelocities{i_eef} = obj.endEffectorJacobians{i_eef} * obj.jointVelocities;
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
                % first element of this column, i=j+1
                obj.interTransformations{j_joint+1, j_joint} = obj.twistExponentials{j_joint+1};
                % rest of the column
                for i_joint = j_joint+2 : obj.numberOfJoints
                    obj.interTransformations{i_joint, j_joint} = obj.interTransformations{i_joint-1, j_joint} * obj.twistExponentials{i_joint};
                end
            end
        end
        function calculateInterAdjoints(obj)
            % calculate transformations that are non-zero and non-identity
            for i_joint = 1 : obj.numberOfJoints
                for j_joint = 1 : obj.numberOfJoints
                    obj.interAdjoints{i_joint, j_joint} = createAdjointTransformation(obj.interTransformations{i_joint, j_joint});
                end
            end
        end
        function calculateInverseInterAdjoints(obj)
            % calculate transformations that are non-zero and non-identity
            for i_joint = 1 : obj.numberOfJoints
                for j_joint = 1 : obj.numberOfJoints
                    obj.inverseInterAdjoints{i_joint, j_joint} = invertAdjoint(obj.interAdjoints{i_joint, j_joint});
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
                        twist = twistCoordinatesToMatrix(obj.spatialJacobian(1:6, j_joint));
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
                obj.gravitationalTorqueMatrix(k_joint) = 9.81 * N_k;
            end
        end
    end
end


































