


classdef generalKinematicChain < kinematicChain
    properties
        % reference data
        referenceJointTwists;
        referenceJointTransformations;
        referenceEndEffectorTransformation;
        referenceLinkTransformations;
        transformedLinkInertiaMatrices;
        generalizedInertiaMatrices;
        
        % screw geometry data
        twistExponentials
        productsOfExponentials
        jointTransformations
        endEffectorTransformation
        linkTransformations
        interTransformations
        interAdjoints
        inverseInterAdjoints
        inertiaMatrixPartialDerivatives
        spatialJacobian
        spatialJacobianTemporalDerivative
        linkJacobians
    end
    methods
        function obj = generalKinematicChain(jointPositions, jointAxes, endEffectorPosition, linkCenters, linkMasses, linkMomentsOfInertia)
            degrees_of_freedom = length(jointPositions);
            obj = obj@kinematicChain(degrees_of_freedom);
            obj.linkMasses = linkMasses;
            
            % generate references
            for i_joint = 1 : degrees_of_freedom
                obj.referenceJointTransformations{i_joint} = createReferenceTransformation(jointPositions{i_joint}, eye(3));
                obj.referenceJointTwists{i_joint} = createTwist(jointPositions{i_joint}, jointAxes{i_joint});
            end
            obj.referenceEndEffectorTransformation = createReferenceTransformation(endEffectorPosition, eye(3));
            
            if nargin > 3
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
            obj.calculateEndEffectorTransformation();
            
            % update Jacobians
            obj.calculateSpatialJacobian();
            obj.calculateEndEffectorJacobian();
            
            % update dependent variables
            for i_joint = 1 : obj.numberOfJoints
                obj.jointPositions{i_joint} = obj.jointTransformations{i_joint}(1:3, 4);
            end
            obj.endEffectorPosition = obj.endEffectorTransformation(1:3, 4);
            obj.endEffectorVelocity = obj.endEffectorJacobian * obj.jointVelocities;
            
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
            
            % update second-order temporal derivatives
            obj.calculateSpatialJacobianTemporalDerivative();
            obj.calculateEndEffectorAcceleration();
        end
        function calculateTwistExponentials(obj)
            % calculate the twist exponentials from the current joint angles and the reference twists
            for i_joint = 1 : obj.numberOfJoints
                obj.twistExponentials{i_joint} = twistExponential(obj.referenceJointTwists{i_joint}, obj.jointAngles(i_joint));
            end
        end
        function calculateProductsOfExponentials(obj)
        % calculates the products of exponentials from the current twist exponentials
            obj.productsOfExponentials{1} = obj.twistExponentials{1};
            for i_joint = 2 : obj.numberOfJoints
                obj.productsOfExponentials{i_joint} = obj.productsOfExponentials{i_joint-1} * obj.twistExponentials{i_joint};
            end
        end
        function calculateJointTransformations(obj)
            % calculates the transformations between world and joint coordinate frames
            for i_joint = 1 : obj.numberOfJoints
                obj.jointTransformations{i_joint} = obj.productsOfExponentials{i_joint} * obj.referenceJointTransformations{i_joint};
            end
        end
        function calculateEndEffectorTransformation(obj)
            % calculates the transformations between world and joint end-effector coordinate frames
            obj.endEffectorTransformation = obj.productsOfExponentials{obj.numberOfJoints} * obj.referenceEndEffectorTransformation;
        end
        function calculateSpatialJacobian(obj)
            % first column
            obj.spatialJacobian(1:6, 1) = obj.referenceJointTwists{1};
            % child columns
            for i_joint = 2 : obj.numberOfJoints
                adjoint = createAdjointTransformation(obj.productsOfExponentials{i_joint-1});
                obj.spatialJacobian(1:6, i_joint) = adjoint * obj.referenceJointTwists{i_joint};
            end
        end
        function calculateEndEffectorJacobian(obj)
            eef_position_local = [0; 0; 0; 1]; % end-effector position in local coordinates
            for i_joint = 1 : obj.numberOfJoints
                joint_twist = twistCoordinatesToMatrix(obj.spatialJacobian(:, i_joint));
                current_column = joint_twist * obj.endEffectorTransformation * eef_position_local;
                obj.endEffectorJacobian(1:3, i_joint) = current_column(1:3);
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
                    for l_joint = max(i_joint, j_joint) : obj.numberOfJoints
                        p = obj.referenceJointTwists{i_joint}' ...
                            * obj.inverseInterAdjoints{l_joint, i_joint}' ...
                            * obj.transformedLinkInertiaMatrices{l_joint} ...
                            * obj.inverseInterAdjoints{l_joint, j_joint} ...
                            * obj.referenceJointTwists{j_joint};
                        m_ij =  m_ij + p;           
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

                        % calculate $\partial M_{ij} \by \partial \theta_k$
                        A_kiTimesXi_i = obj.inverseInterAdjoints{k_joint, i_joint} * obj.referenceJointTwists{i_joint};
                        lieBracket1 = twistLieBracket(A_kiTimesXi_i, obj.referenceJointTwists{k_joint});
                        A_kjTimesXi_j = obj.inverseInterAdjoints{k_joint, j_joint} * obj.referenceJointTwists{j_joint};
                        lieBracket2 = twistLieBracket(A_kjTimesXi_j, obj.referenceJointTwists{k_joint});
                        % add up the derivatives
                        for l_joint = max(i_joint, j_joint) : obj.numberOfJoints
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
                    twist = twistCoordinatesToMatrix(obj.spatialJacobian(1:6, j_joint));
                    column = twist * position;
                    jacobian(1:3, j_joint) = column(1:3);
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
        function calculateSpatialJacobianTemporalDerivative(obj)
            for j_joint = 1 : obj.numberOfJoints
                % g = exp(xiHat_1*theta_1)*...*exp(xiHat_{j-1}*theta_{j-1}) 
                %       * xiHat_j 
                %       * exp(-xiHat_{j-1}*theta_{j-1})*...*exp(xiHat_1*theta_1)
                % this is a product with 2 * (j-1) factors depending upon time
                % its derivative is a sum with 2 * (j-1) summands

                g = zeros(4, 4);
                for k_joint = 1 : j_joint-1
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % the k-th summand, deriving the factor with positive sign theta_k
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    s_k = eye(4, 4); % the summand where the factor with the positive sign theta_k is derived

                    % factors before the k-th
                    for i_joint = 1 : k_joint-1
                        % i-th factor stays the same for i < k
                        s_k = s_k * obj.twistExponentials{i_joint};

                    end
                    % k-th factor is derived by time
                    s_k = s_k * twistCoordinatesToMatrix(obj.referenceJointTwists{k_joint}) * obj.twistExponentials{k_joint};
                    % factors after the k-th
                    for i_joint = k_joint+1 : j_joint-1
                        % i-th factor stays the same for i > k
                        s_k = s_k * obj.twistExponentials{i_joint};

                %         if (jointIndex == 3) && (k == 2)
                %             disp(['j = ' num2str(i)]);
                %             disp(num2str(s_k));
                %         end
                    end
                    s_k = s_k * twistCoordinatesToMatrix(obj.referenceJointTwists{j_joint}) * obj.productsOfExponentials{j_joint-1}^(-1);

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % the (2*(j-1)-k)-th summand, deriving the factor with negative sign theta_k
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    t_k = obj.productsOfExponentials{j_joint-1} * twistCoordinatesToMatrix(obj.referenceJointTwists{j_joint}); 
                    % the summand where the factor with the negative sign theta_k is derived
                    for i_joint = j_joint-1 : -1 : k_joint+1
                        % i-th factor stays the same for i > k
                        t_k = t_k * obj.twistExponentials{i_joint}^(-1);
                    end
                    t_k = t_k * twistCoordinatesToMatrix(obj.referenceJointTwists{k_joint}) * obj.twistExponentials{k_joint}^(-1);
                    for i_joint = k_joint-1 : -1 : 1
                        % i-th factor stays the same for i < k
                        t_k = t_k * obj.twistExponentials{i_joint}^(-1);
                    end

                    % add this summand to the sum
                    g = g + obj.jointVelocities(k_joint)*(s_k - t_k);
                end

                obj.spatialJacobianTemporalDerivative(:, j_joint) = twistMatrixToCoordinates(g);
            end
        end
        function calculateEndEffectorAcceleration(obj)
            T1 = obj.spatialJacobianTemporalDerivative * obj.jointVelocities;
            T2 = obj.spatialJacobian * obj.jointAccelerations;
            S1 = twistCoordinatesToMatrix(T1 + T2) * [obj.endEffectorPosition; 1];
            S2 = twistCoordinatesToMatrix(obj.spatialJacobian * obj.jointVelocities) * [obj.endEffectorVelocity; 0];
            end_effector_acceleration = S1 + S2;
            obj.endEffectorAcceleration = end_effector_acceleration(1:3);
        end        
    end
end