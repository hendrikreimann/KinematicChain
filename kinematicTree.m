


classdef kinematicTree < handle
    properties
        % reference data
        numberOfJoints;
        numberOfBranches;
        branchMatrix;
        jointParents;
        endEffectorParents;
        connectivityMatrix; % C_ij = 1 if the j-the joint twist is affected by movement in the i-th joint
        
        % kinematic variables
        jointAngles;
        jointVelocities;
        jointAccelerations;
        externalTorques;
        
        % dynamic variables
        linkMasses;
        inertiaMatrix;
        coriolisMatrix;
        gravitationalTorqueMatrix;
        
        % dependent variables - joints
        jointPositions;
        
        % dependent variables - end-effector
        endEffectorPositions;
        endEffectorVelocities;
        endEffectorAccelerations;
        endEffectorJacobians;
        
    end
    methods (Abstract)
        updateInternals(obj)
    end
    methods
        function obj = kinematicTree(degreesOfFreedom, branchMatrix)
            obj.numberOfJoints = degreesOfFreedom;
            obj.numberOfBranches = size(branchMatrix, 1);
            obj.branchMatrix = branchMatrix;
            obj.connectivityMatrix = zeros(degreesOfFreedom, degreesOfFreedom);
            % fill connectivity matrix
            for i_moving_joint = 1 : degreesOfFreedom
                for i_moved_joint = i_moving_joint + 1 : degreesOfFreedom
                    % is there a branch that contains both the moving and the moved joints?
                    common_branch = 0;
                    for i_branch = 1 : obj.numberOfBranches
                        if (obj.branchMatrix(i_branch, i_moving_joint) == 1) && (obj.branchMatrix(i_branch, i_moved_joint) == 1)
                            common_branch = 1;
                        end
                    end
                    if common_branch
                        obj.connectivityMatrix(i_moving_joint, i_moved_joint) = 1;
                    end
                end
            end
            
            
            % fill parent matrices
            obj.jointParents = zeros(degreesOfFreedom, 1);
            for i_joint = 1 : degreesOfFreedom
                parent = 0;
                i_parent = 1;
                while i_parent < i_joint
                    if obj.connectivityMatrix(i_parent, i_joint) == 1
                        parent = i_parent;
                    end
                    i_parent = i_parent + 1;
                end
                obj.jointParents(i_joint) = parent;
            end
            obj.endEffectorParents = zeros(obj.numberOfBranches, 1);
            for i_eef = 1 : obj.numberOfBranches
                parent = 0;
                for i_parent = 1 : obj.numberOfJoints
                    if obj.branchMatrix(i_eef, i_parent) == 1
                        parent = i_parent;
                    end
                end
                obj.endEffectorParents(i_eef) = parent;
            end
            
            

            obj.jointAngles = zeros(degreesOfFreedom, 1);
            obj.jointVelocities = zeros(degreesOfFreedom, 1);
            obj.jointAccelerations = zeros(degreesOfFreedom, 1);
            obj.externalTorques = zeros(degreesOfFreedom, 1);
            
            obj.inertiaMatrix = zeros(degreesOfFreedom);
            obj.coriolisMatrix = zeros(degreesOfFreedom);
            obj.gravitationalTorqueMatrix = zeros(degreesOfFreedom, 1);
            
            obj.jointPositions = cell(degreesOfFreedom, 1);
            
            obj.endEffectorPositions = cell(obj.numberOfBranches, 1);
            obj.endEffectorVelocities = cell(obj.numberOfBranches, 1);
            obj.endEffectorAccelerations = cell(obj.numberOfBranches, 1);
            obj.endEffectorJacobians = cell(obj.numberOfBranches, 1);
            
            
        end
        function obj = calculateAccelerationsFromExternalTorques(obj)
            obj.jointAccelerations = obj.inertiaMatrix^(-1) * ...
                                          (obj.externalTorques ...
                                            - obj.gravitationalTorqueMatrix ...
                                            - obj.coriolisMatrix * obj.jointVelocities ...
                                          );
            
        end
    
    end
end