


classdef KinematicChain < handle
    properties
        % reference data
        numberOfJoints;
        
        % kinematic variables
        jointAngles;
        jointVelocities;
        jointAccelerations;
        externalTorques;
        constraintTorques;
        
        % dynamic variables
        linkMasses;
        inertiaMatrix;
        coriolisMatrix;
        gravitationalTorqueMatrix;
        
        % dependent variables - joints
        jointPositions;
        
        % dependent variables - markers
        markerPositions;
        
        % dependent variables - end-effector
        endEffectorPosition
        endEffectorVelocity
        endEffectorAcceleration
        endEffectorJacobian
        endEffectorJacobianTemporalDerivative
        
        % visualization data
        linkVisualizationData
        miscellaneousLinesStartPoints
        miscellaneousLinesEndPoints
    end
    methods (Abstract)
        updateInternals(obj)
    end
    methods
        function obj = KinematicChain(degreesOfFreedom)
            obj.numberOfJoints = degreesOfFreedom;

            obj.jointAngles = zeros(degreesOfFreedom, 1);
            obj.jointVelocities = zeros(degreesOfFreedom, 1);
            obj.jointAccelerations = zeros(degreesOfFreedom, 1);
            obj.externalTorques = zeros(degreesOfFreedom, 1);
            obj.constraintTorques = zeros(degreesOfFreedom, 1);
            
            obj.inertiaMatrix = zeros(degreesOfFreedom);
            obj.coriolisMatrix = zeros(degreesOfFreedom);
            obj.gravitationalTorqueMatrix = zeros(degreesOfFreedom, 1);
            
            obj.jointPositions = cell(degreesOfFreedom, 1);
            
            obj.endEffectorPosition = zeros(3, 1);
            obj.endEffectorVelocity = zeros(3, 1);
            obj.endEffectorAcceleration = zeros(3, 1);
            obj.endEffectorJacobian = zeros(3, degreesOfFreedom);
            obj.endEffectorJacobianTemporalDerivative = zeros(3, degreesOfFreedom);

            % generate marker data container
            obj.markerPositions = cell(obj.numberOfJoints, 1);

            % generate link visualization data
            % XXX this does not work for the general case yet, just making
            % sure that there is a skeleton data structure
            obj.linkVisualizationData = struct([]);
            for i_joint = 1 : obj.numberOfJoints
                start_point = zeros(3, 1);
                end_point = zeros(3, 1);
                obj.linkVisualizationData(i_joint).startPoints(:, 1) = start_point;
                obj.linkVisualizationData(i_joint).endPoints(:, 1) = end_point;
            end
            obj.miscellaneousLinesStartPoints = [];
            obj.miscellaneousLinesEndPoints = [];
            
        end
        function obj = calculateAccelerationsFromExternalTorques(obj)
            obj.jointAccelerations = obj.inertiaMatrix^(-1) * ...
                                          (obj.externalTorques ...
                                            - obj.constraintTorques ...
                                            - obj.gravitationalTorqueMatrix ...
                                            - obj.coriolisMatrix * obj.jointVelocities ...
                                          );
            
        end
    
    end
end