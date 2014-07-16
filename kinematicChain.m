


classdef kinematicChain < handle
    properties
        % reference data
        numberOfJoints;
        
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
        endEffectorPosition;
        endEffectorVelocity;
        endEffectorAcceleration;
        endEffectorJacobian;
        
    end
    methods (Abstract)
        updateInternals(obj)
    end
    methods
        function obj = kinematicChain(degreesOfFreedom)
            obj.numberOfJoints = degreesOfFreedom;

            obj.jointAngles = zeros(degreesOfFreedom, 1);
            obj.jointVelocities = zeros(degreesOfFreedom, 1);
            obj.jointAccelerations = zeros(degreesOfFreedom, 1);
            obj.externalTorques = zeros(degreesOfFreedom, 1);
            
            obj.inertiaMatrix = zeros(degreesOfFreedom);
            obj.coriolisMatrix = zeros(degreesOfFreedom);
            obj.gravitationalTorqueMatrix = zeros(degreesOfFreedom, 1);
            
            obj.jointPositions = cell(degreesOfFreedom, 1);
            
            obj.endEffectorPosition = zeros(3, 1);
            obj.endEffectorVelocity = zeros(3, 1);
            obj.endEffectorAcceleration = zeros(3, 1);
            obj.endEffectorJacobian = zeros(3, degreesOfFreedom);
            
            
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