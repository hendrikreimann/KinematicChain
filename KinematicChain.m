


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
        
        % markers
        markerPositions;
        markerExportMap;
        
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
            obj.markerExportMap = [];

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
        function numberOfMarkers = getNumberOfMarkers(obj, jointIndex)
            if nargin < 2
                numberOfMarkers = size(obj.markerExportMap, 2);
            else
                joint_occurrences = (obj.markerExportMap(1, :) == jointIndex);
                numberOfMarkers = sum(joint_occurrences);
            end
        end
        function markerPositions = exportMarkerPositions(obj)
            number_of_markers = size(obj.markerExportMap, 2);
            markerPositions = zeros(1, number_of_markers*3);
            for i_marker = 1 : number_of_markers
                marker_joint = obj.markerExportMap(1, i_marker);
                marker_index = obj.markerExportMap(2, i_marker);
                marker_position = obj.markerPositions{marker_joint}(1:3, marker_index);
                markerPositions(1, 3*(i_marker - 1) + 1 : 3*(i_marker - 1) + 3) = marker_position';                
            end
        end
        function exportMapIndices = getMarkerExportIndices(obj, jointIndex, markerIndex)
            if nargin < 3
                % return indices for all markers at this segment
                export_index_map = (obj.markerExportMap(1, :) == jointIndex);
                export_indices = find(export_index_map);
                number_of_markers = length(export_indices);
                exportMapIndices = zeros(1, number_of_markers*3);
                for i_marker = 1 : number_of_markers
                    exportMapIndices(3*(i_marker-1)+1 : 3*(i_marker-1)+3) ...
                        = 3*(export_indices(i_marker)-1)+1 : 3*(export_indices(i_marker)-1)+3;
                end
            else
                % return indices for the specified marker
                joint_occurrences = (obj.markerExportMap(1, :) == jointIndex);
                marker_occurrences = (obj.markerExportMap(2, :) == markerIndex);
                export_index_map = joint_occurrences .* marker_occurrences;
                export_index = find(export_index_map);
                exportMapIndices = 3*(export_index-1)+1 : 3*(export_index-1)+3;
            end
        end
    end
end

















