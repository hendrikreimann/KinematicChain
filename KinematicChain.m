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

classdef KinematicChain < handle
    properties
        % reference data
        numberOfJoints;
        
        % kinematic variables
        jointAngles;
        jointVelocities;
        jointAccelerations;
        
        % dynamic variables
        linkMasses;
        inertiaMatrix;
        coriolisMatrix;
        gravitationalTorqueMatrix;
        externalTorques;
        constraintTorques;
        
        % dependent variables - joints
        jointPositions;
        
        % dependent variables - end-effector
        endEffectorPosition
        endEffectorVelocity
        endEffectorAcceleration
        endEffectorJacobian
        endEffectorJacobianTemporalDerivative
        
        % markers
        markerPositions;
        markerExportMap;
        
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
        function new = copy(this) % Make a deep copy of this object
            % Instantiate new object of the same class.
            new = feval(class(this));
 
            % Copy all non-hidden properties.
            p = properties(this);
            for i = 1:length(p)
                new.(p{i}) = this.(p{i});
            end
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

















