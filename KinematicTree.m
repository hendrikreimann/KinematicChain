classdef KinematicTree < handle
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
        endEffectorPositions;
        endEffectorVelocities;
        endEffectorAccelerations;
        endEffectorJacobians;
        endEffectorOrientationJacobians;
        endEffectorJacobianTemporalDerivatives;
        
        % markers
        fixedMarkerPositions;
        markerPositions;
        markerExportMap;
        
        % visualization data
        linkVisualizationData
        fixedMarkerVisualizationColors
        markerVisualizationColors
        miscellaneousLinesStartPoints
        miscellaneousLinesEndPoints
        
        % adjustable gravitational constant
        standardGravity = 9.80665;
        
        % labels
        jointLabels;
        
    end
    methods (Abstract)
        updateInternals(obj)
    end
    methods
        function obj = KinematicTree(degreesOfFreedom, branchMatrix)
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
            obj.constraintTorques = zeros(degreesOfFreedom, 1);
            
            obj.inertiaMatrix = zeros(degreesOfFreedom);
            obj.coriolisMatrix = zeros(degreesOfFreedom);
            obj.gravitationalTorqueMatrix = zeros(degreesOfFreedom, 1);
            
            obj.jointPositions = cell(degreesOfFreedom, 1);
            
            obj.endEffectorPositions = cell(obj.numberOfBranches, 1);
            obj.endEffectorVelocities = cell(obj.numberOfBranches, 1);
            obj.endEffectorAccelerations = cell(obj.numberOfBranches, 1);
            obj.endEffectorJacobians = cell(obj.numberOfBranches, 1);
            obj.endEffectorOrientationJacobians = cell(obj.numberOfBranches, 1);
            obj.endEffectorJacobianTemporalDerivatives = cell(obj.numberOfBranches, 1);
            
            % generate marker data container
            obj.markerPositions = cell(obj.numberOfJoints, 1);
            obj.fixedMarkerPositions = [];
            obj.markerExportMap = [];
            
            % marker colors
            obj.fixedMarkerVisualizationColors = [];
            obj.markerVisualizationColors = cell(obj.numberOfJoints, 1);
            
            
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
                if isempty(obj.markerExportMap)
                    numberOfMarkers = 0;
                else
                    joint_occurrences = (obj.markerExportMap(1, :) == jointIndex);
                    numberOfMarkers = sum(joint_occurrences);
                end
            end
        end
        function markerPositions = exportMarkerPositions(obj)
            number_of_markers = size(obj.markerExportMap, 2);
            markerPositions = zeros(1, number_of_markers*3);
            for i_marker = 1 : number_of_markers
                marker_joint = obj.markerExportMap(1, i_marker);
                marker_index = obj.markerExportMap(2, i_marker);
                if marker_joint == 0
                    marker_position = obj.fixedMarkerPositions(1:3, marker_index);
                else
                    marker_position = obj.markerPositions{marker_joint}(1:3, marker_index);
                end
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
        function position = getMarkerPosition(obj, jointIndex, markerIndex)
            if jointIndex == 0
                position = obj.fixedMarkerPositions(:, markerIndex);
            else
                position = obj.markerPositions{jointIndex}(:, markerIndex);
            end
        end
    end
end