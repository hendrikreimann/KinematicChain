% class for the display of a stick figure

classdef KinematicTreeStickFigure < handle
    properties
        kinematicTree;
        sceneFigure;
        sceneAxes;
        sceneBound;
        jointPlots;
        linkPlots;
        linkCenterPlots;
        endEffectorPlots;
        endEffectorLinkPlots;
        endEffectorVelocityPlots;
        miscellaneousPlots;
        markerPlots;
        fixedMarkerPlots;
        markerConnectionLinePlots;
        linkMassEllipsoids;
        linkMassEllipsoidSurfs;
        linkMassEllipsoidResolution = 15;
        
        
        % flags
        showLinkCentersOfMass = false;
        showLinkMassEllipsoids = false;
        showStickFigure = true;
        showMarkers = true;
    end 
    methods
        function obj = KinematicTreeStickFigure(kinematicTree, sceneBound)
            obj.kinematicTree = kinematicTree;
            kinematicTree.updateLinkVisualizationData();
            if nargin < 2
                obj.sceneBound = [-1; 1; -1; 1; -1; 1];
            else
                obj.sceneBound = sceneBound;
            end
            obj.sceneFigure = figure( 'Position', [ 1250, 1100, 400, 400 ], 'Name', 'scene' );
            obj.sceneAxes = axes( 'Position', [ 0.1 0.1 0.8 0.8 ]);
            axis equal; hold on;
            plot3([obj.sceneBound(1), obj.sceneBound(2)], [0, 0], [0, 0], 'color', 'k','Linewidth', 1, 'Linestyle',':');
            plot3([0, 0], [obj.sceneBound(3), obj.sceneBound(4)], [0, 0], 'color', 'k','Linewidth', 1, 'Linestyle',':');
            plot3([0, 0], [0, 0], [obj.sceneBound(5), obj.sceneBound(6)], 'color', 'k','Linewidth', 1, 'Linestyle',':');

            % set up joint plots
            obj.jointPlots = zeros(1, kinematicTree.numberOfJoints);
            for i_joint = 1 : kinematicTree.numberOfJoints
                obj.jointPlots(i_joint) = plot(0, 0, 'color', 'b', 'Linewidth', 3, 'Marker', 'o');
            end
            
            % set up link plots
            obj.linkPlots = zeros(1, kinematicTree.numberOfJoints);
            for i_joint = 1 : kinematicTree.numberOfJoints
                obj.linkPlots(i_joint) = plot([0, 0], [0, 0], 'color', 'b', 'Linewidth', 2, 'Linestyle', '-');
                obj.linkCenterPlots(i_joint) = plot([0, 0], [0, 0], 'color', 'm', 'Linewidth', 2, 'Marker', 'x');
            end
            % TODO: port the support for arbitrary lines from the KinematicTree
            
            % set up eef plots
            for i_eef = 1 : kinematicTree.numberOfBranches
                obj.endEffectorPlots(i_eef) = plot(0, 0, 'color', 'b', 'Linewidth', 3, 'Marker', 'o');
                obj.endEffectorLinkPlots(i_eef) = plot([0, 0], [0, 0], 'color', 'b', 'Linewidth', 2, 'Linestyle', '-');
                obj.endEffectorVelocityPlots(i_eef) = plot([0, 0], [0, 0], 'color', 'c', 'Linewidth', 1, 'Linestyle', '-');
            end
            
            % set up miscellaneous plots
            for i_line = 1 : size(kinematicTree.miscellaneousLinesStartPoints, 2)
                obj.miscellaneousPlots(i_line) = plot3([0 0], [0 0], [0 0], 'color', 'c', 'Linewidth', 1, 'Linestyle', '-');
            end
            
            % set up marker plots
            obj.markerPlots = cell(kinematicTree.numberOfJoints, 1);
%             marker_index = 0;
            for i_marker = 1 : kinematicTree.getNumberOfMarkers(0)
%                 marker_index = marker_index + 1;
                obj.fixedMarkerPlots(i_marker) = plot3(0, 0, 0, 'color', obj.kinematicTree.getMarkerVisualizationColor(0, i_marker), 'Linewidth', 2, 'Marker', 'h');
            end
            for i_joint = 1 : kinematicTree.numberOfJoints
                for i_marker = 1 : size(kinematicTree.markerPositions{i_joint}, 2)
%                     marker_index = marker_index + 1;
                    obj.markerPlots{i_joint}(i_marker) = plot3(0, 0, 0, 'color', obj.kinematicTree.getMarkerVisualizationColor(i_joint, i_marker), 'Linewidth', 2, 'Marker', 'h');
                end
            end
            for i_line = 1 : size(kinematicTree.markerConnectionLineIndices, 1)
                obj.markerConnectionLinePlots(i_line) = plot3([0 0], [0 0], [0 0], 'k', 'linewidth', 1);
            end
            
            % set up link mass ellipsoids
            for i_joint = 1 : kinematicTree.numberOfJoints
                m = kinematicTree.linkMasses(i_joint);
                I_a = kinematicTree.generalizedInertiaMatrices{i_joint}(4, 4);
                I_b = kinematicTree.generalizedInertiaMatrices{i_joint}(5, 5);
                I_c = kinematicTree.generalizedInertiaMatrices{i_joint}(6, 6);
                a = sqrt((5/(2*m)*(- I_a + I_b + I_c)));
                b = sqrt((5/(2*m)*(+ I_a - I_b + I_c)));
                c = sqrt((5/(2*m)*(+ I_a + I_b - I_c)));
                
                [x,y,z] = ...
                  ellipsoid ...
                  ( ...
                    0, 0, 0, ...
                    a, b, c, ...
                    obj.linkMassEllipsoidResolution ...
                  );
                ellipsoid_data = ones(4, obj.linkMassEllipsoidResolution+1, obj.linkMassEllipsoidResolution+1);
                ellipsoid_data(1, :, :) = x;
                ellipsoid_data(2, :, :) = y;
                ellipsoid_data(3, :, :) = z;
                
                obj.linkMassEllipsoids{i_joint} = ellipsoid_data;
                
                obj.linkMassEllipsoidSurfs(i_joint) = surf(x, y, z);
            end
            
            
            
            
            % groom
            set(gca,'xlim',[obj.sceneBound(1), obj.sceneBound(2)],'ylim',[obj.sceneBound(3), obj.sceneBound(4)]);
            xlabel('x');
            ylabel('y');
            zlabel('z');
            
            colormap cool
            alpha(.4)
        end
        
        function update(obj)
            for i_joint = 1 : obj.kinematicTree.numberOfJoints
                if obj.showStickFigure
                    set( ...
                         obj.jointPlots(i_joint), ...
                         'Xdata', obj.kinematicTree.jointPositions{i_joint}(1), ...
                         'Ydata', obj.kinematicTree.jointPositions{i_joint}(2), ...
                         'Zdata', obj.kinematicTree.jointPositions{i_joint}(3) ...
                       )
                else
                    set(obj.jointPlots(i_joint), 'visible', 'off');
                end
                if obj.kinematicTree.linkMasses(i_joint) > 0 && obj.showLinkCentersOfMass
                    set( ...
                         obj.linkCenterPlots(i_joint), ...
                         'Xdata', obj.kinematicTree.linkTransformations{i_joint}(1, 4), ...
                         'Ydata', obj.kinematicTree.linkTransformations{i_joint}(2, 4), ...
                         'Zdata', obj.kinematicTree.linkTransformations{i_joint}(3, 4) ...
                       )
                else
                    set(obj.linkCenterPlots(i_joint), 'visible', 'off');
                end
            end
            for i_joint = 2 : obj.kinematicTree.numberOfJoints
                if obj.showStickFigure
                    set( ...
                         obj.linkPlots(i_joint), ...
                         'Xdata', [obj.kinematicTree.jointPositions{obj.kinematicTree.jointParents(i_joint)}(1), obj.kinematicTree.jointPositions{i_joint}(1)], ...
                         'Ydata', [obj.kinematicTree.jointPositions{obj.kinematicTree.jointParents(i_joint)}(2), obj.kinematicTree.jointPositions{i_joint}(2)], ...
                         'Zdata', [obj.kinematicTree.jointPositions{obj.kinematicTree.jointParents(i_joint)}(3), obj.kinematicTree.jointPositions{i_joint}(3)] ...
                       )
                else
                    set(obj.linkPlots(i_joint), 'visible', 'off');
                end
            end
            
            for i_eef = 1 : obj.kinematicTree.numberOfBranches
                if obj.showStickFigure
                    set(obj.endEffectorPlots(i_eef), ...
                            'Xdata', obj.kinematicTree.endEffectorPositions{i_eef}(1), ...
                            'Ydata', obj.kinematicTree.endEffectorPositions{i_eef}(2), ...
                            'Zdata', obj.kinematicTree.endEffectorPositions{i_eef}(3))
                    set( ...
                         obj.endEffectorLinkPlots(i_eef), ...
                         'Xdata', [obj.kinematicTree.jointPositions{obj.kinematicTree.endEffectorParents(i_eef)}(1), obj.kinematicTree.endEffectorPositions{i_eef}(1)], ...
                         'Ydata', [obj.kinematicTree.jointPositions{obj.kinematicTree.endEffectorParents(i_eef)}(2), obj.kinematicTree.endEffectorPositions{i_eef}(2)], ...
                         'Zdata', [obj.kinematicTree.jointPositions{obj.kinematicTree.endEffectorParents(i_eef)}(3), obj.kinematicTree.endEffectorPositions{i_eef}(3)] ...
                       )
                    set( ...
                         obj.endEffectorVelocityPlots(i_eef), ...
                         'Xdata', [obj.kinematicTree.endEffectorPositions{i_eef}(1), obj.kinematicTree.endEffectorPositions{i_eef}(1) + obj.kinematicTree.endEffectorVelocities{i_eef}(1)], ...
                         'Ydata', [obj.kinematicTree.endEffectorPositions{i_eef}(2), obj.kinematicTree.endEffectorPositions{i_eef}(2) + obj.kinematicTree.endEffectorVelocities{i_eef}(2)], ...
                         'Zdata', [obj.kinematicTree.endEffectorPositions{i_eef}(3), obj.kinematicTree.endEffectorPositions{i_eef}(3) + obj.kinematicTree.endEffectorVelocities{i_eef}(3)] ...
                       )
                else
                    set(obj.endEffectorPlots(i_eef), 'visible', 'off');
                    set(obj.endEffectorLinkPlots(i_eef), 'visible', 'off');
                    set(obj.endEffectorVelocityPlots(i_eef), 'visible', 'off');
                end
            end
            
            % update segment mass ellipsoids
            for i_joint = 1 : obj.kinematicTree.numberOfJoints
                if obj.showLinkMassEllipsoids && (obj.kinematicTree.linkMasses(i_joint) > 0)
                    % transform ellipsoid
                    link_transformation = obj.kinematicTree.linkTransformations{i_joint};
                    ellipsoid_transformed = zeros(size(obj.linkMassEllipsoids{i_joint}));
                    for i_point = 1 : obj.linkMassEllipsoidResolution + 1
                        for j_point = 1 : obj.linkMassEllipsoidResolution + 1
                            point = squeeze(obj.linkMassEllipsoids{i_joint}(:, i_point, j_point));
                            ellipsoid_transformed(:, i_point, j_point) = link_transformation * point;
                        end
                    end
                    
                    set( ...
                         obj.linkMassEllipsoidSurfs(i_joint), ...
                         'Xdata', squeeze(ellipsoid_transformed(1, :, :)), ...
                         'Ydata', squeeze(ellipsoid_transformed(2, :, :)), ...
                         'Zdata', squeeze(ellipsoid_transformed(3, :, :)) ...
                       );
                else
                    set(obj.linkMassEllipsoidSurfs(i_joint), 'visible', 'off');
                end
            end
                
            % update marker plots
            for i_marker = 1 : obj.kinematicTree.getNumberOfMarkers(0)
                if obj.showMarkers
                    position = obj.kinematicTree.getMarkerPosition(0, i_marker);
                    set( ...
                         obj.fixedMarkerPlots(i_marker), ...
                            'Xdata', position(1), ...
                            'Ydata', position(2), ...
                            'Zdata', position(3) ...
                       )
                else
                    set(obj.fixedMarkerPlots(i_marker), 'visible', 'off');
                end
            end
            for i_joint = 1 : obj.kinematicTree.numberOfJoints
                for i_marker = 1 : obj.kinematicTree.getNumberOfMarkers(i_joint)
                    if obj.showMarkers
                        position = obj.kinematicTree.getMarkerPosition(i_joint, i_marker);
                        set( ...
                             obj.markerPlots{i_joint}(i_marker), ...
                                'Xdata', position(1), ...
                                'Ydata', position(2), ...
                                'Zdata', position(3) ...
                           )
                    else
                        set(obj.markerPlots{i_joint}(i_marker), 'visible', 'off');
                    end
                end
            end
            for i_line = 1 : length(obj.markerConnectionLinePlots)
                if obj.showMarkers
                    joint_index_marker_one = obj.kinematicTree.markerExportMap(1, obj.kinematicTree.markerConnectionLineIndices(i_line, 1));
                    marker_index_marker_one = obj.kinematicTree.markerExportMap(2, obj.kinematicTree.markerConnectionLineIndices(i_line, 1));
                    joint_index_marker_two = obj.kinematicTree.markerExportMap(1, obj.kinematicTree.markerConnectionLineIndices(i_line, 2));
                    marker_index_marker_two = obj.kinematicTree.markerExportMap(2, obj.kinematicTree.markerConnectionLineIndices(i_line, 2));
                    position_marker_one = obj.kinematicTree.getMarkerPosition(joint_index_marker_one, marker_index_marker_one);
                    position_marker_two = obj.kinematicTree.getMarkerPosition(joint_index_marker_two, marker_index_marker_two);
                    set( ...
                         obj.markerConnectionLinePlots(i_line), ...
                            'Xdata', [position_marker_one(1) position_marker_two(1)], ...
                            'Ydata', [position_marker_one(2) position_marker_two(2)], ...
                            'Zdata', [position_marker_one(3) position_marker_two(3)] ...
                       )
                else
                    set(obj.markerConnectionLinePlots(i_line), 'visible', 'off');
                end
            end
            
            
        end
        function setMiscellaneousPlotColor(obj, index, color)
            if index > length(obj.miscellaneousPlots)
                error('index larger than number of miscellaneous plots')
            else
                obj.miscellaneousPlots(index) = plot3(obj.sceneAxes, [0 1], [0 1], [0 0], 'color', color, 'Linewidth', 1, 'Linestyle', '-');
                obj.update();
            end
        end
        function setMarkerColor(obj, joint_index, marker_index, color)
            set(obj.markerPlots{joint_index}(marker_index), 'color', color);
            obj.update();
        end
        function color = getMarkerColor(obj, joint_index, marker_index)
            if joint_index == 0
                color = get(obj.fixedMarkerPlots(marker_index), 'color');
            else
                color = get(obj.markerPlots{joint_index}(marker_index), 'color');
            end
        end
    end
end