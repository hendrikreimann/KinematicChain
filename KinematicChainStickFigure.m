% class for the display of a stick figure

classdef KinematicChainStickFigure < handle
    properties
        kinematicChain;
        sceneFigure;
        sceneAxes;
        sceneBound;
        jointPlots;
        linkPlots;
        linkCenterPlots;
        endEffectorPlot;
        miscellaneousPlots;
        markerPlots;
    end 
    methods
        function obj = KinematicChainStickFigure(kinematicChain, sceneBound)
            obj.kinematicChain = kinematicChain;
            kinematicChain.updateLinkVisualizationData();
            if nargin < 2
                obj.sceneBound = [-1; 1; -1; 1; -1; 1];
            else
                obj.sceneBound = sceneBound;
            end
            obj.sceneFigure = figure( 'Position', [ 1250, 1100, 400, 400 ], 'Name', 'scene' );
            obj.sceneAxes = axes( 'Position', [ 0.1 0.1 0.8 0.8 ]);
            axis equal; hold on;
            plot3([obj.sceneBound(1), obj.sceneBound(2)], [0, 0], [0, 0], 'color','k','Linewidth', 1, 'Linestyle',':');
            plot3([0, 0], [obj.sceneBound(3), obj.sceneBound(4)], [0, 0], 'color','k','Linewidth', 1, 'Linestyle',':');
            plot3([0, 0], [0, 0], [obj.sceneBound(5), obj.sceneBound(6)], 'color','k','Linewidth', 1, 'Linestyle',':');
            
            % set up joint plots
            obj.jointPlots = zeros(1, kinematicChain.numberOfJoints);
            for i_joint = 1 : kinematicChain.numberOfJoints
                obj.jointPlots(i_joint) = plot3(0, 0, 0, 'color', 'b', 'Linewidth', 3, 'Marker', 'o');
            end
            obj.endEffectorPlot = plot3(0, 0, 0, 'color', 'b', 'Linewidth', 3, 'Marker', 'o');
            
            % set up link plots
            obj.linkPlots = struct([]);
            for i_joint = 1 : kinematicChain.numberOfJoints
                for i_line = 1 : size(kinematicChain.linkVisualizationReferenceData(i_joint).startPoints, 2)
                    obj.linkPlots(i_joint).linePlots(i_line) = plot3(0, 0, 0, 'color', 'b', 'Linewidth', 2, 'Linestyle', '-');
                end
                if obj.kinematicChain.linkMasses(i_joint) > 0
                    obj.linkCenterPlots(i_joint) = plot3([0, 0], [0, 0], [0, 0], 'color', 'm', 'Linewidth', 2, 'Linestyle', 'x');
                end
            end
            
            % set up miscellaneous plots
            for i_line = 1 : size(kinematicChain.miscellaneousLinesStartPoints, 2)
                obj.miscellaneousPlots(i_line) = plot3([0 0], [0 0], [0 0], 'color', 'c', 'Linewidth', 1, 'Linestyle', '-');
            end
            
            % set up marker plots
            obj.markerPlots = cell(kinematicChain.numberOfJoints, 1);
            for i_joint = 1 : kinematicChain.numberOfJoints
                for i_marker = 1 : size(kinematicChain.markerPositions{i_joint}, 2)
                    obj.markerPlots{i_joint}(i_marker) = plot3(0, 0, 0, 'color', 'g', 'Linewidth', 1, 'Marker', '.');
                end
            end

            
            set(gca,'xlim',[obj.sceneBound(1), obj.sceneBound(2)], ...
                    'ylim',[obj.sceneBound(3), obj.sceneBound(4)], ...
                    'zlim',[obj.sceneBound(5), obj.sceneBound(6)] ...
               );
            xlabel('x');
            ylabel('y');
            zlabel('z');
        end
        
        function update(obj)
            % update data
            obj.kinematicChain.updateLinkVisualizationData();
            
            % update joint plots
            for i_joint = 1 : obj.kinematicChain.numberOfJoints
                set(obj.jointPlots(i_joint), ...
                        'Xdata', obj.kinematicChain.jointPositions{i_joint}(1), ...
                        'Ydata', obj.kinematicChain.jointPositions{i_joint}(2), ...
                        'Zdata', obj.kinematicChain.jointPositions{i_joint}(3))
            end
            % update end-effector plots
            set(obj.endEffectorPlot, ...
                    'Xdata', obj.kinematicChain.endEffectorPosition(1), ...
                    'Ydata', obj.kinematicChain.endEffectorPosition(2), ...
                    'Zdata', obj.kinematicChain.endEffectorPosition(3));
            % update links plots
            for i_joint = 1 : obj.kinematicChain.numberOfJoints
                % lines
                for i_line = 1 : size(obj.kinematicChain.linkVisualizationReferenceData(i_joint).startPoints, 2)
                    
                    start_point_current = obj.kinematicChain.linkVisualizationData(i_joint).startPoints(:, i_line);
                    end_point_current = obj.kinematicChain.linkVisualizationData(i_joint).endPoints(:, i_line);
                    
                    
                    set(obj.linkPlots(i_joint).linePlots(i_line), ...
                        'Xdata', [start_point_current(1), end_point_current(1)], ...
                        'Ydata', [start_point_current(2), end_point_current(2)], ...
                        'Zdata', [start_point_current(3), end_point_current(3)] ...
                        );
                end
                % center of mass
                if obj.kinematicChain.linkMasses(i_joint) > 0
                    set( ...
                         obj.linkCenterPlots(i_joint), ...
                         'Xdata', obj.kinematicChain.linkTransformations{i_joint}(1, 4), ...
                         'Ydata', obj.kinematicChain.linkTransformations{i_joint}(2, 4), ...
                         'Zdata', obj.kinematicChain.linkTransformations{i_joint}(3, 4) ...
                       )
                end
            end
            for i_line = 1 : length(obj.miscellaneousPlots)
                set( ...
                     obj.miscellaneousPlots(i_line), ...
                        'Xdata', [obj.kinematicChain.miscellaneousLinesStartPoints(1, i_line), obj.kinematicChain.miscellaneousLinesEndPoints(1, i_line)], ...
                        'Ydata', [obj.kinematicChain.miscellaneousLinesStartPoints(2, i_line), obj.kinematicChain.miscellaneousLinesEndPoints(2, i_line)], ...
                        'Zdata', [obj.kinematicChain.miscellaneousLinesStartPoints(3, i_line), obj.kinematicChain.miscellaneousLinesEndPoints(3, i_line)] ...
                   )
                
            end

            % update marker plots
            for i_joint = 1 : obj.kinematicChain.numberOfJoints
                for i_marker = 1 : size(obj.kinematicChain.markerPositions{i_joint}, 2)
                    set( ...
                         obj.markerPlots{i_joint}(i_marker), ...
                            'Xdata', obj.kinematicChain.markerPositions{i_joint}(1, i_marker), ...
                            'Ydata', obj.kinematicChain.markerPositions{i_joint}(2, i_marker), ...
                            'Zdata', obj.kinematicChain.markerPositions{i_joint}(3, i_marker) ...
                       )
                end
            end

            
        end
        function setMiscellaneousPlotColor(obj, index, color)
            if index > length(obj.miscellaneousPlots)
                error('index larger than number of miscellaneous plots')
            else
                set(obj.miscellaneousPlots(index), 'color', color);
                obj.update();
            end
        end
        function setMarkerColor(obj, index, color)
            if index > length(obj.markerPlots)
                error('index larger than number of marker plots')
            else
                set(obj.markerPlots(index), 'color', color);
                obj.update();
            end
        end
    end
end