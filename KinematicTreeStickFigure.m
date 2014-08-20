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
            plot3([obj.sceneBound(1), obj.sceneBound(2)], [0, 0], [0, 0], 'color','k','Linewidth', 1, 'Linestyle',':');
            plot3([0, 0], [obj.sceneBound(3), obj.sceneBound(4)], [0, 0], 'color','k','Linewidth', 1, 'Linestyle',':');
            plot3([0, 0], [0, 0], [obj.sceneBound(5), obj.sceneBound(6)], 'color','k','Linewidth', 1, 'Linestyle',':');

            % set up joint plots
            obj.jointPlots = zeros(1, kinematicTree.numberOfJoints);
            for i_joint = 1 : kinematicTree.numberOfJoints
                obj.jointPlots(i_joint) = plot(0, 0, 'color', 'b', 'Linewidth', 3, 'Marker', 'o');
            end
            
            % set up link plots
            obj.linkPlots = zeros(1, kinematicTree.numberOfJoints);
            for i_joint = 1 : kinematicTree.numberOfJoints
                obj.linkPlots(i_joint) = plot([0, 0], [0, 0], 'color', 'b', 'Linewidth', 2, 'Linestyle', '-');
                obj.linkCenterPlots(i_joint) = plot([0, 0], [0, 0], 'color', 'm', 'Linewidth', 2, 'Linestyle', 'x');
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
            for i_joint = 1 : kinematicTree.numberOfJoints
                for i_marker = 1 : size(kinematicTree.markerPositions{i_joint}, 2)
                    obj.markerPlots{i_joint}(i_marker) = plot3(0, 0, 0, 'color', 'g', 'Linewidth', 1, 'Marker', '.');
                end
            end
            
            
            % groom
            set(gca,'xlim',[obj.sceneBound(1), obj.sceneBound(2)],'ylim',[obj.sceneBound(3), obj.sceneBound(4)]);
            xlabel('x');
            ylabel('y');
            zlabel('z');
        end
        
        function update(obj)
            for i_joint = 1 : obj.kinematicTree.numberOfJoints
                set( ...
                     obj.jointPlots(i_joint), ...
                     'Xdata', obj.kinematicTree.jointPositions{i_joint}(1), ...
                     'Ydata', obj.kinematicTree.jointPositions{i_joint}(2), ...
                     'Zdata', obj.kinematicTree.jointPositions{i_joint}(3) ...
                   )
                if obj.kinematicTree.linkMasses(i_joint) > 0
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
                set( ...
                     obj.linkPlots(i_joint), ...
                     'Xdata', [obj.kinematicTree.jointPositions{obj.kinematicTree.jointParents(i_joint)}(1), obj.kinematicTree.jointPositions{i_joint}(1)], ...
                     'Ydata', [obj.kinematicTree.jointPositions{obj.kinematicTree.jointParents(i_joint)}(2), obj.kinematicTree.jointPositions{i_joint}(2)], ...
                     'Zdata', [obj.kinematicTree.jointPositions{obj.kinematicTree.jointParents(i_joint)}(3), obj.kinematicTree.jointPositions{i_joint}(3)] ...
                   )
            end
            
            for i_eef = 1 : obj.kinematicTree.numberOfBranches
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
            end
            
            % update marker plots
            for i_joint = 1 : obj.kinematicTree.numberOfJoints
                for i_marker = 1 : size(obj.kinematicTree.markerPositions{i_joint}, 2)
                    set( ...
                         obj.markerPlots{i_joint}(i_marker), ...
                            'Xdata', obj.kinematicTree.markerPositions{i_joint}(1, i_marker), ...
                            'Ydata', obj.kinematicTree.markerPositions{i_joint}(2, i_marker), ...
                            'Zdata', obj.kinematicTree.markerPositions{i_joint}(3, i_marker) ...
                       )
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
    end
end