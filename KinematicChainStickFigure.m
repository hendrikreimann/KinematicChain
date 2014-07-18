% class for the display of a stick figure

classdef KinematicChainStickFigure
    properties
        kinematicChain;
        sceneFigure;
        sceneAxes;
        sceneBound;
        jointPlots;
        linkPlots;
        linkCenterPlots;
        endEffectorPlot;
    end 
    methods
        function obj = KinematicChainStickFigure(kinematicChain, sceneBound)
            obj.kinematicChain = kinematicChain;
            kinematicChain.updateLinkVisualizationData();
            if nargin < 2
                obj.sceneBound = [-2; 2; -2; 2; -2; 2];
%                 obj.sceneBound = [-5; 5; -5; 5; -5; 5];
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
                obj.linkCenterPlots(i_joint) = plot([0, 0], [0, 0], 'color', 'm', 'Linewidth', 2, 'Linestyle', 'x');
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
            obj.kinematicChain.updateLinkVisualizationData();
            % plot joints
            for i_joint = 1 : obj.kinematicChain.numberOfJoints
                set(obj.jointPlots(i_joint), ...
                        'Xdata', obj.kinematicChain.jointPositions{i_joint}(1), ...
                        'Ydata', obj.kinematicChain.jointPositions{i_joint}(2), ...
                        'Zdata', obj.kinematicChain.jointPositions{i_joint}(3))
            end
            % plot end-effector
            set(obj.endEffectorPlot, ...
                    'Xdata', obj.kinematicChain.endEffectorPosition(1), ...
                    'Ydata', obj.kinematicChain.endEffectorPosition(2), ...
                    'Zdata', obj.kinematicChain.endEffectorPosition(3));
            % plot links
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
                set( ...
                     obj.linkCenterPlots(i_joint), ...
                     'Xdata', obj.kinematicChain.linkTransformations{i_joint}(1, 4), ...
                     'Ydata', obj.kinematicChain.linkTransformations{i_joint}(2, 4), ...
                     'Zdata', obj.kinematicChain.linkTransformations{i_joint}(3, 4) ...
                   )
                
            end
            
            
            
        end        
    end
end