% class for the display of a stick figure

classdef SerialChainStickFigure
    properties
        serialChain;
        sceneFigure;
        sceneAxes;
        sceneBound;
        jointPlots;
        linkPlots;
        endEffectorPlot;
    end 
    methods
        function obj = SerialChainStickFigure(serialChain, sceneBound)
            obj.serialChain = serialChain;
            if nargin < 2
                obj.sceneBound = [-2; 2; -2; 2; -2; 2];
                obj.sceneBound = [-5; 5; -5; 5; -5; 5];
            else
                obj.sceneBound = sceneBound;
            end
            obj.sceneFigure = figure( 'Position', [ 1250, 1100, 400, 400 ], 'Name', 'scene' );
            obj.sceneAxes = axes( 'Position', [ 0.1 0.1 0.8 0.8 ]);
            axis equal; hold on;
            plot3([obj.sceneBound(1), obj.sceneBound(2)], [0, 0], [0, 0], 'color','k','Linewidth', 1, 'Linestyle',':');
            plot3([0, 0], [obj.sceneBound(3), obj.sceneBound(4)], [0, 0], 'color','k','Linewidth', 1, 'Linestyle',':');
            plot3([0, 0], [0, 0], [obj.sceneBound(5), obj.sceneBound(6)], 'color','k','Linewidth', 1, 'Linestyle',':');
            
            obj.jointPlots = zeros(1, serialChain.numberOfJoints);
            obj.linkPlots = zeros(1, serialChain.numberOfJoints);
            for l = 1 : serialChain.numberOfJoints
                obj.jointPlots(l) = plot3(0, 0, 0, 'color', 'b', 'Linewidth', 3, 'Marker', 'o');
                obj.linkPlots(l) = plot3(0, 0, 0, 'color', 'b', 'Linewidth', 2, 'Linestyle', '-');
            end
            obj.endEffectorPlot = plot3(0, 0, 0, 'color', 'b', 'Linewidth', 3, 'Marker', 'o');
            obj.jointPlots(serialChain.numberOfJoints+1) = plot3(0, 0, 0, 'color','b','Linewidth',3,'Linestyle','o');
            set(gca,'xlim',[obj.sceneBound(1), obj.sceneBound(2)], ...
                    'ylim',[obj.sceneBound(3), obj.sceneBound(4)], ...
                    'zlim',[obj.sceneBound(5), obj.sceneBound(6)] ...
               );
            xlabel('x');
            ylabel('y');
            zlabel('z');
        end
        
        function update(obj)
            for i_joint = 1 : obj.serialChain.numberOfJoints
                set(obj.jointPlots(i_joint), ...
                        'Xdata', obj.serialChain.jointPositions{i_joint}(1), ...
                        'Ydata', obj.serialChain.jointPositions{i_joint}(2), ...
                        'Zdata', obj.serialChain.jointPositions{i_joint}(3))
            end
            for i_joint = 1 : obj.serialChain.numberOfJoints - 1
                set(obj.linkPlots(i_joint), ...
                        'Xdata', [obj.serialChain.jointPositions{i_joint}(1), obj.serialChain.jointPositions{i_joint+1}(1)], ...
                        'Ydata', [obj.serialChain.jointPositions{i_joint}(2), obj.serialChain.jointPositions{i_joint+1}(2)], ...
                        'Zdata', [obj.serialChain.jointPositions{i_joint}(3), obj.serialChain.jointPositions{i_joint+1}(3)]);
            end            
            set(obj.endEffectorPlot, ...
                    'Xdata', obj.serialChain.endEffectorPosition(1), ...
                    'Ydata', obj.serialChain.endEffectorPosition(2), ...
                    'Zdata', obj.serialChain.endEffectorPosition(3));
            set(obj.linkPlots(obj.serialChain.numberOfJoints), ...
                    'Xdata', [obj.serialChain.jointPositions{obj.serialChain.numberOfJoints}(1), obj.serialChain.endEffectorPosition(1)], ...
                    'Ydata', [obj.serialChain.jointPositions{obj.serialChain.numberOfJoints}(2), obj.serialChain.endEffectorPosition(2)], ...
                    'Zdata', [obj.serialChain.jointPositions{obj.serialChain.numberOfJoints}(3), obj.serialChain.endEffectorPosition(3)]);
                
            
%             set(obj.jointPlots(end), ...
%                     'Xdata', plant.mJointTransformations{ obj.mNumberOfJoints + 1 }(2, 4), ...
%                     'Ydata', plant.mJointTransformations{ obj.mNumberOfJoints + 1 }(3, 4));
            
        end        
    end
end