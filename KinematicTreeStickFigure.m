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
        linkMassShapeData;
        linkMassShapeSurfs;
        linkMassShapeResolution = 15;
        linkFrameToLinkInertiaRotations;
        jointLabels;
        recordedMarkerPlots;
        
        % graphics
        jointPlotsColor = [0 0 1];
        linkPlotsColor = [0 0 0];
        jointPlotsLinewidth = 3;
        linkPlotsLinewidth = 2;
        
        % flags
        showLinkCentersOfMass = false;
        showLinkMassEllipsoids = false;
        showStickFigure = true;
        showMarkers = true;
        showJointLabels = false;
    end 
    methods
        function this = KinematicTreeStickFigure(kinematicTree, sceneBound, visualizationVolume, recordedMarkers, axesHandle)
            this.kinematicTree = kinematicTree;
            kinematicTree.updateLinkVisualizationData();
            if nargin < 2
                this.sceneBound = [-1; 1; -1; 1; -1; 1];
            else
                this.sceneBound = sceneBound;
            end
            if nargin < 3
                visualizationVolume = 'none';
            end
            if nargin < 5
                this.sceneFigure = figure( 'Position', [500, 500, 600, 600], 'Name', 'scene' );
                this.sceneAxes = axes( 'Position', [ 0.1 0.1 0.8 0.8 ]);
            else
                this.sceneFigure = get(axesHandle, 'parent');
                this.sceneAxes = axesHandle;
            end
            if nargin < 4
                recordedMarkers = [];
            end
            
            axis equal; hold on;
            plot3([this.sceneBound(1, 1), this.sceneBound(1, 2)], [0, 0]+mean(sceneBound(2, :)), [0, 0]+mean(sceneBound(3, :)), 'color', 'k','Linewidth', 1, 'Linestyle',':');
            plot3([0, 0]+mean(sceneBound(1, :)), [this.sceneBound(2, 1), this.sceneBound(2, 2)], [0, 0]+mean(sceneBound(3, :)), 'color', 'k','Linewidth', 1, 'Linestyle',':');
            plot3([0, 0]+mean(sceneBound(1, :)), [0, 0]+mean(sceneBound(2, :)), [this.sceneBound(3, 1), this.sceneBound(3, 2)], 'color', 'k','Linewidth', 1, 'Linestyle',':');

            % set up joint plots
            this.jointPlots = zeros(1, kinematicTree.numberOfJoints);
            for i_joint = 1 : kinematicTree.numberOfJoints
                this.jointPlots(i_joint) = plot(0, 0, 'color', this.jointPlotsColor, 'Linewidth', this.jointPlotsLinewidth, 'Marker', 'o');
            end
            
            % set up link plots
            this.linkPlots = zeros(1, kinematicTree.numberOfJoints);
            for i_joint = 1 : kinematicTree.numberOfJoints
                this.linkPlots(i_joint) = plot([0, 0], [0, 0], 'color', this.linkPlotsColor, 'Linewidth', this.linkPlotsLinewidth, 'Linestyle', '-');
                this.linkCenterPlots(i_joint) = plot([0, 0], [0, 0], 'color', 'm', 'Linewidth', 2, 'Marker', 'x');
            end
            % TODO: port the support for arbitrary lines from the KinematicChain
            
            % set up eef plots
            for i_eef = 1 : kinematicTree.numberOfBranches
                this.endEffectorPlots(i_eef) = plot(0, 0, 'color', this.jointPlotsColor, 'Linewidth', this.jointPlotsLinewidth, 'Marker', 'o');
                this.endEffectorLinkPlots(i_eef) = plot([0, 0], [0, 0], 'color', this.linkPlotsColor, 'Linewidth', this.linkPlotsLinewidth, 'Linestyle', '-');
                this.endEffectorVelocityPlots(i_eef) = plot([0, 0], [0, 0], 'color', 'c', 'Linewidth', 1, 'Linestyle', '-');
            end
            
            % set up miscellaneous plots
            for i_line = 1 : size(kinematicTree.miscellaneousLinesStartPoints, 2)
                this.miscellaneousPlots(i_line) = plot3([0 0], [0 0], [0 0], 'color', 'c', 'Linewidth', 1, 'Linestyle', '-');
            end
            
            % set up marker plots
            this.markerPlots = cell(kinematicTree.numberOfJoints, 1);
%             marker_index = 0;
            for i_marker = 1 : kinematicTree.getNumberOfMarkers(0)
%                 marker_index = marker_index + 1;
                this.fixedMarkerPlots(i_marker) = plot3(0, 0, 0, 'color', this.kinematicTree.getMarkerVisualizationColor(0, i_marker), 'Linewidth', 2, 'Marker', 'h');
            end
            for i_joint = 1 : kinematicTree.numberOfJoints
                for i_marker = 1 : size(kinematicTree.markerPositions{i_joint}, 2)
%                     marker_index = marker_index + 1;
                    this.markerPlots{i_joint}(i_marker) = plot3(0, 0, 0, 'color', this.kinematicTree.getMarkerVisualizationColor(i_joint, i_marker), 'markersize', 10, 'linewidth', 3, 'Marker', 'x');
                end
            end
            for i_line = 1 : size(kinematicTree.markerConnectionLineIndices, 1)
                this.markerConnectionLinePlots(i_line) = plot3([0 0], [0 0], [0 0], 'k', 'linewidth', 1);
            end
            
            % set up link mass shapes
            this.linkFrameToLinkInertiaRotations = cell(kinematicTree.numberOfJoints, 1);
            for i_joint = 1 : kinematicTree.numberOfJoints
                m = kinematicTree.linkMasses(i_joint);
                
                % diagonalize
                inertia_tensor_link_frame = kinematicTree.generalizedInertiaMatrices{i_joint}(4:6, 4:6);
                [U, S, V] = svd(inertia_tensor_link_frame);
                this.linkFrameToLinkInertiaRotations{i_joint} = U;
                I_a = S(1, 1);
                I_b = S(2, 2);
                I_c = S(3, 3);
                
%                 if i_joint == 1
%                     disp('inertia_tensor_link_frame : ')
%                     disp(num2str(inertia_tensor_link_frame));
%                     disp('U_1 : ')
%                     disp(num2str(U));
%                 end
%                 if i_joint == 10
%                     disp('inertia_tensor_link_frame : ')
%                     disp(num2str(inertia_tensor_link_frame));
%                     disp('U_10 : ')
%                     disp(num2str(U));
%                 end
%                 if i_joint == 12
%                     disp('inertia_tensor_link_frame : ')
%                     disp(num2str(inertia_tensor_link_frame));
%                     disp('U_12 : ')
%                     disp(num2str(U));
%                 end
%                 if i_joint == 21
%                     disp('inertia_tensor_link_frame : ')
%                     disp(num2str(inertia_tensor_link_frame));
%                     disp('U_21 : ')
%                     disp(num2str(U));
%                 end
                
                if strcmp(visualizationVolume, 'ellipsoid');
                
                    a = sqrt((5/(2*m)*(- I_a + I_b + I_c)));
                    b = sqrt((5/(2*m)*(+ I_a - I_b + I_c)));
                    c = sqrt((5/(2*m)*(+ I_a + I_b - I_c)));

                    [x,y,z] = ...
                      ellipsoid ...
                      ( ...
                        0, 0, 0, ...
                        a, b, c, ...
                        this.linkMassShapeResolution ...
                      );
                    shape_data = ones(4, this.linkMassShapeResolution+1, this.linkMassShapeResolution+1);
                    shape_data(1, :, :) = x;
                    shape_data(2, :, :) = y;
                    shape_data(3, :, :) = z;

                    this.linkMassShapeData{i_joint} = shape_data;
                    this.linkMassShapeSurfs(i_joint) = surf ...
                      ( ...
                        x, y, z, ...
                        'FaceColor','interp',...
                        'EdgeColor','none',...
                        'FaceLighting','gouraud' ...
                      );
                    
                    
% surf(X,Y,Z,'FaceColor','interp',...
%    'EdgeColor','none',...
%    'FaceLighting','gouraud')            
                    
                elseif strcmp(visualizationVolume, 'cuboid');
                    a = sqrt((6/m * (- I_a + I_b + I_c)));
                    b = sqrt((6/m * (+ I_a - I_b + I_c)));
                    c = sqrt((6/m * (+ I_a + I_b - I_c)));
                    faces = ...
                      [ ...
                         1     2     3     4; ...
                         5     6     7     8; ...
                         4     3     6     5; ...
                         3     2     7     6; ...
                         2     1     8     7; ...
                         1     4     5     8; ...
                      ];
                    x = 0.5 * a * [-1 1 1 -1 -1 1 1 -1]';
                    y = 0.5 * b * [1 1 1 1 -1 -1 -1 -1]';
                    z = 0.5 * c * [-1 -1 1 1 1 1 -1 -1]';
                    vertices = [x y z];
                    
                    volume = a*b*c;
                    if m > 0
                        density = m / volume;
                        disp(['segment ' num2str(i_joint) ' density: ' num2str(density) ' kg/m^3']);
                    end

%                     patch('Faces', faces, 'Vertices', vertices);
%                     patch('Faces', faces, 'Vertices', vertices', 'FaceColor', colr, 'FaceAlpha', alph);
                    this.linkMassShapeSurfs(i_joint) = patch('Faces', faces, 'Vertices', vertices, 'FaceColor', [1 0.6 0]);
                    
                    shape_data = ones(4, 4, 6);
                    shape_data(1, :, :) = get(this.linkMassShapeSurfs(i_joint), 'xdata');
                    shape_data(2, :, :) = get(this.linkMassShapeSurfs(i_joint), 'ydata');
                    shape_data(3, :, :) = get(this.linkMassShapeSurfs(i_joint), 'zdata');

                    this.linkMassShapeData{i_joint} = shape_data;
                    
                    
%                     x = [-a -a; -a -a; a a; a -a;];
%                     y = [-b -b; b b; -b -b; b b];
%                     z = [-c c; -c c; -c c; -c c];
%                     surf(x, y, z);
%                     
%                     [x, y, z] = meshgrid([-a a], [-b b], [-c c]);
%                     
%                     
%                     [z, y, x] = cylinder; % around the x-axis
%                     x = l * (x - 1/2);
%                     y = y * b/2;
%                     z = z * c/2;
% 
%                     shape_data = ones(4, 2, 20+1);
%                     shape_data(1, :, :) = x;
%                     shape_data(2, :, :) = y;
%                     shape_data(3, :, :) = z;
% 
%                     this.linkMassShapeData{i_joint} = shape_data;
                else
                    % don't show anything
                    this.linkMassShapeData{i_joint} = [];
                    this.linkMassShapeSurfs(i_joint) = patch('Faces', [], 'Vertices', []);
                end
            end
            
            % set up joint labels
            for i_joint = 1 : kinematicTree.numberOfJoints
                this.jointLabels(i_joint) = text(0, 0, ['  ' kinematicTree.jointLabels{i_joint}]); 
            end
            
            
            % groom
            set(gca,'xlim',[this.sceneBound(1, 1), this.sceneBound(1, 2)],'ylim',[this.sceneBound(2, 1), this.sceneBound(2, 2)], 'zlim',[this.sceneBound(3, 1), this.sceneBound(3, 2)]);
            xlabel('x');
            ylabel('y');
            zlabel('z');
            
%             daspect([5 5 1])
%             axis tight
            view(-50,30)
%             camlight left
%             camlight('headlight')
            camlight(30, 30)
%             colormap cool
%             colormap autumn
            alpha(.4)
    
            this.createRecordedMarkerPlots(recordedMarkers);
        end
        
        function update(this)
            for i_joint = 1 : this.kinematicTree.numberOfJoints
                if this.showStickFigure
                    set( ...
                         this.jointPlots(i_joint), ...
                         'Xdata', this.kinematicTree.jointPositions{i_joint}(1), ...
                         'Ydata', this.kinematicTree.jointPositions{i_joint}(2), ...
                         'Zdata', this.kinematicTree.jointPositions{i_joint}(3) ...
                       )
                else
                    set(this.jointPlots(i_joint), 'visible', 'off');
                end
                if this.kinematicTree.linkMasses(i_joint) > 0 && this.showLinkCentersOfMass
                    set( ...
                         this.linkCenterPlots(i_joint), ...
                         'Xdata', this.kinematicTree.linkTransformations{i_joint}(1, 4), ...
                         'Ydata', this.kinematicTree.linkTransformations{i_joint}(2, 4), ...
                         'Zdata', this.kinematicTree.linkTransformations{i_joint}(3, 4) ...
                       )
                else
                    set(this.linkCenterPlots(i_joint), 'visible', 'off');
                end
                if this.showJointLabels
                    set(this.jointLabels(i_joint), 'position', this.kinematicTree.jointPositions{i_joint}, 'visible', 'on');
                else
                    set(this.jointLabels(i_joint), 'visible', 'off');

                end
                
            end
            for i_joint = 2 : this.kinematicTree.numberOfJoints
                if this.showStickFigure
                    set( ...
                         this.linkPlots(i_joint), ...
                         'Xdata', [this.kinematicTree.jointPositions{this.kinematicTree.jointParents(i_joint)}(1), this.kinematicTree.jointPositions{i_joint}(1)], ...
                         'Ydata', [this.kinematicTree.jointPositions{this.kinematicTree.jointParents(i_joint)}(2), this.kinematicTree.jointPositions{i_joint}(2)], ...
                         'Zdata', [this.kinematicTree.jointPositions{this.kinematicTree.jointParents(i_joint)}(3), this.kinematicTree.jointPositions{i_joint}(3)] ...
                       )
                else
                    set(this.linkPlots(i_joint), 'visible', 'off');
                end
            end
            
            for i_eef = 1 : this.kinematicTree.numberOfBranches
                if this.showStickFigure
                    set(this.endEffectorPlots(i_eef), ...
                            'Xdata', this.kinematicTree.endEffectorPositions{i_eef}(1), ...
                            'Ydata', this.kinematicTree.endEffectorPositions{i_eef}(2), ...
                            'Zdata', this.kinematicTree.endEffectorPositions{i_eef}(3))
                    set( ...
                         this.endEffectorLinkPlots(i_eef), ...
                         'Xdata', [this.kinematicTree.jointPositions{this.kinematicTree.endEffectorParents(i_eef)}(1), this.kinematicTree.endEffectorPositions{i_eef}(1)], ...
                         'Ydata', [this.kinematicTree.jointPositions{this.kinematicTree.endEffectorParents(i_eef)}(2), this.kinematicTree.endEffectorPositions{i_eef}(2)], ...
                         'Zdata', [this.kinematicTree.jointPositions{this.kinematicTree.endEffectorParents(i_eef)}(3), this.kinematicTree.endEffectorPositions{i_eef}(3)] ...
                       )
%                     set( ...
%                          this.endEffectorVelocityPlots(i_eef), ...
%                          'Xdata', [this.kinematicTree.endEffectorPositions{i_eef}(1), this.kinematicTree.endEffectorPositions{i_eef}(1) + this.kinematicTree.endEffectorVelocities{i_eef}(1)], ...
%                          'Ydata', [this.kinematicTree.endEffectorPositions{i_eef}(2), this.kinematicTree.endEffectorPositions{i_eef}(2) + this.kinematicTree.endEffectorVelocities{i_eef}(2)], ...
%                          'Zdata', [this.kinematicTree.endEffectorPositions{i_eef}(3), this.kinematicTree.endEffectorPositions{i_eef}(3) + this.kinematicTree.endEffectorVelocities{i_eef}(3)] ...
%                        )
                else
                    set(this.endEffectorPlots(i_eef), 'visible', 'off');
                    set(this.endEffectorLinkPlots(i_eef), 'visible', 'off');
                    set(this.endEffectorVelocityPlots(i_eef), 'visible', 'off');
                end
            end
            
            % update segment mass shapes
            for i_joint = 1 : this.kinematicTree.numberOfJoints
                if this.showLinkMassEllipsoids && (this.kinematicTree.linkMasses(i_joint) > 0) && ~isempty(this.linkMassShapeData{i_joint})
                    % transform ellipsoid
                    link_transformation = this.kinematicTree.linkTransformations{i_joint};
                    inertia_transformation = [this.linkFrameToLinkInertiaRotations{i_joint} zeros(3, 1); 0 0 0 1];
                    shape_transformed = zeros(size(this.linkMassShapeData{i_joint}));
                    mass_shape_data = this.linkMassShapeData{i_joint};
                    for i_point = 1 : size(mass_shape_data, 2)
                        for j_point = 1 : size(mass_shape_data, 3)
                            point = squeeze(mass_shape_data(:, i_point, j_point));
                            shape_transformed(:, i_point, j_point) = link_transformation * inertia_transformation * point;
                        end
                    end
                    
                    set( ...
                         this.linkMassShapeSurfs(i_joint), ...
                         'Xdata', squeeze(shape_transformed(1, :, :)), ...
                         'Ydata', squeeze(shape_transformed(2, :, :)), ...
                         'Zdata', squeeze(shape_transformed(3, :, :)) ...
                       );
                else
                    set(this.linkMassShapeSurfs(i_joint), 'visible', 'off');
                end
            end
                
            % update marker plots
            for i_marker = 1 : this.kinematicTree.getNumberOfMarkers(0)
                if this.showMarkers
                    position = this.kinematicTree.getMarkerPosition(0, i_marker);
                    set( ...
                         this.fixedMarkerPlots(i_marker), ...
                            'Xdata', position(1), ...
                            'Ydata', position(2), ...
                            'Zdata', position(3) ...
                       )
                else
                    set(this.fixedMarkerPlots(i_marker), 'visible', 'off');
                end
            end
            for i_joint = 1 : this.kinematicTree.numberOfJoints
                for i_marker = 1 : this.kinematicTree.getNumberOfMarkers(i_joint)
                    if this.showMarkers
                        position = this.kinematicTree.getMarkerPosition(i_joint, i_marker);
                        set( ...
                             this.markerPlots{i_joint}(i_marker), ...
                                'Xdata', position(1), ...
                                'Ydata', position(2), ...
                                'Zdata', position(3) ...
                           )
                    else
                        set(this.markerPlots{i_joint}(i_marker), 'visible', 'off');
                    end
                end
            end
            for i_line = 1 : length(this.markerConnectionLinePlots)
                if this.showMarkers
                    joint_index_marker_one = this.kinematicTree.markerExportMap(1, this.kinematicTree.markerConnectionLineIndices(i_line, 1));
                    marker_index_marker_one = this.kinematicTree.markerExportMap(2, this.kinematicTree.markerConnectionLineIndices(i_line, 1));
                    joint_index_marker_two = this.kinematicTree.markerExportMap(1, this.kinematicTree.markerConnectionLineIndices(i_line, 2));
                    marker_index_marker_two = this.kinematicTree.markerExportMap(2, this.kinematicTree.markerConnectionLineIndices(i_line, 2));
                    position_marker_one = this.kinematicTree.getMarkerPosition(joint_index_marker_one, marker_index_marker_one);
                    position_marker_two = this.kinematicTree.getMarkerPosition(joint_index_marker_two, marker_index_marker_two);
                    set( ...
                         this.markerConnectionLinePlots(i_line), ...
                            'Xdata', [position_marker_one(1) position_marker_two(1)], ...
                            'Ydata', [position_marker_one(2) position_marker_two(2)], ...
                            'Zdata', [position_marker_one(3) position_marker_two(3)] ...
                       )
                else
                    set(this.markerConnectionLinePlots(i_line), 'visible', 'off');
                end
            end
            
            
        end
        function setMiscellaneousPlotColor(this, index, color)
            if index > length(this.miscellaneousPlots)
                error('index larger than number of miscellaneous plots')
            else
                this.miscellaneousPlots(index) = plot3(this.sceneAxes, [0 1], [0 1], [0 0], 'color', color, 'Linewidth', 1, 'Linestyle', '-');
                this.update();
            end
        end
        function setMarkerColor(this, joint_index, marker_index, color)
            set(this.markerPlots{joint_index}(marker_index), 'color', color);
            this.update();
        end
        function color = getMarkerColor(this, joint_index, marker_index)
            if joint_index == 0
                color = get(this.fixedMarkerPlots(marker_index), 'color');
            else
                color = get(this.markerPlots{joint_index}(marker_index), 'color');
            end
        end
        
        function createRecordedMarkerPlots(this, recorded_marker_positions)
            for i_marker = 1 : length(recorded_marker_positions) / 3;
                this.recordedMarkerPlots(i_marker) = plot3(0, 0, 0, 'o', 'color', [1 1 1]*0.7);
                
                set ...
                  ( ...
                    this.recordedMarkerPlots(i_marker), ...
                    'XData', recorded_marker_positions((i_marker-1)*3 + 1), ...
                    'YData', recorded_marker_positions((i_marker-1)*3 + 2), ...
                    'ZData', recorded_marker_positions((i_marker-1)*3 + 3) ...
                  );
            end
            this.updateRecordedMarkerPlots(recorded_marker_positions);
        end
        function updateRecordedMarkerPlots(this, recorded_marker_positions)
            for i_marker = 1 : length(recorded_marker_positions) / 3;
                set ...
                  ( ...
                    this.recordedMarkerPlots(i_marker), ...
                    'XData', recorded_marker_positions((i_marker-1)*3 + 1), ...
                    'YData', recorded_marker_positions((i_marker-1)*3 + 2), ...
                    'ZData', recorded_marker_positions((i_marker-1)*3 + 3) ...
                  );
            end
        end
        
        function setLinkPlotsColor(this, color)
            for i_joint = 1 : this.kinematicTree.numberOfJoints
                this.linkPlotsColor = color;
                set(this.linkPlots(i_joint), 'color', color);
            for i_eef = 1 : this.kinematicTree.numberOfBranches
                set(this.endEffectorLinkPlots(i_eef), 'color', color);
            end
        end
        end
        function setLinkPlotsLinewidth(this, linewidth)
            for i_joint = 1 : this.kinematicTree.numberOfJoints
                this.linkPlotsLinewidth = linewidth;
                set(this.linkPlots(i_joint), 'linewidth', linewidth);
            end
            for i_eef = 1 : this.kinematicTree.numberOfBranches
                set(this.endEffectorLinkPlots(i_eef), 'linewidth', linewidth);
            end
        
        end
    end
end