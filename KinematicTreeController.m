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

function stickFigure = KinematicTreeController(kinematicTree, sceneBound, visualizationVolume, recordedMarkers)
    if nargin < 3
        visualizationVolume = 'none';
    end
    if nargin < 4
        recordedMarkers = [];
    end

    stickFigure = KinematicTreeStickFigure(kinematicTree, sceneBound, visualizationVolume, recordedMarkers);
    stickFigure.showLinkMassEllipsoids = true;
    stickFigure.update();

    small_step = 0.01;
    large_step = 0.5;
    
    % make controller figure
    line_height = 20;
    button_width = 20;
    box_width = 50;
    label_width = 200;
    
    figure_width = button_width*4 + box_width + label_width;
    figure_height = line_height * kinematicTree.numberOfJoints;
    
    controller_figure = figure('Position', [100, 100, figure_width, figure_height]);
    
    for i_joint = 1 : kinematicTree.numberOfJoints
        joint_angle_boxes(i_joint) = uicontrol(controller_figure, 'Style', 'edit', 'string', num2str(kinematicTree.jointAngles(i_joint)), 'Position', [button_width*2, figure_height-line_height*i_joint, box_width, line_height]);
        joint_angle_minusminus_buttons(i_joint) = uicontrol(controller_figure, 'Style', 'PushButton', 'Callback', @changeJointAngle, 'string', '<<', 'Position', [0, figure_height-line_height*i_joint, button_width, line_height]);
        joint_angle_minus_buttons(i_joint) = uicontrol(controller_figure, 'Style', 'PushButton', 'Callback', @changeJointAngle, 'string', '<', 'Position', [button_width, figure_height-line_height*i_joint, button_width, line_height]);
        joint_angle_plus_buttons(i_joint) = uicontrol(controller_figure, 'Style', 'PushButton', 'Callback', @changeJointAngle, 'string', '>', 'Position', [button_width*2+box_width, figure_height-line_height*i_joint, button_width, line_height]);
        joint_angle_plusplus_buttons(i_joint) = uicontrol(controller_figure, 'Style', 'PushButton', 'Callback', @changeJointAngle, 'string', '>>', 'Position', [button_width*3+box_width, figure_height-line_height*i_joint, button_width, line_height]);
        joint_labels(i_joint) = uicontrol(controller_figure, 'Style', 'text', 'string', kinematicTree.jointLabels{i_joint}, 'HorizontalAlignment', 'left', 'Position', [button_width*4+box_width, figure_height-line_height*i_joint, label_width, line_height]);
    end
    

    function changeJointAngle(source, eventdata)
        if ismember(source, joint_angle_minusminus_buttons);
            joint = find(~(double(joint_angle_minusminus_buttons)-double(source)));
            joint_angle_boxes(joint).String = num2str(str2num(joint_angle_boxes(joint).String) - large_step);
        elseif ismember(source, joint_angle_minus_buttons);
            joint = find(~(double(joint_angle_minus_buttons)-double(source)));
            joint_angle_boxes(joint).String = num2str(str2num(joint_angle_boxes(joint).String) - small_step);
        elseif ismember(source, joint_angle_plus_buttons);
            joint = find(~(double(joint_angle_plus_buttons)-double(source)));
            joint_angle_boxes(joint).String = num2str(str2num(joint_angle_boxes(joint).String) + small_step);
        elseif ismember(source, joint_angle_plusplus_buttons);
            joint = find(~(double(joint_angle_plusplus_buttons)-double(source)));
            joint_angle_boxes(joint).String = num2str(str2num(joint_angle_boxes(joint).String) + large_step);
        end
        kinematicTree.jointAngles(joint) = str2num(joint_angle_boxes(joint).String);
        kinematicTree.updateKinematics;
        stickFigure.update();
    end
end



