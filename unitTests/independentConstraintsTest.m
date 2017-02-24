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
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.

% script to arm whether the forces from two linearly independent constraints can be calculated independently

show_visualization = 1;

ankle = [0; 0; 1]; knee = [0; 0; 2]; hip = [0; 0; 3];
contact = [0; 0; 0];
head = [0; 0; 4];
feet_position = [0; 0; 0.5]; shank_position = [0; 0; 1.5]; thigh_position = [0; 0; 2.5]; torso_position = [0; 0; 3.5]; 

joint_positions_3dof = {ankle, knee, hip};
joint_axes_3dof = {[1; 0; 0], [1; 0; 0], [1; 0; 0]};
joint_types_3dof = [1 1 1];
branch_matrix_3dof = [1 1 1];
link_positions_3dof = {shank_position, thigh_position, torso_position};
link_orientations_3dof = {eye(3), eye(3), eye(3)};
link_masses_3dof = [1 1 2];
link_moments_of_inertia_3dof = [1 1 1; 5 5 5; 1 1 1] * 0.1;

arm = GeneralKinematicTree ...
( ...
  joint_positions_3dof, ...
  joint_axes_3dof, ...
  joint_types_3dof, ...
  branch_matrix_3dof, ...
  {head}, ...
  link_positions_3dof, ...
  link_orientations_3dof, ...
  link_masses_3dof, ...
  link_moments_of_inertia_3dof ...
);

theta_init = [-pi/4; pi/4; pi/4];
theta_init = randn(3, 1);
arm.jointAngles = theta_init;
arm.updateInternals;

if show_visualization
    scene_bound = 4.5*[-1 1; -1 1; -1 1];

    stick_figure_3dof = KinematicTreeStickFigure(arm, scene_bound);
%     stick_figure_3dof.showLinkMassEllipsoids = true;
    view([1, 0, 0]);
    stick_figure_3dof.update();
end

J = arm.bodyJacobians{1};
A_f = J(2:3, :);
A_1 = J(2, :);
A_2 = J(3, :);

M = arm.inertiaMatrix;
N_prime = - arm.gravitationalTorqueMatrix;
lambda_f = (A_f*M^(-1)*A_f')^(-1) * (A_f*M^(-1)*N_prime)
lambda_1 = (A_1*M^(-1)*A_1')^(-1) * (A_1*M^(-1)*N_prime)
lambda_2 = (A_2*M^(-1)*A_2')^(-1) * (A_2*M^(-1)*N_prime)
constraint_torques_f = A_f'*lambda_f
constraint_torques_1 = A_1'*lambda_1
constraint_torques_2 = A_2'*lambda_2
constraint_torques_1 + constraint_torques_2






