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



joint_positions = {[0; 0; 0], [1; 0; 0], [2; 0; 0]};
joint_axes = {[0; 0; 1], [0; 0; 1], [0; 0; 1]};
joint_types = [1 1 1];
end_effector = [3; 0; 0];
link_positions = {[0.5; 0; 0], [1.5; 0; 0], [2.5; 0; 0]};
link_masses = [5, 5, 5];
link_moments_of_inertia = [1 1 1; 1 1 1; 1 1 1];

arm = GeneralKinematicChain ...
( ...
  joint_positions, ...
  joint_axes, ...
  joint_types, ...
  end_effector, ...
  link_positions, ...
  link_masses, ...
  link_moments_of_inertia ...
);

arm.jointAngles = [-1; 1; 1] * pi/2;
arm.jointVelocities = [-1; 1; 1] * -1;
arm.jointAccelerations = [0; 0; 0];
arm.updateInternals;

wall_position = 2;
wall_stiffness = 5000;

% add visualization data for the acceleration induced by the external wrench
arm.miscellaneousLinesStartPoints = zeros(3, 4);
arm.miscellaneousLinesEndPoints = zeros(3, 4);

stickFigure = KinematicChainStickFigure(arm, 4*[-1, 1, -1, 1, -1, 1]);
stickFigure.setMiscellaneousPlotColor(2, [1 0 1]);
stickFigure.setMiscellaneousPlotColor(3, [0 1 0]);
stickFigure.setMiscellaneousPlotColor(4, [0 0 0]);
arm.miscellaneousLinesStartPoints(:, 4) = [wall_position; -5; 0];
arm.miscellaneousLinesEndPoints(:, 4) = [wall_position; 5; 0];

% external_wrench = [-5; 0; 0; 0; 0; 0];





timeStep = 0.001;
counter = 1;
while true
    % calculate the wrench from the elastic contact
    indentation = max(arm.endEffectorPosition(1) - 2, 0);
    elastic_force = wall_stiffness * indentation;
    external_wrench = elastic_force * [-1; 0; 0; 0; 0; 0];
    
    % transform the external wrench into end-effector coordinates
    R_world_eef = arm.endEffectorTransformation(1:3, 1:3);
    T_eefRotation = [R_world_eef zeros(3,1); 0 0 0 1];
    A_eefRotation = createAdjointTransformation(T_eefRotation);
    eef_wrench = A_eefRotation' * external_wrench;
    
    % transform the wrench into joint torques
    eef_transformation_adjoint = createAdjointTransformation(arm.endEffectorTransformation);
    body_jacobian = invertAdjoint(eef_transformation_adjoint) * arm.spatialJacobian;
    torques_from_eef_wrench = body_jacobian' * eef_wrench;
    
    % apply torques and make euler step
    arm.externalTorques = torques_from_eef_wrench;
    arm.calculateAccelerationsFromExternalTorques;
    arm.jointVelocities = arm.jointVelocities + timeStep*arm.jointAccelerations;
    arm.jointAngles = arm.jointAngles + timeStep*arm.jointVelocities + timeStep^2*arm.jointAccelerations;
    arm.updateInternals;
    
    % update eef velocity/acceleration visualization
    joint_acceleration_from_eef_wrench = arm.inertiaMatrix^(-1) * torques_from_eef_wrench;
    eef_acceleration_from_eef_wrench = arm.endEffectorJacobian * joint_acceleration_from_eef_wrench;
    arm.miscellaneousLinesStartPoints(:, 1) = arm.endEffectorPosition;
    arm.miscellaneousLinesEndPoints(:, 1) = arm.endEffectorPosition + arm.endEffectorVelocity;
    arm.miscellaneousLinesStartPoints(:, 2) = arm.endEffectorPosition + arm.endEffectorVelocity;
    arm.miscellaneousLinesEndPoints(:, 2) = arm.endEffectorPosition + arm.endEffectorVelocity + arm.endEffectorAcceleration;
    arm.miscellaneousLinesStartPoints(:, 3) = arm.endEffectorPosition + arm.endEffectorVelocity;
    arm.miscellaneousLinesEndPoints(:, 3) = arm.endEffectorPosition + arm.endEffectorVelocity + eef_acceleration_from_eef_wrench;
    
    
    counter = counter+1;
    if counter == 10
        counter = 0;
        stickFigure.update();
        drawnow;
    end
end    
    
    
    
    
    
    
    
    
