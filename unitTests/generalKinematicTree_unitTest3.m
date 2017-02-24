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

% 5 joints, vertical
joint_positions = {[0; 0; 0]; [0.5; 0; 0]; [0.5; -1; 0]; [-0.5; 0; 0]; [-0.5; -1; 0]; };
joint_axes = {[1; 0; 0], [1; 0; 0], [0; 1; 0], [1; 0; 0], [0; 1; 0]};
joint_types = [2 2 2 2 2];
end_effectors = {[0.5; -2; 0], [-0.5; -2; 0]};
link_positions = {[0; 0; 0]; [0.5; -0.5; 0]; [0.5; -1.5; 0]; [-0.5; -0.5; 0]; [-0.5; -1.5; 0];};
branch_matrix = [1 1 1 0 0; 1 0 0 1 1]; % each row is a branch, listing the joints that move the end-effector of that branch
link_orientations = {eye(3), eye(3), eye(3), eye(3), eye(3)};



degrees_of_freedom = length(joint_axes);
link_masses = ones(degrees_of_freedom, 1);
link_moments_of_inertia = ones(degrees_of_freedom, 3) * 0.15;

test_hand = GeneralKinematicTree ...
( ...
  joint_positions, ...
  joint_axes, ...
  joint_types, ...
  branch_matrix, ...
  end_effectors, ...
  link_positions, ...
  link_masses, ...
  link_moments_of_inertia, ...
  link_orientations ...
);
test_hand.updateInternals;



% test_hand.jointAngles = 3*randn(degrees_of_freedom, 1);
% test_hand.jointVelocities = ones(degrees_of_freedom, 1);

% test_hand.jointVelocities = [0.2; 0.5; 0.5; -0.5; -0.5];
% test_hand.jointVelocities = [0.2; 0.1; -0.3; -0.6; -0.5; 1.0; -1.0];
% test_hand.jointVelocities = ones(test_hand.numberOfJoints, 1);

% test_hand.jointAngles = 1.0*randn(test_hand.numberOfJoints, 1);
% test_hand.jointAngles = [0; -3*pi/4; pi/4; pi/4; -pi/4];
% test_hand.jointAngles = [0; pi/4; -pi/3];
% test_hand.jointVelocities = 1.5*randn(test_hand.numberOfJoints, 1);
test_hand.jointVelocities = 0.1*ones(test_hand.numberOfJoints, 1);
% test_hand.jointVelocities(1) = 0;
% test_hand.jointVelocities = randn(test_hand.numberOfJoints, 1);
test_hand.updateInternals;



diff_hand = GeneralKinematicTree ...
( ...
  joint_positions, ...
  joint_axes, ...
  joint_types, ...
  branch_matrix, ...
  end_effectors, ...
  link_positions, ...
  link_masses, ...
  link_moments_of_inertia, ...
  link_orientations ...
);






sceneBound = 3*[-1; 1; -1; 1; -1; 1];
stickFigure = KinematicTreeStickFigure(test_hand, sceneBound);
set(gca, 'xlim', [stickFigure.sceneBound(1), stickFigure.sceneBound(2)], ...
         'ylim', [stickFigure.sceneBound(3), stickFigure.sceneBound(4)], ...
         'zlim',[stickFigure.sceneBound(5), stickFigure.sceneBound(6)]);
view([0, 0, 1]);
% stickFigure.showLinkMassEllipsoids = true;
% view(-130, 20);
stickFigure.update();

% return



external_torques = ones(test_hand.numberOfJoints, 1) * 10;

timeStep = 0.001;
counter = 1;
while true
    test_hand.externalTorques = external_torques; 
    
%     % apply constraints
    A = test_hand.endEffectorJacobians{2}([1 2], :);
    ADot = test_hand.endEffectorJacobianTemporalDerivatives{2}([1 2], :);
%     A = test_hand.endEffectorJacobians{1}(1, :);
%     ADot = test_hand.endEffectorJacobianTemporalDerivatives{1}(1, :);
    
    M = test_hand.inertiaMatrix;
    lambda = (A*M^(-1)*A')^(-1) ...
        * (A*M^(-1)*(test_hand.externalTorques - test_hand.coriolisMatrix*test_hand.jointVelocities - test_hand.gravitationalTorqueMatrix) + ADot*test_hand.jointVelocities);
    test_hand.constraintTorques = A'*lambda;
    constraint_torques = A'*lambda;
    
    
    
    test_hand.calculateAccelerationsFromExternalTorques;
    test_hand.jointVelocities = test_hand.jointVelocities + timeStep*test_hand.jointAccelerations;
    test_hand.jointAngles = test_hand.jointAngles + timeStep*test_hand.jointVelocities + timeStep^2*test_hand.jointAccelerations;
    test_hand.updateInternals;
    
    counter = counter+1;
    if counter == 2
        counter = 0;
        stickFigure.update;
        drawnow;
    end
end    
    
    
    
    
    
    
    
    
