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

joint_positions = {[0; 0; 0]; [1; 1; 0]; [-1; 1; 0]; };
joint_axes = {[0; 0; 1], [0; 0; 1], [0; 0; 1]};
joint_types = [1; 1; 1];
end_effectors = {[1; 2; 0], [-1; 2; 0]};
link_positions = {[0; 0.5; 0]; [1; 1.5; 0]; [-1; 1.5; 0];};
branch_matrix = [1 1 0; 1 0 1]; % each row is a branch, listing the joints that move the end-effector of that branch
degrees_of_freedom = length(joint_axes);
link_masses = ones(degrees_of_freedom, 1);
link_moments_of_inertia = [1 1 1; 1 1 1; 1 1 1;] * 0.1;
link_orientations = {eye(3), eye(3), eye(3)};

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

sceneBound = 3*[-1; 1; -1; 1; -1; 1];
stickFigure = KinematicTreeStickFigure(test_hand, sceneBound);
stickFigure.update();

% test_hand.jointAngles = 3*randn(degrees_of_freedom, 1);
% test_hand.jointVelocities = ones(degrees_of_freedom, 1);

test_hand.jointVelocities = [0.2; 1; -1];


timeStep = 0.01;
counter = 1;
while true
%     arm.calculateAccelerationsFromExternalTorques;
%     arm.jointVelocities = arm.jointVelocities + timeStep*arm.jointAccelerations;
    test_hand.jointAngles = test_hand.jointAngles + timeStep*test_hand.jointVelocities + timeStep^2*test_hand.jointAccelerations;
    test_hand.updateInternals;
    
    counter = counter+1;
    if counter == 2
        counter = 0;
        stickFigure.update();
        drawnow;
    end
end    
    
    
    
    
    
    
    
    
