% 3 joints
joint_positions = {[0; 0; 0]; [1; 0; 0]; [-1; 0; 0];};
joint_axes = {[0; 1; 0], [0; 1; 0], [0; 1; 0]};
joint_types = [1 1 1];
end_effectors = {[1; 0; -1], [-1; 0; -1]};
link_positions = {[0; 0; 0]; [1; 0; -0.5]; [-1; 0; -0.5];};
branch_matrix = [1 1 0; 1 0 1]; % each row is a branch, listing the joints that move the end-effector of that branch
link_orientations = {eye(3), eye(3), eye(3)};

% 2 joints out of these 3, to check
% joint_positions_chain = {[0; 0; 0]; [1; 0; 0]};
% joint_axes_chain = {[0; 1; 0], [0; 1; 0]};
% joint_types_chain = [1 1];
% end_effector_chain = [1; 0; -1];
% link_positions_chain = {[0; 0; 0]; [1; 0; -0.5];};
% link_orientations_chain = {eye(3), eye(3)};


% 5 joints, vertical
% joint_positions = {[0; 0; 0]; [0.5; 0; 0]; [0.5; 0; -1]; [-0.5; 0; 0]; [-0.5; 0; -1]; };
% joint_axes = {[0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0]};
% joint_types = [1 1 1 1 1];
% end_effectors = {[0.5; 0; -2], [-0.5; 0; -2]};
% link_positions = {[0; 0; 0]; [0.5; 0; -0.5]; [0.5; 0; -1.5]; [-0.5; 0; -0.5]; [-0.5; 0; -1.5];};
% branch_matrix = [1 1 1 0 0; 1 0 0 1 1]; % each row is a branch, listing the joints that move the end-effector of that branch
% link_orientations = {eye(3), eye(3), eye(3), eye(3), eye(3)};

% % 5 joints, human-like
% joint_positions = {[0; 0; 0]; [0.5; 0; 0]; [0.5; 0; -1]; [-0.5; 0; 0]; [-0.5; 0; -1]; };
% joint_axes = {[0; 1; 0], [1; 0; 0], [1; 0; 0], [1; 0; 0], [1; 0; 0]};
% joint_types = [1 1 1 1 1];
% end_effectors = {[0.5; 0; -2], [-0.5; 0; -2]};
% link_positions = {[0; 0; 0]; [0.5; 0; -0.5]; [0.5; 0; -1.5]; [-0.5; 0; -0.5]; [-0.5; 0; -1.5];};
% branch_matrix = [1 1 1 0 0; 1 0 0 1 1]; % each row is a branch, listing the joints that move the end-effector of that branch

% 5 joints, chain
% joint_positions = {[0; 0; 0]; [0.5; 0; 0]; [0.5; 0; -1]; [-0.5; 0; 0]; [-0.5; 0; -1]; };
% joint_axes = {[0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0]};
% joint_types = [1 1 1 1 1];
% end_effectors = {[0.5; 0; -2]};
% link_positions = {[0; 0; 0]; [0.5; 0; -0.5]; [0.5; 0; -1.5]; [-0.5; 0; -0.5]; [-0.5; 0; -1.5];};
% branch_matrix = [1 1 1 1 1]; % each row is a branch, listing the joints that move the end-effector of that branch
% link_orientations = {eye(3), eye(3), eye(3), eye(3), eye(3)};

% 7 joints
% joint_positions = {[0; 0; 0]; [0.5; 0; 0]; [0.5; 0; -1]; [0.5; 0; -1.5]; [-0.5; 0; 0]; [-0.5; 0; -1]; [-0.5; 0; -1.5]; };
% joint_axes = {[0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0]};
% joint_types = [1 1 1 1 1 1 1];
% end_effectors = {[0.5; 0; -2], [-0.5; 0; -2]};
% link_positions = {[0; 0; 0]; [0.5; 0; -0.5]; [0.5; 0; -1.25]; [0.5; 0; -1.75]; [-0.5; 0; -0.5]; [-0.5; 0; -1.25]; [-0.5; 0; -1.75];};
% link_orientations = {eye(3), eye(3), eye(3), eye(3), eye(3), eye(3), eye(3)};
% branch_matrix = ...
%   [ ...
%     1 1 1 1 0 0 0; ...
%     1 0 0 0 1 1 1; ...
%   ];

% reset(RandStream.getGlobalStream);

% randomize joint axes
% for i_joint = 1 : length(joint_axes)
%     joint_axis = rand(3, 1);
%     joint_axes{i_joint} = joint_axis * 1 / norm(joint_axis);
% end


degrees_of_freedom = length(joint_axes);
link_masses = ones(degrees_of_freedom, 1);
link_moments_of_inertia = [[1 5 5] ; repmat([1 1 .5], degrees_of_freedom-1, 1)] * 0.01;

link_masses = [200 300 100];
omega = [sqrt(2)/2; sqrt(2)/2; 0];
link_orientations = {[1 0 0; 0 1 0; 0 0 1], rodrigues(omega, pi/2), eye(3)};
% define link inertia
generalized_inertia_matrix_1 = ...
  [ ...
    link_masses(1)*eye(3) zeros(3); ...
    zeros(3) [30 0 0; 0 30 0; 0 0 2] ...
  ];
generalized_inertia_matrix_2 = ...
  [ ...
    link_masses(2)*eye(3) zeros(3); ...
    zeros(3) 2*[10 0 0; 0 10 0; 0 0 1] ...
  ];
generalized_inertia_matrix_3 = ...
  [ ...
    link_masses(3)*eye(3) zeros(3); ...
    zeros(3) 2*[10 0 0; 0 10 0; 0 0 1] ...
  ];
% generalized_inertia_matrix_1 = ...
%   [ ...
%     mass_1*eye(3) zeros(3); ...
%     zeros(3) [1 .2 0; .2 1 .2; 0 .2 1] ...
%   ];
% generalized_inertia_matrix_2 = ...
%   [ ...
%     mass_1*eye(3) zeros(3); ...
%     zeros(3) rodrigues(omega, pi/2)*[1 0 0; 0 1 0; 0 0 1] ...
%   ];
% generalized_inertia_matrix_3 = ...
%   [ ...
%     mass_1*eye(3) zeros(3); ...
%     zeros(3) rodrigues(omega, pi/2)*[1 0 0; 0 1 0; 0 0 1] ...
%   ];
generalized_link_inertia_matrices = {generalized_inertia_matrix_1; generalized_inertia_matrix_2; generalized_inertia_matrix_3};



test_hand = GeneralKinematicTree ...
( ...
  joint_positions, ...
  joint_axes, ...
  joint_types, ...
  branch_matrix, ...
  end_effectors, ...
  link_positions, ...
  link_orientations, ...
  generalized_link_inertia_matrices ...
);

test_hand.jointVelocities = 0.1*ones(test_hand.numberOfJoints, 1);
% test_hand.jointVelocities = ones(test_hand.numberOfJoints, 1);
% test_hand.jointVelocities([4 7]) = 1;
% test_hand.jointAngles(2) = -pi/4;
% test_hand_chain.jointAngles(2) = -pi/4;
% test_hand.jointAngles(5) = pi/4;
% test_hand.jointVelocities = randn(test_hand.numberOfJoints, 1);
test_hand.updateInternals();

sceneBound = 2.5*[-1; 1; -1; 1; -1; 1];
% shapeTypes = {'ellipsoid', 'cylinder-x', 'cylinder-x'};
% shapeTypes = {'cylinder-x', 'cylinder-x', 'cylinder-x'};
% shapeTypes = {'cylinder-x', 'ellipsoid', 'ellipsoid'};
shapeTypes = {'cuboid', 'cuboid', 'cuboid'};
stickFigure = KinematicTreeStickFigure(test_hand, sceneBound, shapeTypes);
set(gca, 'xlim', [stickFigure.sceneBound(1), stickFigure.sceneBound(2)], ...
         'ylim', [stickFigure.sceneBound(3), stickFigure.sceneBound(4)], ...
         'zlim',[stickFigure.sceneBound(5), stickFigure.sceneBound(6)]);
view([0, -1, 0]);
stickFigure.showLinkMassEllipsoids = true;
% view(-130, 20);
stickFigure.update();

return

timeStep = 0.01;
counter = 0;
while true
    test_hand.calculateAccelerationsFromExternalTorques;
    test_hand.jointVelocities = test_hand.jointVelocities + timeStep*test_hand.jointAccelerations;
    test_hand.jointAngles = test_hand.jointAngles + timeStep*test_hand.jointVelocities + timeStep^2*test_hand.jointAccelerations;
    test_hand.updateInternals;
    
    counter = counter+1;
    if counter == 3
        counter = 0;
        stickFigure.update;
        drawnow;
    end
end    
    
    
    
    
    
    
    
    
