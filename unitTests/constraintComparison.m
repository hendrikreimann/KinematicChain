% compare the kinematic tree against the kinematic chain

real_joint_left_pos = [-1; 0; 0];
real_joint_right_pos = [1; 0; 0];
real_joint_left_axis_chain = [0; 0; 1];
real_joint_left_axis_tree = [0; 0; -1];
real_joint_right_axis = [0; 0; 1];
virtual_joint_chain_pos = [-2; 0; 0];
virtual_joint_tree_pos = [0; 0; 0];
virtual_joint_one_axis = [1; 0; 0];
virtual_joint_two_axis = [0; 1; 0];
virtual_joint_three_axis = [0; 0; 1];
center_link_position = [0; 0; 0;];
left_link_position = [-1.5; 0; 0;];
right_link_position = [1.5; 0; 0;];

joint_positions_chain = {virtual_joint_chain_pos, virtual_joint_chain_pos, virtual_joint_chain_pos, real_joint_left_pos, real_joint_right_pos};
joint_axes_chain = {virtual_joint_one_axis, virtual_joint_two_axis, virtual_joint_three_axis, real_joint_left_axis_chain, real_joint_right_axis};
end_effector_chain_pos = {[2; 0; 0]};
link_positions_chain = {virtual_joint_chain_pos, virtual_joint_chain_pos, left_link_position, center_link_position, right_link_position};
link_masses_chain = [0 0 1 2 1];
link_moments_of_inertia_chain = [0 0 0; 0 0 0; 1 1 1; 5 5 5; 1 1 1] * 0.1;
branch_matrix_chain = ...
  [ ...
    1 1 1 1 1; ...
  ];

joint_positions_tree = {virtual_joint_tree_pos, virtual_joint_tree_pos, virtual_joint_tree_pos, real_joint_left_pos, real_joint_right_pos};
joint_axes_tree = {virtual_joint_one_axis, virtual_joint_two_axis, virtual_joint_three_axis, real_joint_left_axis_tree, real_joint_right_axis};
end_effectors_tree_pos = {[-2; 0; 0]; [2; 0; 0];};
link_positions_tree = {virtual_joint_tree_pos, virtual_joint_tree_pos, center_link_position, left_link_position, right_link_position};
link_masses_tree = [0 0 2 1 1];
link_moments_of_inertia_tree = [0 0 0; 0 0 0; 5 5 5; 1 1 1; 1 1 1] * 0.1;
branch_matrix_tree = ...
  [ ...
    1 1 1 1 0; ...
    1 1 1 0 1; ...
  ];


link_orientations = {eye(3), eye(3), eye(3), eye(3), eye(3)};
joint_types = [2 2 1 1 1];

test_chain = GeneralKinematicTree ...
( ...
  joint_positions_chain, ...
  joint_axes_chain, ...
  joint_types, ...
  branch_matrix_chain, ...
  end_effector_chain_pos, ...
  link_positions_chain, ...
  link_orientations, ...
  link_masses_chain, ...
  link_moments_of_inertia_chain ...
);

test_tree = GeneralKinematicTree ...
( ...
  joint_positions_tree, ...
  joint_axes_tree, ...
  joint_types, ...
  branch_matrix_tree, ...
  end_effectors_tree_pos, ...
  link_positions_tree, ...
  link_orientations, ...
  link_masses_tree, ...
  link_moments_of_inertia_tree ...
);

% test_chain.jointAngles(4) = 0.5;
% test_tree.jointAngles(4) = 0.5;
% test_chain.updateInternals();
% test_tree.updateInternals();

scene_bound = 2.5*[-1; 1; -1; 1; -1; 1];

stick_figure_chain = KinematicTreeStickFigure(test_chain, scene_bound);
view([0, 0, 1]);
stick_figure_chain.update();

stick_figure_tree = KinematicTreeStickFigure(test_tree, scene_bound);
% stick_figure_tree = KinematicTreeStickFigure(test_tree, scene_bound);
% stick_figure_tree.showLinkMassEllipsoids = true;
position = get(stick_figure_tree.sceneFigure, 'Position');
position(1) = position(1) + position(3);
set(stick_figure_tree.sceneFigure, 'Position', position);

stick_figure_tree.setLinkPlotsLinewidth(5);
stick_figure_tree.setLinkPlotsColor([1 0.7 0]);
stick_figure_tree.update();

% return

test_chain.externalTorques = [0; 0; 0; 2; 1] * .1;
test_tree.externalTorques = [0; 0; 0; 2; 1] * .1;
time_step = 0.005;
counter = 0;
total_time = 10;
time = time_step : time_step : total_time;
number_of_time_steps = length(time);

joint_angles_chain = zeros(number_of_time_steps, 5);
joint_velocities_chain = zeros(number_of_time_steps, 5);
joint_accelerations_chain = zeros(number_of_time_steps, 5);
joint_torques_chain = zeros(number_of_time_steps, 5);
contact_forces_chain = zeros(number_of_time_steps, 3);
joint_angles_tree = zeros(number_of_time_steps, 5);
joint_velocities_tree = zeros(number_of_time_steps, 5);
joint_accelerations_tree = zeros(number_of_time_steps, 5);
joint_torques_tree = zeros(number_of_time_steps, 5);
contact_forces_tree = zeros(number_of_time_steps, 3);
for i_time = 1 : length(time);
    % apply constraints
    A_chain = [test_chain.endEffectorJacobians{1}([1 2], :); test_chain.bodyJacobians{1}(6, :)];
    A_chainDot = [test_chain.endEffectorJacobianTemporalDerivatives{1}([1 2], :); zeros(1, 5)];
    M_chain = test_chain.inertiaMatrix;
    lambda_chain = (A_chain*M_chain^(-1)*A_chain')^(-1) ...
        * (A_chain*M_chain^(-1)*(test_chain.externalTorques - test_chain.coriolisMatrix*test_chain.jointVelocities - test_chain.gravitationalTorqueMatrix) + A_chainDot*test_chain.jointVelocities);
    test_chain.constraintTorques = A_chain'*lambda_chain;
    constraint_torques_chain = A_chain'*lambda_chain;
    
    A_tree = [test_tree.endEffectorJacobians{2}([1 2], :); test_tree.bodyJacobians{2}(6, :)];
    A_treeDot = [test_tree.endEffectorJacobianTemporalDerivatives{2}([1 2], :); zeros(1, 5)];
    M_tree = test_tree.inertiaMatrix;
    lambda_tree = (A_tree*M_tree^(-1)*A_tree')^(-1) ...
        * (A_tree*M_tree^(-1)*(test_tree.externalTorques - test_tree.coriolisMatrix*test_tree.jointVelocities - test_tree.gravitationalTorqueMatrix) + A_treeDot*test_tree.jointVelocities);
    test_tree.constraintTorques = A_tree'*lambda_tree;
    constraint_torques_tree = A_tree'*lambda_tree;
    
    % make Euler step
    test_chain.calculateAccelerationsFromExternalTorques;
    test_chain.jointVelocities = test_chain.jointVelocities + time_step*test_chain.jointAccelerations;
    test_chain.jointAngles = test_chain.jointAngles + time_step*test_chain.jointVelocities + time_step^2*test_chain.jointAccelerations;
    test_chain.updateInternals;

    test_tree.calculateAccelerationsFromExternalTorques;
    test_tree.jointVelocities = test_tree.jointVelocities + time_step*test_tree.jointAccelerations;
    test_tree.jointAngles = test_tree.jointAngles + time_step*test_tree.jointVelocities + time_step^2*test_tree.jointAccelerations;
    test_tree.updateInternals;
    
    % store data
    joint_angles_chain(i_time, :) = test_chain.jointAngles;
    joint_velocities_chain(i_time, :) = test_chain.jointVelocities;
    joint_accelerations_chain(i_time, :) = test_chain.jointAccelerations;
    joint_torques_chain(i_time, :) = test_chain.externalTorques;
    contact_forces_chain(i_time, :) = lambda_chain;
    joint_angles_tree(i_time, :) = test_tree.jointAngles;
    joint_velocities_tree(i_time, :) = test_tree.jointVelocities;
    joint_accelerations_tree(i_time, :) = test_tree.jointAccelerations;
    joint_torques_tree(i_time, :) = test_tree.externalTorques;
    contact_forces_tree(i_time, :) = lambda_tree;

    counter = counter+1;
    if counter == 5
        counter = 0;
        stick_figure_chain.update;
        stick_figure_tree.update;
        drawnow;
    end
end

figure; axes; hold on; title('joint angles');
plot(time, joint_angles_chain(:, 1), 'r-', 'linewidth', 1, 'displayname', 'chain 1');
plot(time, joint_angles_chain(:, 2), 'g-', 'linewidth', 1, 'displayname', 'chain 2');
plot(time, joint_angles_chain(:, 3), 'b-', 'linewidth', 1, 'displayname', 'chain 3');
plot(time, joint_angles_chain(:, 4), 'c-', 'linewidth', 1, 'displayname', 'chain 4');
plot(time, joint_angles_chain(:, 5), 'm-', 'linewidth', 1, 'displayname', 'chain 5');
plot(time, joint_angles_tree(:, 1), 'r:', 'linewidth', 2, 'displayname', 'tree 1');
plot(time, joint_angles_tree(:, 2), 'g:', 'linewidth', 2, 'displayname', 'tree 2');
plot(time, joint_angles_tree(:, 3), 'b:', 'linewidth', 2, 'displayname', 'tree 3');
plot(time, joint_angles_tree(:, 4), 'c:', 'linewidth', 2, 'displayname', 'tree 4');
plot(time, joint_angles_tree(:, 5), 'm:', 'linewidth', 2, 'displayname', 'tree 5');
legend('toggle')


figure; axes; hold on; title('contact forces');
plot(time, contact_forces_chain(:, 1), 'r-', 'linewidth', 1, 'displayname', 'chain 1');
plot(time, contact_forces_chain(:, 2), 'g-', 'linewidth', 1, 'displayname', 'chain 1');
plot(time, contact_forces_chain(:, 3), 'b-', 'linewidth', 1, 'displayname', 'chain 1');
plot(time, contact_forces_tree(:, 1), 'r:', 'linewidth', 1, 'displayname', 'tree 1');
plot(time, contact_forces_tree(:, 2), 'g:', 'linewidth', 1, 'displayname', 'tree 1');
plot(time, contact_forces_tree(:, 3), 'b:', 'linewidth', 1, 'displayname', 'tree 1');


