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
end_effector_chain_pos = [2; 0; 0];
center_link_position = [0; 0; 0;];
left_link_position = [-1.5; 0; 0;];
right_link_position = [1.5; 0; 0;];

joint_positions_chain = {virtual_joint_chain_pos, virtual_joint_chain_pos, virtual_joint_chain_pos, real_joint_left_pos, real_joint_right_pos};
joint_axes_chain = {virtual_joint_one_axis, virtual_joint_two_axis, virtual_joint_three_axis, real_joint_left_axis_chain, real_joint_right_axis};
link_positions_chain = {virtual_joint_chain_pos, virtual_joint_chain_pos, left_link_position, center_link_position, right_link_position};
link_masses_chain = [0 0 1 2 1];
link_moments_of_inertia_chain = [0 0 0; 0 0 0; 1 1 1; 5 5 5; 1 1 1] * 0.1;

joint_positions_tree = {virtual_joint_tree_pos, virtual_joint_tree_pos, virtual_joint_tree_pos, real_joint_left_pos, real_joint_right_pos};
joint_axes_tree = {virtual_joint_one_axis, virtual_joint_two_axis, virtual_joint_three_axis, real_joint_left_axis_tree, real_joint_right_axis};
end_effectors_tree_pos = {[-2; 0; 0]; [2; 0; 0];};
link_positions_tree = {virtual_joint_tree_pos, virtual_joint_tree_pos, center_link_position, left_link_position, right_link_position};
link_masses_tree = [0 0 2 1 1];
link_moments_of_inertia_tree = [0 0 0; 0 0 0; 5 5 5; 1 1 1; 1 1 1] * 0.1;
branch_matrix = ...
  [ ...
    1 1 1 1 0; ...
    1 1 1 0 1; ...
  ];


link_orientations = {eye(3), eye(3), eye(3), eye(3), eye(3)};
joint_types = [2 2 1 1 1];

test_chain = GeneralKinematicChain ...
( ...
  joint_positions_chain, ...
  joint_axes_chain, ...
  joint_types, ...
  end_effector_chain_pos, ...
  link_positions_chain, ...
  link_masses_chain, ...
  link_moments_of_inertia_chain ...
);

test_tree = GeneralKinematicTree ...
( ...
  joint_positions_tree, ...
  joint_axes_tree, ...
  joint_types, ...
  branch_matrix, ...
  end_effectors_tree_pos, ...
  link_positions_tree, ...
  link_masses_tree, ...
  link_moments_of_inertia_tree, ...
  link_orientations ...
);

% test_chain.jointAngles(4) = 0.5;
% test_tree.jointAngles(4) = 0.5;
% test_chain.updateInternals();
% test_tree.updateInternals();

scene_bound = 2.5*[-1; 1; -1; 1; -1; 1];

stick_figure_chain = KinematicChainStickFigure(test_chain, scene_bound);
view([0, 0, 1]);
stick_figure_chain.update();

stick_figure_tree = KinematicTreeStickFigure(test_tree, scene_bound, stick_figure_chain.sceneAxes);
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
time_step = 0.001;
counter = 0;
while true
    % apply constraints
    A_chain = test_chain.endEffectorJacobian([1 2], :);
    A_chainDot = test_chain.endEffectorJacobianTemporalDerivative([1 2], :);
    M_chain = test_chain.inertiaMatrix;
    lambda_chain = (A_chain*M_chain^(-1)*A_chain')^(-1) ...
        * (A_chain*M_chain^(-1)*(test_chain.externalTorques - test_chain.coriolisMatrix*test_chain.jointVelocities - test_chain.gravitationalTorqueMatrix) + A_chainDot*test_chain.jointVelocities);
    test_chain.constraintTorques = A_chain'*lambda_chain;
    constraint_torques = A_chain'*lambda_chain;
    
    A_tree = test_tree.endEffectorJacobians{2}([1 2], :);
    A_treeDot = test_tree.endEffectorJacobianTemporalDerivatives{2}([1 2], :);
    M_tree = test_tree.inertiaMatrix;
    lambda_tree = (A_tree*M_tree^(-1)*A_tree')^(-1) ...
        * (A_tree*M_tree^(-1)*(test_tree.externalTorques - test_tree.coriolisMatrix*test_tree.jointVelocities - test_tree.gravitationalTorqueMatrix) + A_treeDot*test_tree.jointVelocities);
    test_tree.constraintTorques = A_tree'*lambda_tree;
    constraint_torques = A_tree'*lambda_tree;
    
    
    test_chain.calculateAccelerationsFromExternalTorques;
    test_chain.jointVelocities = test_chain.jointVelocities + time_step*test_chain.jointAccelerations;
    test_chain.jointAngles = test_chain.jointAngles + time_step*test_chain.jointVelocities + time_step^2*test_chain.jointAccelerations;
    test_chain.updateInternals;

    % check the weird error
    test_tree.calculateAccelerationsFromExternalTorques;
    test_tree.jointVelocities = test_tree.jointVelocities + time_step*test_tree.jointAccelerations;
    test_tree.jointAngles = test_tree.jointAngles + time_step*test_tree.jointVelocities + time_step^2*test_tree.jointAccelerations;
    test_tree.updateInternals;

    counter = counter+1;
    if counter == 5
        counter = 0;
        stick_figure_chain.update;
        stick_figure_tree.update;
        drawnow;
    end
end    