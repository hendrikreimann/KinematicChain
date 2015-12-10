% compare the kinematic tree against the kinematic chain

real_joint_left_pos = [-1; 0; 0];
real_joint_right_pos = [1; 0; 0];
real_joint_left_axis = [0; 0; 1];
real_joint_right_axis = [0; 0; 1];
virtual_joint_pos = [-2; 0; 0];
virtual_joint_tree_pos = [0; 0; 0];
virtual_joint_one_axis = [1; 0; 0];
virtual_joint_two_axis = [0; 1; 0];
virtual_joint_three_axis = [0; 0; 1];
center_link_position = [0; 0; 0;];
left_link_position = [-1.5; 0; 0;];
right_link_position = [1.5; 0; 0;];

joint_positions = {virtual_joint_pos, virtual_joint_pos, virtual_joint_pos, real_joint_left_pos, real_joint_right_pos};
joint_axes = {virtual_joint_one_axis, virtual_joint_two_axis, virtual_joint_three_axis, real_joint_left_axis, real_joint_right_axis};
end_effector_pos = {[2; 0; 0]};
link_positions = {virtual_joint_pos, virtual_joint_pos, left_link_position, center_link_position, right_link_position};
link_masses = [0 0 1 2 1];
link_moments_of_inertia = [0 0 0; 0 0 0; 1 1 1; 5 5 5; 1 1 1] * 0.1;
branch_matrix = ...
  [ ...
    1 1 1 1 1; ...
  ];

link_orientations = {eye(3), eye(3), eye(3), eye(3), eye(3)};
joint_types = [2 2 1 1 1];

test_arm = GeneralKinematicTree ...
( ...
  joint_positions, ...
  joint_axes, ...
  joint_types, ...
  branch_matrix, ...
  end_effector_pos, ...
  link_positions, ...
  link_orientations, ...
  link_masses, ...
  link_moments_of_inertia ...
);

scene_bound = 10.0*[-1 1; -1 1; -1 1];

stick_figure = KinematicTreeStickFigure(test_arm, scene_bound);
view([0, 0, 1]);
stick_figure.update();

test_arm.jointAngles = [0; 0; pi/4; 0; 0] * 1;
test_arm.jointVelocities = [0; 0; 0; -1; 3] * 1;

test_arm.externalTorques = [0; 0; 0; 2; 1] * .1;
test_arm.externalTorques = [0; 0; 0; 2; 1] * 0;
time_step = 0.005;
counter = 0;
total_time = 10;
time = time_step : time_step : total_time;
number_of_time_steps = length(time);

joint_angle_trajectory = zeros(number_of_time_steps, 5);
joint_velocity_trajectory = zeros(number_of_time_steps, 5);
joint_acceleration_trajectory = zeros(number_of_time_steps, 5);
joint_torque_trajectory = zeros(number_of_time_steps, 5);
contact_force_trajectory = zeros(number_of_time_steps, 3);
center_of_mass_trajectory = zeros(number_of_time_steps, 3);
for i_time = 1 : length(time);
    % apply constraints
    A = [test_arm.endEffectorJacobians{1}([1 2], :); test_arm.bodyJacobians{1}(6, :)];
    ADot = [test_arm.endEffectorJacobianTemporalDerivatives{1}([1 2], :); zeros(1, 5)];
    M = test_arm.inertiaMatrix;
    lambda = (A*M^(-1)*A')^(-1) ...
        * (A*M^(-1)*(test_arm.externalTorques - test_arm.coriolisMatrix*test_arm.jointVelocities - test_arm.gravitationalTorqueMatrix) + ADot*test_arm.jointVelocities);
    test_arm.constraintTorques = A'*lambda * 0;
    constraint_torques = A'*lambda * 0;
    
    % make Euler step
    test_arm.calculateAccelerationsFromExternalTorques;
    test_arm.jointVelocities = test_arm.jointVelocities + time_step*test_arm.jointAccelerations;
    test_arm.jointAngles = test_arm.jointAngles + time_step*test_arm.jointVelocities + time_step^2*test_arm.jointAccelerations;
    test_arm.updateInternals;
    
    % store data
    joint_angle_trajectory(i_time, :) = test_arm.jointAngles;
    joint_velocity_trajectory(i_time, :) = test_arm.jointVelocities;
    joint_acceleration_trajectory(i_time, :) = test_arm.jointAccelerations;
    joint_torque_trajectory(i_time, :) = test_arm.externalTorques;
    contact_force_trajectory(i_time, :) = lambda;
    center_of_mass_trajectory(i_time, :) = test_arm.calculateCenterOfMassPosition;

    counter = counter+1;
    if counter == 5
        counter = 0;
        stick_figure.update;
        drawnow;
    end
end

% figure; axes; hold on; title('joint angles');
% plot(time, joint_angle_trajectory(:, 1), 'r-', 'linewidth', 1, 'displayname', 'chain 1');
% plot(time, joint_angle_trajectory(:, 2), 'g-', 'linewidth', 1, 'displayname', 'chain 2');
% plot(time, joint_angle_trajectory(:, 3), 'b-', 'linewidth', 1, 'displayname', 'chain 3');
% plot(time, joint_angle_trajectory(:, 4), 'c-', 'linewidth', 1, 'displayname', 'chain 4');
% plot(time, joint_angle_trajectory(:, 5), 'm-', 'linewidth', 1, 'displayname', 'chain 5');
% legend('toggle')

figure; axes; hold on; title('CoM position');
plot(time, center_of_mass_trajectory(:, 1), 'r-', 'linewidth', 1, 'displayname', 'x');
plot(time, center_of_mass_trajectory(:, 2), 'g-', 'linewidth', 1, 'displayname', 'y');
plot(time, center_of_mass_trajectory(:, 3), 'b-', 'linewidth', 1, 'displayname', 'z');
legend('toggle')





