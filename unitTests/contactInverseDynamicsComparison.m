% compares three different representations of the same physical system

show_visualization = 1;
simulate_new_data = 1;
do_inverse_dynamics = 1;
plot_angles = 0;
plot_torques = 0;
plot_induced_accelerations = 1;

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

joint_positions_6dof = {contact, contact, contact, ankle, knee, hip};
joint_axes_6dof = {[0; 1; 0], [0; 0; 1], [1; 0; 0], [1; 0; 0], [1; 0; 0], [1; 0; 0]};
joint_types_6dof = [2 2 1 1 1 1];
branch_matrix_6dof = [1 1 1 1 1 1];
link_positions_6dof = {feet_position, feet_position, feet_position, shank_position, thigh_position, torso_position};
link_orientations_6dof = {eye(3), eye(3), eye(3), eye(3), eye(3), eye(3)};
link_masses_6dof = [0 0 1 1 1 2];
link_moments_of_inertia_6dof = [0 0 0; 0 0 0; 1 1 1; 1 1 1; 5 5 5; 1 1 1] * 0.1;

joint_positions_6dofinv = {head, head, head, hip, knee, ankle};
joint_axes_6dofinv = {[0; 1; 0], [0; 0; 1], [1; 0; 0], -[1; 0; 0], -[1; 0; 0], -[1; 0; 0]};
joint_types_6dofinv = [2 2 1 1 1 1];
branch_matrix_6dofinv = [1 1 1 1 1 1];
link_positions_6dofinv = {torso_position, torso_position, torso_position, thigh_position, shank_position, feet_position};
link_orientations_6dofinv = {eye(3), eye(3), eye(3), eye(3), eye(3), eye(3)};
link_masses_6dofinv = [0 0 2 1 1 1];
link_moments_of_inertia_6dofinv = [0 0 0; 0 0 0; 1 1 1; 5 5 5; 1 1 1; 1 1 1;] * 0.1;

test_3dof = GeneralKinematicTree ...
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

test_6dof = GeneralKinematicTree ...
( ...
  joint_positions_6dof, ...
  joint_axes_6dof, ...
  joint_types_6dof, ...
  branch_matrix_6dof, ...
  {head}, ...
  link_positions_6dof, ...
  link_orientations_6dof, ...
  link_masses_6dof, ...
  link_moments_of_inertia_6dof ...
);

test_6dofinv = GeneralKinematicTree ...
( ...
  joint_positions_6dofinv, ...
  joint_axes_6dofinv, ...
  joint_types_6dofinv, ...
  branch_matrix_6dofinv, ...
  {contact}, ...
  link_positions_6dofinv, ...
  link_orientations_6dofinv, ...
  link_masses_6dofinv, ...
  link_moments_of_inertia_6dofinv ...
);

theta_init = [0; 0; pi/2];
test_3dof.jointAngles = theta_init;
test_6dof.jointAngles = [0; 0; 0; theta_init];
test_6dofinv.jointAngles = [-1; -1; pi/2; theta_init(3); theta_init(2); theta_init(1)];
test_3dof.updateInternals;
test_6dof.updateInternals;
test_6dofinv.updateInternals;

%% show visualization
if show_visualization
    scene_bound = 4.5*[-1 1; -1 1; -1 1];

    stick_figure_3dof = KinematicTreeStickFigure(test_3dof, scene_bound);
%     stick_figure_3dof.showLinkMassEllipsoids = true;
    view([1, 0, 0]);
    stick_figure_3dof.update();

    stick_figure_6dof = KinematicTreeStickFigure(test_6dof, scene_bound);
    position = get(stick_figure_6dof.sceneFigure, 'Position');
    position(1) = position(1) + position(3);
    set(stick_figure_6dof.sceneFigure, 'Position', position);
    view([1, 0, 0]);
    % stick_figure_6dof.setLinkPlotsLinewidth(5);
    % stick_figure_6dof.setLinkPlotsColor([1 0.7 0]);
%     stick_figure_6dof.showLinkMassEllipsoids = true;
    stick_figure_6dof.update();
    
    stick_figure_6dofinv = KinematicTreeStickFigure(test_6dofinv, scene_bound);
    position = get(stick_figure_6dofinv.sceneFigure, 'Position');
    position(1) = position(1) + 2*position(3);
    set(stick_figure_6dofinv.sceneFigure, 'Position', position);
    view([1, 0, 0]);
    % stick_figure_6dofinv.setLinkPlotsLinewidth(5);
    % stick_figure_6dofinv.setLinkPlotsColor([1 0.7 0]);
%     stick_figure_6dofinv.showLinkMassEllipsoids = true;
    stick_figure_6dofinv.update();
    
end

%% forward dynamics
if simulate_new_data
    
    time_step = 0.002;
    total_time = 1;
    time = time_step : time_step : total_time;
    number_of_time_steps = length(time);

    timeseries_joint_angles_3dof = zeros(number_of_time_steps, 3);
    timeseries_joint_velocity_3dof = zeros(number_of_time_steps, 3);
    timeseries_joint_acceleration_3dof = zeros(number_of_time_steps, 3);

    timeseries_joint_angles_6dof = zeros(number_of_time_steps, 6);
    timeseries_joint_velocity_6dof = zeros(number_of_time_steps, 6);
    timeseries_joint_acceleration_6dof = zeros(number_of_time_steps, 6);

    timeseries_joint_angles_6dofinv = zeros(number_of_time_steps, 6);
    timeseries_joint_velocity_6dofinv = zeros(number_of_time_steps, 6);
    timeseries_joint_acceleration_6dofinv = zeros(number_of_time_steps, 6);

    external_torque_direction = [3; 2; 1];
    amplitude = 5;
    frequency = 1.2;
    sinusoid_torque = amplitude * (-cos(time * 2 * pi * frequency / total_time)+1);

    counter = 0;
    for i_time = 1 : number_of_time_steps
        % calculate external torques
        test_3dof.externalTorques = external_torque_direction * sinusoid_torque(i_time);
        test_6dof.externalTorques = [0; 0; 0; test_3dof.externalTorques];
        test_6dofinv.externalTorques = [0; 0; 0; test_3dof.externalTorques(3); test_3dof.externalTorques(2); test_3dof.externalTorques(1)];

        % apply constraints - 6dof
        A_6dof = eye(3, 6);
        A_6dofDot = zeros(3, 6);
        M_6dof = test_6dof.inertiaMatrix;
        lambda_6dof = (A_6dof*M_6dof^(-1)*A_6dof')^(-1) ...
            * (A_6dof*M_6dof^(-1)*(test_6dof.externalTorques - test_6dof.coriolisMatrix*test_6dof.jointVelocities - test_6dof.gravitationalTorqueMatrix) + A_6dofDot*test_6dof.jointVelocities);
        test_6dof.constraintTorques = A_6dof'*lambda_6dof;
        constraint_torques_6dof = A_6dof'*lambda_6dof;

        % apply constraints - 6dofinv
        A_6dofinv = [test_6dofinv.endEffectorJacobians{1}(2:3, :); test_6dofinv.bodyJacobians{1}(4, :)];
        A_6dofinvDot = [test_6dofinv.endEffectorJacobianTemporalDerivatives{1}(2:3, :); test_6dofinv.bodyJacobianTemporalDerivatives{1}(4, :)];
        M_6dofinv = test_6dofinv.inertiaMatrix;
        lambda_6dofinv = (A_6dofinv*M_6dofinv^(-1)*A_6dofinv')^(-1) ...
            * (A_6dofinv*M_6dofinv^(-1)*(test_6dofinv.externalTorques - test_6dofinv.coriolisMatrix*test_6dofinv.jointVelocities - test_6dofinv.gravitationalTorqueMatrix) + A_6dofinvDot*test_6dofinv.jointVelocities);
        test_6dofinv.constraintTorques = A_6dofinv'*lambda_6dofinv;
        constraint_torques_6dofinv = A_6dofinv'*lambda_6dofinv;

        if i_time > 50
%             % check modified equation of motion
%             test_6dof.calculateAccelerationsFromExternalTorques;
%             P_6dof = eye(6) - A_6dof' * (A_6dof*M_6dof^(-1)*A_6dof')^(-1) * A_6dof*M_6dof^(-1);
%             test_6dof.inertiaMatrix*test_6dof.jointAccelerations ...
%                 + P_6dof * test_6dof.coriolisMatrix*test_6dof.jointVelocities ...
%                 + P_6dof * test_6dof.gravitationalTorqueMatrix ...
%                 + A_6dof' * (A_6dof*M_6dof^(-1)*A_6dof')^(-1) * A_6dofDot * test_6dof.jointVelocities
%             P_6dof * test_6dof.externalTorques
%             test_6dof.externalTorques
% 
%             % check modified equation of motion
%             test_6dofinv.calculateAccelerationsFromExternalTorques;
%             P_6dofinv = eye(6) - A_6dofinv' * (A_6dofinv*M_6dofinv^(-1)*A_6dofinv')^(-1) * A_6dofinv*M_6dofinv^(-1);
%             test_6dofinv.inertiaMatrix*test_6dofinv.jointAccelerations ...
%                 + P_6dofinv * test_6dofinv.coriolisMatrix*test_6dofinv.jointVelocities ...
%                 + P_6dofinv * test_6dofinv.gravitationalTorqueMatrix ...
%                 + A_6dofinv' * (A_6dofinv*M_6dofinv^(-1)*A_6dofinv')^(-1) * A_6dofinvDot * test_6dofinv.jointVelocities
%             P_6dofinv * test_6dofinv.externalTorques
%             test_6dofinv.externalTorques
        end
        
        % euler step 3dof
        test_3dof.calculateAccelerationsFromExternalTorques;
        test_3dof.jointVelocities = test_3dof.jointVelocities + time_step*test_3dof.jointAccelerations;
        test_3dof.jointAngles = test_3dof.jointAngles + time_step*test_3dof.jointVelocities + time_step^2*test_3dof.jointAccelerations;
        test_3dof.updateInternals;

        % euler step 6dof
        test_6dof.calculateAccelerationsFromExternalTorques;
        test_6dof.jointVelocities = test_6dof.jointVelocities + time_step*test_6dof.jointAccelerations;
        test_6dof.jointAngles = test_6dof.jointAngles + time_step*test_6dof.jointVelocities + time_step^2*test_6dof.jointAccelerations;
        test_6dof.updateInternals;

        % euler step 6dofinv
        test_6dofinv.calculateAccelerationsFromExternalTorques;
        test_6dofinv.jointVelocities = test_6dofinv.jointVelocities + time_step*test_6dofinv.jointAccelerations;
        test_6dofinv.jointAngles = test_6dofinv.jointAngles + time_step*test_6dofinv.jointVelocities + time_step^2*test_6dofinv.jointAccelerations;
        test_6dofinv.updateInternals;

        % store data
        timeseries_joint_angles_3dof(i_time, :) = test_3dof.jointAngles;
        timeseries_joint_velocity_3dof(i_time, :) = test_3dof.jointVelocities;
        timeseries_joint_acceleration_3dof(i_time, :) = test_3dof.jointAccelerations;
        timeseries_joint_angles_6dof(i_time, :) = test_6dof.jointAngles;
        timeseries_joint_velocity_6dof(i_time, :) = test_6dof.jointVelocities;
        timeseries_joint_acceleration_6dof(i_time, :) = test_6dof.jointAccelerations;
        timeseries_joint_angles_6dofinv(i_time, :) = test_6dofinv.jointAngles;
        timeseries_joint_velocity_6dofinv(i_time, :) = test_6dofinv.jointVelocities;
        timeseries_joint_acceleration_6dofinv(i_time, :) = test_6dofinv.jointAccelerations;

        counter = counter+1;
        if (counter == 5) && show_visualization
            counter = 0;
            stick_figure_3dof.update;
            stick_figure_6dof.update;
            stick_figure_6dofinv.update;
            drawnow;
        end
    end
end

%% inverse dynamics
if do_inverse_dynamics
    timeseries_joint_torques_applied_3dof = zeros(number_of_time_steps, 3);
    timeseries_joint_torques_gravity_3dof = zeros(number_of_time_steps, 3);
    timeseries_joint_torques_coriolis_3dof = zeros(number_of_time_steps, 3);
    timeseries_joint_torques_acceleration_dependent_3dof = zeros(number_of_time_steps, 3);
    timeseries_joint_torques_constraint_3dof = zeros(number_of_time_steps, 3);
    timeseries_joint_torques_net_3dof = zeros(number_of_time_steps, 3);
    timeseries_accelerations_induced_by_applied_3dof = zeros(number_of_time_steps, 3);
    timeseries_accelerations_induced_by_gravity_3dof = zeros(number_of_time_steps, 3);
    timeseries_accelerations_induced_by_movement_3dof = zeros(number_of_time_steps, 3);
    timeseries_accelerations_induced_by_constraint_3dof = zeros(number_of_time_steps, 3);
    
    timeseries_joint_torques_applied_6dof = zeros(number_of_time_steps, 6);
    timeseries_joint_torques_gravity_6dof = zeros(number_of_time_steps, 6);
    timeseries_joint_torques_coriolis_6dof = zeros(number_of_time_steps, 6);
    timeseries_joint_torques_acceleration_dependent_6dof = zeros(number_of_time_steps, 6);
    timeseries_joint_torques_constraint_6dof = zeros(number_of_time_steps, 6);
    timeseries_joint_torques_net_6dof = zeros(number_of_time_steps, 6);
    timeseries_accelerations_induced_by_applied_6dof = zeros(number_of_time_steps, 6);
    timeseries_accelerations_induced_by_gravity_6dof = zeros(number_of_time_steps, 6);
    timeseries_accelerations_induced_by_movement_6dof = zeros(number_of_time_steps, 6);
    timeseries_accelerations_induced_by_constraint_6dof = zeros(number_of_time_steps, 6);

    timeseries_joint_torques_applied_6dofinv = zeros(number_of_time_steps, 6);
    timeseries_joint_torques_gravity_6dofinv = zeros(number_of_time_steps, 6);
    timeseries_joint_torques_coriolis_6dofinv = zeros(number_of_time_steps, 6);
    timeseries_joint_torques_acceleration_dependent_6dofinv = zeros(number_of_time_steps, 6);
    timeseries_joint_torques_constraint_6dofinv = zeros(number_of_time_steps, 6);
    timeseries_joint_torques_net_6dofinv = zeros(number_of_time_steps, 6);
    timeseries_accelerations_induced_by_applied_6dofinv = zeros(number_of_time_steps, 6);
    timeseries_accelerations_induced_by_gravity_6dofinv = zeros(number_of_time_steps, 6);
    timeseries_accelerations_induced_by_movement_6dofinv = zeros(number_of_time_steps, 6);
    timeseries_accelerations_induced_by_constraint_6dofinv = zeros(number_of_time_steps, 6);

    for i_time = 2 : number_of_time_steps
        % get current data
        theta = timeseries_joint_angles_3dof(i_time-1, :)';
        theta_dot = timeseries_joint_velocity_3dof(i_time-1, :)';
        theta_two_dot = timeseries_joint_acceleration_3dof(i_time, :)';

        % inverse dynamics 3dof
        test_3dof.jointAngles = theta;
        test_3dof.jointVelocities = theta_dot;
        test_3dof.jointAccelerations = theta_two_dot;
        test_3dof.updateInternals;
        A_3dof = zeros(3);
        A_3dofDot = zeros(3);
        M_3dof = test_3dof.inertiaMatrix;
        inertia_matrix_diagonal = zeros(size(test_3dof.inertiaMatrix));
        for i_joint = 1 : length(test_3dof.inertiaMatrix)
            inertia_matrix_diagonal(i_joint, i_joint) = test_3dof.inertiaMatrix(i_joint, i_joint);
        end
        inertia_matrix_off_diagonal = test_3dof.inertiaMatrix - inertia_matrix_diagonal;
        tau_3dof = test_3dof.inertiaMatrix*test_3dof.jointAccelerations ...
                          + test_3dof.coriolisMatrix*test_3dof.jointVelocities ...
                          + test_3dof.gravitationalTorqueMatrix;


        timeseries_joint_torques_applied_3dof(i_time, :) = tau_3dof;
        timeseries_joint_torques_gravity_3dof(i_time, :) = - test_3dof.gravitationalTorqueMatrix;
        timeseries_joint_torques_coriolis_3dof(i_time, :) = - test_3dof.coriolisMatrix*test_3dof.jointVelocities;
        timeseries_joint_torques_acceleration_dependent_3dof(i_time, :) = - inertia_matrix_off_diagonal*test_3dof.jointAccelerations;
        timeseries_joint_torques_net_3dof(i_time, :) = inertia_matrix_diagonal*test_3dof.jointAccelerations;
        
        
        % inverse dynamics 6dof
        test_6dof.jointAngles = [zeros(3, 1); theta];
        test_6dof.jointVelocities = [zeros(3, 1); theta_dot];
        test_6dof.jointAccelerations = [zeros(3, 1); theta_two_dot];
        test_6dof.updateInternals;
        virtual_joints = 1:3;
        A_6dof = eye(3, 6);
        A_6dofDot = zeros(3, 6);
        M_6dof = test_6dof.inertiaMatrix;
        A_transposed_times_lambda_without_F = - test_6dof.inertiaMatrix*test_6dof.jointAccelerations - test_6dof.coriolisMatrix*test_6dof.jointVelocities - test_6dof.gravitationalTorqueMatrix;
        G_gamma = A_transposed_times_lambda_without_F(virtual_joints, :);
        A_virtual = A_6dof(:, virtual_joints);
        prod = A_virtual * A_virtual';
        lambda_reconstructed = (A_virtual * A_virtual')^(-1) * A_virtual * G_gamma;
        constraint_torque_reconstructed = A_6dof'*lambda_reconstructed;
        test_6dof.constraintTorques = constraint_torque_reconstructed;

        inertia_matrix_diagonal = zeros(size(test_6dof.inertiaMatrix));
        for i_joint = 1 : length(test_6dof.inertiaMatrix)
            inertia_matrix_diagonal(i_joint, i_joint) = test_6dof.inertiaMatrix(i_joint, i_joint);
        end
        inertia_matrix_off_diagonal = test_6dof.inertiaMatrix - inertia_matrix_diagonal;
        tau_6dof = test_6dof.inertiaMatrix*test_6dof.jointAccelerations ...
                          + test_6dof.constraintTorques ...
                          + test_6dof.coriolisMatrix*test_6dof.jointVelocities ...
                          + test_6dof.gravitationalTorqueMatrix;

        timeseries_joint_torques_applied_6dof(i_time, :) = tau_6dof;
        timeseries_joint_torques_gravity_6dof(i_time, :) = - test_6dof.gravitationalTorqueMatrix;
        timeseries_joint_torques_constraint_6dof(i_time, :) = - test_6dof.constraintTorques;
        timeseries_joint_torques_coriolis_6dof(i_time, :) = - test_6dof.coriolisMatrix*test_6dof.jointVelocities;
        timeseries_joint_torques_acceleration_dependent_6dof(i_time, :) = - inertia_matrix_off_diagonal*test_6dof.jointAccelerations;
        timeseries_joint_torques_net_6dof(i_time, :) = inertia_matrix_diagonal*test_6dof.jointAccelerations;

        
        
%         % inverse dynamics - 6dof - alternative, checking whether this way works
%         G = - M_6dof*test_6dof.jointAccelerations - test_6dof.coriolisMatrix*test_6dof.jointVelocities - test_6dof.gravitationalTorqueMatrix;
%         k_c = rank(A_6dof);
%         k_v = 3;
%         [~, ~, V_c] = svd(A_6dof);
%         C_c = V_c(:, k_c+1:end);
%         B_v = [eye(k_v); zeros(test_6dof.numberOfJoints - k_v, k_v)];
%         k_w = rank([B_v C_c]);
%         [~, ~, V_w] = svd([B_v C_c]');
%         C_w = V_w(:, k_w+1:end);
%         D = [B_v C_w];
% 
%         lambda = pinv(D'*A_6dof')*D'*G;
%         
%         
%         F = M_6dof*test_6dof.jointAccelerations + test_6dof.coriolisMatrix*test_6dof.jointVelocities + test_6dof.gravitationalTorqueMatrix + A_6dof'*lambda;
%         test_zero = D' * F;
        
        
        
        
        
        
        
        % inverse dynamics 6dofinv
        theta_prime = timeseries_joint_angles_6dofinv(i_time-1, :)';
        theta_dot_prime = timeseries_joint_velocity_6dofinv(i_time-1, :)';
        theta_two_dot_prime = timeseries_joint_acceleration_6dofinv(i_time, :)';

        test_6dofinv.jointAngles = timeseries_joint_angles_6dofinv(i_time-1, :)';
        test_6dofinv.jointVelocities = timeseries_joint_velocity_6dofinv(i_time-1, :)';
        test_6dofinv.jointAccelerations = timeseries_joint_acceleration_6dofinv(i_time, :)';
        test_6dofinv.updateInternals;
        virtual_joints = 1:3;
        real_joints = 4:6;
        A_6dofinv = [test_6dofinv.endEffectorJacobians{1}(2:3, :); test_6dofinv.bodyJacobians{1}(4, :)];
        A_6dofinvDot = [test_6dofinv.endEffectorJacobianTemporalDerivatives{1}(2:3, :); test_6dofinv.bodyJacobianTemporalDerivatives{1}(4, :)];
        M_6dofinv = test_6dofinv.inertiaMatrix;
        A_transposed_times_lambda_without_F = - test_6dofinv.inertiaMatrix*test_6dofinv.jointAccelerations - test_6dofinv.coriolisMatrix*test_6dofinv.jointVelocities - test_6dofinv.gravitationalTorqueMatrix;
        G_gamma = A_transposed_times_lambda_without_F(virtual_joints, :);
        A_virtual = A_6dofinv(:, virtual_joints);
        prod = A_virtual * A_virtual';
        prod_inv = prod^(-1);
        lambda_reconstructed = (A_virtual * A_virtual')^(-1) * A_virtual * G_gamma;
        constraint_torque_reconstructed = A_6dofinv'*lambda_reconstructed;
        test_6dofinv.constraintTorques = constraint_torque_reconstructed;

        inertia_matrix_diagonal = zeros(size(test_6dofinv.inertiaMatrix));
        for i_joint = 1 : length(test_6dofinv.inertiaMatrix)
            inertia_matrix_diagonal(i_joint, i_joint) = test_6dofinv.inertiaMatrix(i_joint, i_joint);
        end
        inertia_matrix_off_diagonal = test_6dofinv.inertiaMatrix - inertia_matrix_diagonal;
        tau_6dofinv = test_6dofinv.inertiaMatrix*test_6dofinv.jointAccelerations ...
                          + test_6dofinv.constraintTorques ...
                          + test_6dofinv.coriolisMatrix*test_6dofinv.jointVelocities ...
                          + test_6dofinv.gravitationalTorqueMatrix;

        timeseries_joint_torques_applied_6dofinv(i_time, :) = tau_6dofinv;
        timeseries_joint_torques_gravity_6dofinv(i_time, :) = - test_6dofinv.gravitationalTorqueMatrix;
        timeseries_joint_torques_constraint_6dofinv(i_time, :) = - test_6dofinv.constraintTorques;
        timeseries_joint_torques_coriolis_6dofinv(i_time, :) = - test_6dofinv.coriolisMatrix*test_6dofinv.jointVelocities;
        timeseries_joint_torques_acceleration_dependent_6dofinv(i_time, :) = - inertia_matrix_off_diagonal*test_6dofinv.jointAccelerations;
        timeseries_joint_torques_net_6dofinv(i_time, :) = inertia_matrix_diagonal*test_6dofinv.jointAccelerations;

        
        % calculate induced accelerations
        timeseries_accelerations_induced_by_applied_3dof(i_time, :) = test_3dof.inertiaMatrix^(-1) * tau_3dof;
        timeseries_accelerations_induced_by_gravity_3dof(i_time, :) = - test_3dof.inertiaMatrix^(-1) * test_3dof.gravitationalTorqueMatrix;
        timeseries_accelerations_induced_by_movement_3dof(i_time, :) = - test_3dof.inertiaMatrix^(-1) * test_3dof.coriolisMatrix * test_3dof.jointVelocities;
        
        P_6dof = eye(test_6dof.numberOfJoints) - A_6dof' * (A_6dof*M_6dof^(-1)*A_6dof')^(-1) * A_6dof*M_6dof^(-1);
        timeseries_accelerations_induced_by_applied_6dof(i_time, :) = test_6dof.inertiaMatrix^(-1) * P_6dof * tau_6dof;
        timeseries_accelerations_induced_by_gravity_6dof(i_time, :) = - test_6dof.inertiaMatrix^(-1) * P_6dof * test_6dof.gravitationalTorqueMatrix;
        timeseries_accelerations_induced_by_movement_6dof(i_time, :) = - test_6dof.inertiaMatrix^(-1) * (P_6dof*test_6dof.coriolisMatrix + A_6dof' * (A_6dof*M_6dof^(-1)*A_6dof')^(-1) * A_6dofDot) * test_6dof.jointVelocities;

        P_6dofinv = eye(test_6dofinv.numberOfJoints) - A_6dofinv' * (A_6dofinv*M_6dofinv^(-1)*A_6dofinv')^(-1) * A_6dofinv*M_6dofinv^(-1);
        timeseries_accelerations_induced_by_applied_6dofinv(i_time, :) = test_6dofinv.inertiaMatrix^(-1) * P_6dofinv * tau_6dofinv;
        timeseries_accelerations_induced_by_gravity_6dofinv(i_time, :) = - test_6dofinv.inertiaMatrix^(-1) * P_6dofinv * test_6dofinv.gravitationalTorqueMatrix;
        timeseries_accelerations_induced_by_movement_6dofinv(i_time, :) = - test_6dofinv.inertiaMatrix^(-1) * (P_6dofinv*test_6dofinv.coriolisMatrix + A_6dofinv' * (A_6dofinv*M_6dofinv^(-1)*A_6dofinv')^(-1) * A_6dofinvDot) * test_6dofinv.jointVelocities;

        inertia_times_acceleration_6dof = M_6dof * test_6dof.jointAccelerations;
        right_side_6dof = P_6dof * tau_6dof - P_6dof * test_6dof.gravitationalTorqueMatrix - (P_6dof*test_6dof.coriolisMatrix + A_6dof' * (A_6dof*M_6dof^(-1)*A_6dof')^(-1) * A_6dofDot) * test_6dof.jointVelocities;
        error_6dof = norm(inertia_times_acceleration_6dof - right_side_6dof);
        
        inertia_times_acceleration_6dofinv = M_6dofinv * test_6dofinv.jointAccelerations;
        right_side_6dofinv = P_6dofinv * tau_6dofinv - P_6dofinv * test_6dofinv.gravitationalTorqueMatrix - (P_6dofinv*test_6dofinv.coriolisMatrix + A_6dofinv' * (A_6dofinv*M_6dofinv^(-1)*A_6dofinv')^(-1) * A_6dofinvDot) * test_6dofinv.jointVelocities;
        error_6dofinv = norm(inertia_times_acceleration_6dofinv - right_side_6dofinv);
        
%         test_3dof.inertiaMatrix*test_3dof.jointAccelerations ...
%             + P_3dof * test_3dof.coriolisMatrix*test_3dof.jointVelocities ...
%             + P_3dof * test_3dof.gravitationalTorqueMatrix ...
%             + A_3dof' * (A_3dof*M_3dof^(-1)*A_3dof')^(-1) * A_3dofDot * test_3dof.jointVelocities
%         P_3dof * test_3dof.externalTorques
%         test_3dof.externalTorques
% 
%         P_6dof = eye(6) - A_6dof' * (A_6dof*M_6dof^(-1)*A_6dof')^(-1) * A_6dof*M_6dof^(-1);
%         test_6dof.inertiaMatrix*test_6dof.jointAccelerations ...
%             + P_6dof * test_6dof.coriolisMatrix*test_6dof.jointVelocities ...
%             + P_6dof * test_6dof.gravitationalTorqueMatrix ...
%             + A_6dof' * (A_6dof*M_6dof^(-1)*A_6dof')^(-1) * A_6dofDot * test_6dof.jointVelocities
%         P_6dof * test_6dof.externalTorques
%         test_6dof.externalTorques
% 
%         P_6dofinv = eye(6) - A_6dofinv' * (A_6dofinv*M_6dofinv^(-1)*A_6dofinv')^(-1) * A_6dofinv*M_6dofinv^(-1);
%         test_6dofinv.inertiaMatrix*test_6dofinv.jointAccelerations ...
%             + P_6dofinv * test_6dofinv.coriolisMatrix*test_6dofinv.jointVelocities ...
%             + P_6dofinv * test_6dofinv.gravitationalTorqueMatrix ...
%             + A_6dofinv' * (A_6dofinv*M_6dofinv^(-1)*A_6dofinv')^(-1) * A_6dofinvDot * test_6dofinv.jointVelocities
%         P_6dofinv * test_6dofinv.externalTorques
%         test_6dofinv.externalTorques
%         
%         
%         timeseries_accelerations_induced_by_applied_6dof(i_time, :) = test_6dof.inertiaMatrix^(-1) * tau_6dof;
%         timeseries_accelerations_induced_by_gravity_6dof(i_time, :) = - test_6dof.inertiaMatrix^(-1) * test_6dof.gravitationalTorqueMatrix;
%         timeseries_accelerations_induced_by_coriolis_6dof(i_time, :) = - test_6dof.inertiaMatrix^(-1) * test_6dof.coriolisMatrix*test_6dof.jointVelocities;
%         timeseries_accelerations_induced_by_constraint_6dof(i_time, :) = - test_6dof.inertiaMatrix^(-1) * test_6dof.constraintTorques;
%         timeseries_accelerations_induced_by_applied_6dofinv(i_time, :) = test_6dofinv.inertiaMatrix^(-1) * tau_6dofinv;
%         timeseries_accelerations_induced_by_gravity_6dofinv(i_time, :) = - test_6dofinv.inertiaMatrix^(-1) * test_6dofinv.gravitationalTorqueMatrix;
%         timeseries_accelerations_induced_by_coriolis_6dofinv(i_time, :) = - test_6dofinv.inertiaMatrix^(-1) * test_6dofinv.coriolisMatrix*test_6dof.jointVelocities;
%         timeseries_accelerations_induced_by_constraint_6dofinv(i_time, :) = - test_6dofinv.inertiaMatrix^(-1) * test_6dofinv.constraintTorques;
    end
end

%% plot
joints_to_plot = 1 : 3;
joint_labels = {'ankle', 'knee', 'hip'};
if plot_angles
    for i_joint = joints_to_plot
        figure; axes; hold on; title(['separated torques, joint ' num2str(i_joint) ' - 3dof'])
        plot(timeseries_joint_angles_3dof(:, i_joint), 'r', 'linewidth', 2, 'displayname', 'angle')
        legend('toggle')

        figure; axes; hold on; title(['separated torques, joint ' num2str(i_joint) ' - 6dof'])
        plot(timeseries_joint_angles_6dof(:, i_joint), 'r', 'linewidth', 2, 'displayname', 'angle')
        legend('toggle')
        
        figure; axes; hold on; title(['separated torques, joint ' num2str(i_joint) ' - 6dofinv'])
        plot(timeseries_joint_angles_6dofinv(:, i_joint+3), 'r', 'linewidth', 2, 'displayname', 'angle')
        legend('toggle')
    end
end
if plot_torques
    for i_joint = joints_to_plot
        figure; axes; hold on; title(['separated torques, joint ' num2str(i_joint) ' - 3dof'])
        plot(timeseries_joint_torques_applied_3dof(:, i_joint), 'r', 'linewidth', 2, 'displayname', 'muscle')
        plot(timeseries_joint_torques_gravity_3dof(:, i_joint), 'g', 'linewidth', 2, 'displayname', 'gravity')
        plot(timeseries_joint_torques_coriolis_3dof(:, i_joint), 'c', 'linewidth', 2, 'displayname', 'velocity-dependent')
        plot(timeseries_joint_torques_acceleration_dependent_3dof(:, i_joint), 'm', 'linewidth', 2, 'displayname', 'acceleration-dependent')
        plot(timeseries_joint_torques_net_3dof(:, i_joint), 'k', 'linewidth', 2, 'displayname', 'net')
        legend('toggle')

        figure; axes; hold on; title(['separated torques, joint ' num2str(i_joint) ' - 6dof'])
        plot(timeseries_joint_torques_applied_6dof(:, i_joint+3), 'r', 'linewidth', 2, 'displayname', 'muscle')
        plot(timeseries_joint_torques_gravity_6dof(:, i_joint+3), 'g', 'linewidth', 2, 'displayname', 'gravity')
        plot(timeseries_joint_torques_constraint_6dof(:, i_joint+3), 'b', 'linewidth', 2, 'displayname', 'constraint')
        plot(timeseries_joint_torques_coriolis_6dof(:, i_joint+3), 'c', 'linewidth', 2, 'displayname', 'velocity-dependent')
        plot(timeseries_joint_torques_acceleration_dependent_6dof(:, i_joint+3), 'm', 'linewidth', 2, 'displayname', 'acceleration-dependent')
        plot(timeseries_joint_torques_net_6dof(:, i_joint+3), 'k', 'linewidth', 2, 'displayname', 'net')
        legend('toggle')
        
        figure; axes; hold on; title(['separated torques, joint ' num2str(i_joint) ' - 6dofinv'])
        plot(timeseries_joint_torques_applied_6dofinv(:, i_joint+3), 'r', 'linewidth', 2, 'displayname', 'muscle')
        plot(timeseries_joint_torques_gravity_6dofinv(:, i_joint+3), 'g', 'linewidth', 2, 'displayname', 'gravity')
        plot(timeseries_joint_torques_constraint_6dofinv(:, i_joint+3), 'b', 'linewidth', 2, 'displayname', 'constraint')
        plot(timeseries_joint_torques_coriolis_6dofinv(:, i_joint+3), 'c', 'linewidth', 2, 'displayname', 'velocity-dependent')
        plot(timeseries_joint_torques_acceleration_dependent_6dofinv(:, i_joint+3), 'm', 'linewidth', 2, 'displayname', 'acceleration-dependent')
        plot(timeseries_joint_torques_net_6dofinv(:, i_joint+3), 'k', 'linewidth', 2, 'displayname', 'net')
        legend('toggle')
    end

%     % check
%     figure; axes; hold on; title(['inverse dynamic check, joint ' num2str(i_joint)])
%     plot(timeseries_joint_torques_applied_3dof(:, i_joint), 'r', 'linewidth', 2, 'displayname', 'muscle')
%     plot(sinusoid_torque*external_torque_direction(i_joint), 'g--', 'linewidth', 2, 'displayname', 'muscle - real')
    
end

if plot_induced_accelerations
    for i_joint = joints_to_plot
        figure; axes; hold on; title(['induced accelerations, ' joint_labels(i_joint) ' joint - 3dof'])
        plot(timeseries_accelerations_induced_by_applied_3dof(:, i_joint), 'r', 'linewidth', 2, 'displayname', 'muscle')
        plot(timeseries_accelerations_induced_by_gravity_3dof(:, i_joint), 'g', 'linewidth', 2, 'displayname', 'gravity')
        plot(timeseries_accelerations_induced_by_movement_3dof(:, i_joint), 'c', 'linewidth', 2, 'displayname', 'velocity-dependent')
        
        plot(timeseries_joint_acceleration_3dof(:, i_joint), 'b', 'linewidth', 2, 'displayname', 'net acceleration')
        plot(timeseries_accelerations_induced_by_applied_3dof(:, i_joint) + timeseries_accelerations_induced_by_gravity_3dof(:, i_joint) + timeseries_accelerations_induced_by_movement_3dof(:, i_joint), 'm-.', 'linewidth', 2, 'displayname', 'sum')
        
        legend('toggle')

        figure; axes; hold on; title(['induced accelerations, ' joint_labels(i_joint) ' joint - 6dof'])
        plot(timeseries_accelerations_induced_by_applied_6dof(:, i_joint+3), 'r', 'linewidth', 2, 'displayname', 'muscle')
        plot(timeseries_accelerations_induced_by_gravity_6dof(:, i_joint+3), 'g', 'linewidth', 2, 'displayname', 'gravity')
        plot(timeseries_accelerations_induced_by_movement_6dof(:, i_joint+3), 'c', 'linewidth', 2, 'displayname', 'velocity-dependent')
%         plot(timeseries_accelerations_induced_by_constraint_6dof(:, i_joint+3), 'b', 'linewidth', 2, 'displayname', 'constraint')
        
        plot(timeseries_joint_acceleration_6dof(:, i_joint+3), 'b', 'linewidth', 2, 'displayname', 'net acceleration')
        plot(timeseries_accelerations_induced_by_applied_6dof(:, i_joint+3) + timeseries_accelerations_induced_by_gravity_6dof(:, i_joint+3) + timeseries_accelerations_induced_by_movement_6dof(:, i_joint+3), 'm-.', 'linewidth', 2, 'displayname', 'sum')
        legend('toggle')
        
        figure; axes; hold on; title(['induced accelerations, ' joint_labels(i_joint) ' joint - 6dofinv'])
        plot(timeseries_accelerations_induced_by_applied_6dofinv(:, 6 - i_joint + 1), 'r', 'linewidth', 2, 'displayname', 'muscle')
        plot(timeseries_accelerations_induced_by_gravity_6dofinv(:, 6 - i_joint + 1), 'g', 'linewidth', 2, 'displayname', 'gravity')
        plot(timeseries_accelerations_induced_by_movement_6dofinv(:,6 - i_joint + 1), 'c', 'linewidth', 2, 'displayname', 'velocity-dependent')
%         plot(timeseries_accelerations_induced_by_constraint_6dofinv(:, i_joint+3), 'b', 'linewidth', 2, 'displayname', 'constraint')
        
        plot(timeseries_joint_acceleration_6dofinv(:, 6 - i_joint + 1), 'b', 'linewidth', 2, 'displayname', 'net acceleration')
        plot(timeseries_accelerations_induced_by_applied_6dofinv(:, 6 - i_joint + 1) + timeseries_accelerations_induced_by_gravity_6dofinv(:, 6 - i_joint + 1) + timeseries_accelerations_induced_by_movement_6dofinv(:, 6 - i_joint + 1), 'm-.', 'linewidth', 2, 'displayname', 'sum')
        legend('toggle')
    end
    distFig('rows', 3)



end








