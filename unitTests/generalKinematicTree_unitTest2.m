% 3 joints
% joint_positions = {[0; 0; 0]; [1; 0; 0]; [-1; 0; 0]};
% joint_axes = {[0; 0; 1], [0; 0; 1], [0; 0; 1]};
% joint_types = [1 1 1];
% end_effectors = {[3; 0; 0], [-3; 0; 0]};
% link_positions = {[0; 0; 0]; [1.5; 0; 0]; [-1.5; 0; 0];};
% branch_matrix = [1 1 0; 1 0 1]; % each row is a branch, listing the joints that move the end-effector of that branch

% 5 joints
% joint_positions = {[0; 0; 0]; [0.5; 0; 0]; [0.5; -1; 0]; [-0.5; 0; 0]; [-0.5; -1; 0]; };
% joint_axes = {[0; 0; 1], [0; 0; 1], [0; 0; 1], [0; 0; 1], [0; 0; 1]};
% joint_types = [1 1 1 1 1];
% end_effectors = {[0.5; -2; 0], [-0.5; -2; 0]};
% link_positions = {[0; 0; 0]; [0.5; -0.5; 0]; [0.5; -1.5; 0]; [-0.5; -0.5; 0]; [-0.5; -1.5; 0];};
% branch_matrix = [1 1 1 0 0; 1 0 0 1 1]; % each row is a branch, listing the joints that move the end-effector of that branch

% 5 joints, vertical
joint_positions = {[0; 0; 0]; [0.5; 0; 0]; [0.5; 0; -1]; [-0.5; 0; 0]; [-0.5; 0; -1]; };
joint_axes = {[0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0]};
joint_types = [1 1 1 1 1];
end_effectors = {[0.5; 0; -2], [-0.5; 0; -2]};
link_positions = {[0; 0; 0]; [0.5; 0; -0.5]; [0.5; 0; -1.5]; [-0.5; 0; -0.5]; [-0.5; 0; -1.5];};
branch_matrix = [1 1 1 0 0; 1 0 0 1 1]; % each row is a branch, listing the joints that move the end-effector of that branch
link_orientations = {eye(3), eye(3), eye(3), eye(3), eye(3)};

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
joint_positions = {[0; 0; 0]; [0.5; 0; 0]; [0.5; 0; -1]; [0.5; 0; -1.5]; [-0.5; 0; 0]; [-0.5; 0; -1]; [-0.5; 0; -1.5]; };
joint_axes = {[0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0], [0; 1; 0]};
joint_types = [1 1 1 1 1 1 1];
end_effectors = {[0.5; 0; -2], [-0.5; 0; -2]};
link_positions = {[0; 0; 0]; [0.5; 0; -0.5]; [0.5; 0; -1.25]; [0.5; 0; -1.75]; [-0.5; 0; -0.5]; [-0.5; 0; -1.25]; [-0.5; 0; -1.75];};
link_orientations = {eye(3), eye(3), eye(3), eye(3), eye(3), eye(3), eye(3)};
branch_matrix = ...
  [ ...
    1 1 1 1 0 0 0; ...
    1 0 0 0 1 1 1; ...
  ];

reset(RandStream.getGlobalStream);

% randomize joint axes
% for i_joint = 1 : length(joint_axes)
%     joint_axis = rand(3, 1);
%     joint_axes{i_joint} = joint_axis * 1 / norm(joint_axis);
% end


degrees_of_freedom = length(joint_axes);
link_masses = ones(degrees_of_freedom, 1);
% link_moments_of_inertia = ones(degrees_of_freedom, 3) * 0.05;
link_moments_of_inertia = [[1 5 5] ; repmat([1 1 .5], degrees_of_freedom-1, 1)] * 0.01;

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



% test_hand.jointAngles = -.3*randn(degrees_of_freedom, 1);
% test_hand.jointAngles = 2*ones(degrees_of_freedom, 1);

% test_hand.jointVelocities = [0.2; 0.5; 0.5; -0.5; -0.5];
% test_hand.jointVelocities = [0.2; 0.1; -0.3; -0.6; -0.5; 1.0; -1.0];
% test_hand.jointVelocities = ones(test_hand.numberOfJoints, 1);

% test_hand.jointAngles = 1.0*randn(test_hand.numberOfJoints, 1);
% test_hand.jointAngles = [0; -3*pi/4; pi/4; pi/4; -pi/4];
% test_hand.jointAngles = [0; -pi/4; pi/4; pi/4; -pi/4];
% test_hand.jointAngles = [0; pi/4; -pi/3];
% test_hand.jointVelocities = 1.0*randn(test_hand.numberOfJoints, 1);
% test_hand.jointVelocities = 0.1*ones(test_hand.numberOfJoints, 1);
% test_hand.jointVelocities = ones(test_hand.numberOfJoints, 1);
% test_hand.jointVelocities([4 7]) = 1;
test_hand.jointAngles(2) = -pi/4;
test_hand.jointAngles(5) = pi/4;
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

% numerically calculate the spatialJacobianTemporalDerivatives
% diff_hand.updateInternals;
% delta_t = 0.00000001;
% diff_hand.jointAngles = test_hand.jointAngles + delta_t * test_hand.jointVelocities;
% diff_hand.updateInternals();
% delta_spatialJacobian = diff_hand.spatialJacobian - test_hand.spatialJacobian;
% delta_endE_effector_jacobian_one = diff_hand.endEffectorJacobians{1} - test_hand.endEffectorJacobians{1};
% delta_endE_effector_jacobian_two = diff_hand.endEffectorJacobians{2} - test_hand.endEffectorJacobians{2};
% theta_dot = test_hand.jointVelocities
% spatialJacobianTemporalDerivative_numerical = delta_spatialJacobian * delta_t^(-1)
% spatialJacobianTemporalDerivative_analytical = test_hand.spatialJacobianTemporalDerivative
% endEffectorJacobianTemporalDerivative_one_numerical = delta_endE_effector_jacobian_one * delta_t^(-1)
% endEffectorJacobianTemporalDerivative_one_analytical = test_hand.endEffectorJacobianTemporalDerivatives{1}
% endEffectorJacobianTemporalDerivative_two_numerical = delta_endE_effector_jacobian_two * delta_t^(-1)
% endEffectorJacobianTemporalDerivative_two_analytical = test_hand.endEffectorJacobianTemporalDerivatives{2}

% return

% numerically calculate the bodyJacobianTemporalDerivatives
% delta_t = 0.0000000001;
% diff_hand.jointAngles = test_hand.jointAngles + delta_t * test_hand.jointVelocities;
% diff_hand.updateInternals();
% delta_body_jacobian_one = diff_hand.bodyJacobians{1} - test_hand.bodyJacobians{1};
% bodyJacobianTemporalDerivative_one_numerical = delta_body_jacobian_one * delta_t^(-1)
% bodyJacobianTemporalDerivative_one_analytical = test_hand.bodyJacobianTemporalDerivatives{1}
% delta_body_jacobian_two = diff_hand.bodyJacobians{2} - test_hand.bodyJacobians{2};
% bodyJacobianTemporalDerivative_two_numerical = delta_body_jacobian_two * delta_t^(-1)
% bodyJacobianTemporalDerivative_two_analytical = test_hand.bodyJacobianTemporalDerivatives{2}
% 
% delta_eef_trafo_one = diff_hand.endEffectorTransformations{1} - test_hand.endEffectorTransformations{1};
% delta_eef_trafo_adjoint_one = createAdjointTransformation(diff_hand.endEffectorTransformations{1}) - createAdjointTransformation(test_hand.endEffectorTransformations{1});
% eef_trafo_dot_numerical = delta_eef_trafo_one * delta_t^(-1);
% eef_trafo_adjoint_dot_numerical = delta_eef_trafo_adjoint_one * delta_t^(-1);
% 
% % test the equations step by step
% J_s = test_hand.spatialJacobian;
% J_s_dot = test_hand.spatialJacobianTemporalDerivative;
% for i_joint = 1 : test_hand.numberOfJoints
%     J_s(:, i_joint) = J_s(:, i_joint) * test_hand.branchMatrix(1, i_joint);
%     J_s_dot(:, i_joint) = J_s_dot(:, i_joint) * test_hand.branchMatrix(1, i_joint);
% end
% J_b = test_hand.bodyJacobians{1};
% eef_trafo = test_hand.endEffectorTransformations{1};
% A_eef_trafo = createAdjointTransformation(eef_trafo);
% body_velocity = test_hand.bodyJacobians{1} * test_hand.jointVelocities;
% eef_trafo_dot_analytical = eef_trafo * twistCoordinatesToMatrix(body_velocity);
% R_eef = eef_trafo(1:3, 1:3);
% p_eef = eef_trafo(1:3, 4);
% R_eef_dot = eef_trafo_dot_analytical(1:3, 1:3);
% p_eef_dot = eef_trafo_dot_analytical(1:3, 4);
% p_eef_skew = skewVectorToMatrix(p_eef);
% p_eef_dot_skew = skewVectorToMatrix(p_eef_dot);
% eef_trafo_adjoint_dot_analytical = [R_eef_dot p_eef_dot_skew*R_eef+p_eef_skew*R_eef_dot; zeros(3) R_eef_dot];

% J_s_dot
% eef_trafo_adjoint_dot_numerical*J_b + createAdjointTransformation(test_hand.endEffectorTransformations{1})*bodyJacobianTemporalDerivative_one_numerical

% bodyJacobianTemporalDerivative_one_numerical
% createAdjointTransformation(test_hand.endEffectorTransformations{1})^(-1)*(J_s_dot - eef_trafo_adjoint_dot_numerical*J_b)

% bodyJacobianTemporalDerivative_one_numerical
% createAdjointTransformation(test_hand.endEffectorTransformations{1})^(-1)*J_s_dot - createAdjointTransformation(test_hand.endEffectorTransformations{1})^(-1)*eef_trafo_adjoint_dot_numerical*J_b

% bodyJacobianTemporalDerivative_one_numerical
% createAdjointTransformation(test_hand.endEffectorTransformations{1}^(-1))*J_s_dot - createAdjointTransformation(test_hand.endEffectorTransformations{1}^(-1))*eef_trafo_adjoint_dot_numerical*J_b

% eef_trafo_adjoint_dot_numerical
% createAdjointTransformation(eef_trafo_dot_numerical)

% eef_trafo_dot_numerical
% eef_trafo_dot_analytical

% eef_trafo_adjoint_dot_numerical
% eef_trafo_adjoint_dot_analytical
% return

% check end-effector Jacobians
% diff_hand.updateInternals;
% delta_theta = 0.00000001;
% diff_hand.updateInternals();
% end_effector_jacobian_one_numerical = zeros(3, test_hand.numberOfJoints);
% end_effector_jacobian_two_numerical = zeros(3, test_hand.numberOfJoints);
% for i_joint = 1 : diff_hand.numberOfJoints
%     diff_hand.jointAngles = test_hand.jointAngles;
%     diff_hand.jointAngles(i_joint) = test_hand.jointAngles(i_joint) + delta_theta;
%     diff_hand.updateInternals();
%     end_effector_jacobian_one_numerical(:, i_joint) = (diff_hand.endEffectorTransformations{1}(1:3, 4) - test_hand.endEffectorTransformations{1}(1:3, 4)) / delta_theta;
%     end_effector_jacobian_two_numerical(:, i_joint) = (diff_hand.endEffectorTransformations{2}(1:3, 4) - test_hand.endEffectorTransformations{2}(1:3, 4)) / delta_theta;
% end
% end_effector_jacobian_one_numerical
% end_effector_jacobian_one_analytical = test_hand.endEffectorJacobians{1}
% end_effector_jacobian_two_numerical
% end_effector_jacobian_two_analytical = test_hand.endEffectorJacobians{2}

% % check arbitrary point Jacobian
% delta_theta = 0.00000001;
% % arbitrary_point = [0; 0; 0; 1];
% arbitrary_point = [rand(3, 1); 0];
% arbitrary_point = [1; 0; 0; 0];
% arbitrary_point_attachment_joint = 5;
% arbitrary_point_jacobian_numerical = zeros(3, test_hand.numberOfJoints);
% arbitrary_point_coordinate_frame = 'local';
% % arbitrary_point_coordinate_frame = 'world';
% 
% if strcmp(arbitrary_point_coordinate_frame, 'local')
%     arbitrary_point_position_local = arbitrary_point;
%     arbitrary_point_position_world = test_hand.jointTransformations{arbitrary_point_attachment_joint} * arbitrary_point;
% elseif strcmp(arbitrary_point_coordinate_frame, 'world')
%     arbitrary_point_position_local = test_hand.jointTransformations{arbitrary_point_attachment_joint}^(-1) * arbitrary_point;
%     arbitrary_point_position_world = arbitrary_point;
% end
% for i_joint = 1 : diff_hand.numberOfJoints
%     diff_hand.jointAngles = test_hand.jointAngles;
%     diff_hand.jointAngles(i_joint) = test_hand.jointAngles(i_joint) + delta_theta;
%     diff_hand.updateInternals();
%     
%     arbitrary_point_position_world_changed = diff_hand.jointTransformations{arbitrary_point_attachment_joint} * arbitrary_point_position_local;
%     delta_arbitrary_point_position_world = arbitrary_point_position_world_changed - arbitrary_point_position_world;
%     
%     arbitrary_point_jacobian_numerical(:, i_joint) = delta_arbitrary_point_position_world(1:3) * delta_theta^(-1);
% end
% arbitrary_point_jacobian_numerical
% end_effector_jacobian_one_analytical = test_hand.calculateArbitraryPointJacobian(arbitrary_point, arbitrary_point_attachment_joint, arbitrary_point_coordinate_frame)

% check euler-angle Jacobians
% delta_theta = 0.00000001;
% diff_hand.updateInternals();
% end_effector_euler_jacobian_numerical = zeros(3, test_hand.numberOfJoints);
% euler_angles = eulerAnglesZXY(test_hand.endEffectorTransformations{1}(1:3, 1:3));
% for i_joint = 1 : diff_hand.numberOfJoints
%     diff_hand.jointAngles = test_hand.jointAngles;
%     diff_hand.jointAngles(i_joint) = test_hand.jointAngles(i_joint) + delta_theta;
%     diff_hand.updateInternals();
%     euler_angles_changed = eulerAnglesZXY(diff_hand.endEffectorTransformations{1}(1:3, 1:3));
%     delta_euler_angles = euler_angles_changed - euler_angles;
%     end_effector_euler_jacobian_numerical(:, i_joint) = delta_euler_angles' / delta_theta;
% end
% end_effector_euler_jacobian_numerical
% % end_effector_jacobian_one_analytical = test_hand.endEffectorJacobians{1}
% 
% J_b = test_hand.bodyJacobians{1};
% J_b_orientation_part = J_b(4:6, :)
% nullspace_euler_jacobian = null(end_effector_euler_jacobian_numerical)
% nullspace_body_jacobian = null(J_b_orientation_part)
% subspace(nullspace_euler_jacobian, nullspace_body_jacobian)
% 
% return

sceneBound = 4*[-1; 1; -1; 1; -1; 1];
stickFigure = KinematicTreeStickFigure(test_hand, sceneBound);
set(gca, 'xlim', [stickFigure.sceneBound(1), stickFigure.sceneBound(2)], ...
         'ylim', [stickFigure.sceneBound(3), stickFigure.sceneBound(4)], ...
         'zlim',[stickFigure.sceneBound(5), stickFigure.sceneBound(6)]);
view([0, -1, 0]);
stickFigure.showLinkMassEllipsoids = true;
% view(-130, 20);
stickFigure.update();

% return

% numerically calculate the inertia matrix partial derivatives
% diff_hand = GeneralKinematicTree ...
% ( ...
%   joint_positions, ...
%   joint_axes, ...
%   branch_matrix, ...
%   end_effectors, ...
%   link_positions, ...
%   link_masses, ...
%   link_moments_of_inertia ...
% );
% diff_hand.updateInternals;
% delta_theta_k = 0.00000001;
% inertiaMatrixPartialDerivatives_numerical = cell(diff_hand.numberOfJoints, diff_hand.numberOfJoints, diff_hand.numberOfJoints);
% inertiaMatrixPartialDerivativesByTheta_k_analytical = cell(diff_hand.numberOfJoints, 1);
% inertiaMatrixPartialDerivativesByTheta_k_numerical = cell(diff_hand.numberOfJoints, 1);
% for i_joint = 1 : diff_hand.numberOfJoints
%     for j_joint = 1 : diff_hand.numberOfJoints
%         for k_joint = 1 : diff_hand.numberOfJoints
%             diff_hand.jointAngles = test_hand.jointAngles;
%             diff_hand.jointAngles(k_joint) = diff_hand.jointAngles(k_joint) + delta_theta_k;
%             diff_hand.updateInternals();
%             delta_M_ij = diff_hand.inertiaMatrix(i_joint, j_joint) - test_hand.inertiaMatrix(i_joint, j_joint);
%             inertiaMatrixPartialDerivatives_numerical{i_joint, j_joint, k_joint} = delta_M_ij / delta_theta_k;
%             
%             % comparison matrices
%             inertiaMatrixPartialDerivativesByTheta_k_analytical{k_joint}(i_joint, j_joint) = test_hand.inertiaMatrixPartialDerivatives{i_joint, j_joint, k_joint};
%             inertiaMatrixPartialDerivativesByTheta_k_numerical{k_joint}(i_joint, j_joint) = inertiaMatrixPartialDerivatives_numerical{i_joint, j_joint, k_joint};
%         end
%     end
% end





timeStep = 0.01;
counter = 1;
while true
%     % apply constraints
    A = test_hand.endEffectorJacobians{1}([1 3], :);
    ADot = test_hand.endEffectorJacobianTemporalDerivatives{1}([1 3], :);
%     A = test_hand.endEffectorJacobians{1}(1, :);
%     ADot = test_hand.endEffectorJacobianTemporalDerivatives{1}(1, :);


% J_b = test_hand.bodyJacobians{1};
% J_b_orientation_part = J_b(4:6, :)
    
    M = test_hand.inertiaMatrix;
    lambda = (A*M^(-1)*A')^(-1) ...
        * (A*M^(-1)*(test_hand.externalTorques - test_hand.coriolisMatrix*test_hand.jointVelocities - test_hand.gravitationalTorqueMatrix) + ADot*test_hand.jointVelocities);
%     test_hand.constraintTorques = A'*lambda;
    constraint_torques = A'*lambda;
    
    
    
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
    
    
    
    
    
    
    
    
