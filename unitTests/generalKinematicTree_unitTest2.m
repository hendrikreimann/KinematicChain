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

% 7 joints
% joint_positions = {[0; 0; 0]; [0; 1; 0]; [1; 1; 0]; [1; 1; 0]; [0; 1; 0]; [-1; 1; 0]; [-1; 1; 0]; };
% joint_axes = {[0; 0; 1], [0; 0; 1], [0; 0; 1], [0; 0; 1], [0; 0; 1], [0; 0; 1], [0; 0; 1]};
% for i_joint = 1 : 7
%     joint_axis = rand(3, 1);
%     joint_axes{i_joint} = joint_axis * 1 / norm(joint_axis);
% end
% joint_types = [1 1 1 1 1 1 1];
% end_effectors = {[1; 2; 0], [2; 1; 0], [-1; 2; 0], [-2; 1; 0]};
% link_positions = {[0; 0.5; 0]; [0.5; 1; 0]; [1; 1.5; 0]; [1.5; 1; 0]; [-0.5; 1; 0]; [-1; 1.5; 0]; [-1.5; 1; 0];};
% link_orientations = {eye(3), eye(3), eye(3), eye(3), eye(3), eye(3), eye(3)};
% branch_matrix = ...
%   [ ...
%     1 1 1 0 0 0 0; ...
%     1 1 0 1 0 0 0; ...
%     1 0 0 0 1 1 0; ...
%     1 0 0 0 1 0 1; ...
%   ];


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
test_hand.jointVelocities = ones(test_hand.numberOfJoints, 1);

% test_hand.jointAngles = 1.0*randn(test_hand.numberOfJoints, 1);
% test_hand.jointAngles = [0; pi/4; -pi/4; -pi/4; pi/4];
% test_hand.jointAngles = [0; pi/4; -pi/3];
% test_hand.jointVelocities = 1.5*randn(test_hand.numberOfJoints, 1);
test_hand.jointVelocities = 0.5*ones(test_hand.numberOfJoints, 1);
test_hand.jointVelocities(1) = 0;
test_hand.jointVelocities = randn(test_hand.numberOfJoints, 1);
test_hand.updateInternals;



% numerically calculate the spatialJacobianTemporalDerivatives
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
diff_hand.updateInternals;
delta_t = 0.00000001;
diff_hand.jointAngles = test_hand.jointAngles + delta_t * test_hand.jointVelocities;
diff_hand.updateInternals();
delta_spatialJacobian = diff_hand.spatialJacobian - test_hand.spatialJacobian;
delta_endE_effector_jacobian_one = diff_hand.endEffectorJacobians{1} - test_hand.endEffectorJacobians{1};
delta_endE_effector_jacobian_two = diff_hand.endEffectorJacobians{2} - test_hand.endEffectorJacobians{2};
theta_dot = test_hand.jointVelocities
spatialJacobianTemporalDerivative_numerical = delta_spatialJacobian * delta_t^(-1)
spatialJacobianTemporalDerivative_analytical = test_hand.spatialJacobianTemporalDerivative
endEffectorJacobianTemporalDerivative_one_numerical = delta_endE_effector_jacobian_one * delta_t^(-1)
endEffectorJacobianTemporalDerivative_one_analytical = test_hand.endEffectorJacobianTemporalDerivatives{1}
endEffectorJacobianTemporalDerivative_two_numerical = delta_endE_effector_jacobian_two * delta_t^(-1)
endEffectorJacobianTemporalDerivative_two_analytical = test_hand.endEffectorJacobianTemporalDerivatives{2}


return

sceneBound = 3*[-1; 1; -1; 1; -1; 1];
stickFigure = KinematicTreeStickFigure(test_hand, sceneBound);
set(gca, 'xlim', [stickFigure.sceneBound(1), stickFigure.sceneBound(2)], ...
         'ylim', [stickFigure.sceneBound(3), stickFigure.sceneBound(4)], ...
         'zlim',[stickFigure.sceneBound(5), stickFigure.sceneBound(6)]);
view([0, -1, 0]);
% stickFigure.showLinkMassEllipsoids = true;
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
    
    
    
    
    
    
    
    
