
arm_check = planarHumanArmTwoDof(80, 1.8);

joint_positions = arm_check.jointPositions;
joint_axes = {[0; 0; 1], [0; 0; 1]};
joint_types = [1 1];
end_effector = arm_check.endEffectorPosition;
link_positions = {[arm_check.linkComDistancesFromJoint(1); 0; 0]; ...
                  [arm_check.linkLengths(1) + arm_check.linkComDistancesFromJoint(2); 0; 0]};
link_masses = arm_check.linkMasses;
link_moments_of_inertia = arm_check.linkMomentsOfInertia;


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

% add eef-velocity visualization data
arm.miscellaneousLinesStartPoints = zeros(3, 2);
arm.miscellaneousLinesEndPoints = zeros(3, 2);

theta = randn(2, 1);
theta_dot = randn(2, 1);
theta_two_dot = randn(2, 1);
arm_check.jointAngles = theta;
arm_check.jointVelocities = theta_dot;
arm_check.jointAccelerations = theta_two_dot;
arm_check.updateInternals;
arm.jointAngles = theta;
arm.jointVelocities = theta_dot;
arm.jointAccelerations = theta_two_dot;
arm.updateInternals;

% disp('arm_check.endEffectorJacobian')
% disp(num2str(arm_check.endEffectorJacobian))
% disp('arm.endEffectorJacobian')
% disp(num2str(arm.endEffectorJacobian))
% 
% disp('arm_check.endEffectorJacobianTemporalDerivative')
% disp(num2str(arm_check.endEffectorJacobianTemporalDerivative))
% disp('arm.endEffectorJacobianTemporalDerivative')
% disp(num2str(arm.endEffectorJacobianTemporalDerivative))
%
% disp('arm_check.endEffectorPosition')
% disp(num2str(arm_check.endEffectorPosition))
% disp('arm.endEffectorPosition')
% disp(num2str(arm.endEffectorPosition))
% 
% disp('arm_check.endEffectorVelocity')
% disp(num2str(arm_check.endEffectorVelocity))
% disp('arm.endEffectorVelocity')
% disp(num2str(arm.endEffectorVelocity))
% 
% disp('arm_check.endEffectorAcceleration')
% disp(num2str(arm_check.endEffectorAcceleration))
% disp('arm.endEffectorAcceleration')
% disp(num2str(arm.endEffectorAcceleration))
% 
% disp('arm_check.inertiaMatrix')
% disp(num2str(arm_check.inertiaMatrix))
% disp('arm.inertiaMatrix')
% disp(num2str(arm.inertiaMatrix))
% 
% disp('arm_check.coriolisMatrix')
% disp(num2str(arm_check.coriolisMatrix))
% disp('arm.coriolisMatrix')
% disp(num2str(arm.coriolisMatrix))
% 
% disp('--------------------------------')
return

stickFigure = KinematicChainStickFigure(arm);
stickFigure.update();
stickFigure.setMiscellaneousPlotColor(2, [1 0 1]);

arm.jointAngles = [0; 0];
arm.jointVelocities = [0.3; 0.5];
arm.jointAccelerations = [0; 0];


timeStep = 0.002;
counter = 1;
while true
    arm.calculateAccelerationsFromExternalTorques;
    arm.jointVelocities = arm.jointVelocities + timeStep*arm.jointAccelerations;
    arm.jointAngles = arm.jointAngles + timeStep*arm.jointVelocities + timeStep^2*arm.jointAccelerations;
    arm.updateInternals;
    
    % update eef velocity/acceleration visualization
    arm.miscellaneousLinesStartPoints(:, 1) = arm.endEffectorPosition;
    arm.miscellaneousLinesEndPoints(:, 1) = arm.endEffectorPosition + arm.endEffectorVelocity;
    arm.miscellaneousLinesStartPoints(:, 2) = arm.endEffectorPosition + arm.endEffectorVelocity;
    arm.miscellaneousLinesEndPoints(:, 2) = arm.endEffectorPosition + arm.endEffectorVelocity + arm.endEffectorAcceleration;
    
    counter = counter+1;
    if counter == 10
        counter = 0;
        stickFigure.update();
        drawnow;
    end
end    
    
    
    
    
    
    
    
    
