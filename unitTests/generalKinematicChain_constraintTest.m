

joint_positions = {[0; 0; 0], [1; 0; 0], [2; 0; 0]};
joint_axes = {[0; 0; 1], [0; 0; 1], [0; 0; 1]};
joint_types = [1 1 1];
end_effector = [3; 0; 0];
link_positions = {[0.5; 0; 0], [1.5; 0; 0], [2.5; 0; 0]};
link_masses = [5, 5, 5];
link_moments_of_inertia = [1 1 1; 1 1 1; 1 1 1];

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

arm.jointAngles = [-1; 1; 1] * pi/2;
arm.jointVelocities = [-1; 1; 1] * -1;
arm.jointAccelerations = [0; 0; 0];
arm.updateInternals;

wall_position = arm.endEffectorPosition(1);

% add visualization data for the acceleration induced by the external wrench
arm.miscellaneousLinesStartPoints = zeros(3, 4);
arm.miscellaneousLinesEndPoints = zeros(3, 4);

stickFigure = KinematicChainStickFigure(arm, 4*[-1, 1, -1, 1, -1, 1]);
stickFigure.setMiscellaneousPlotColor(2, [1 0 1]);
stickFigure.setMiscellaneousPlotColor(3, [0 0 0]);
arm.miscellaneousLinesStartPoints(:, 3) = [wall_position; -5; 0];
arm.miscellaneousLinesEndPoints(:, 3) = [wall_position; 5; 0];


external_torques = [0; 0; 0];



timeStep = 0.001;
counter = 1;
while true
    % calculate constraint forces
    M = arm.inertiaMatrix;
    A = arm.endEffectorJacobian(1, :);
    lambda = (A*M*A')^(-1) * (A*M^(-1)*(
    
    
    
    % apply torques and make euler step
    arm.externalTorques = torques_from_eef_wrench;
    arm.calculateAccelerationsFromExternalTorques;
    arm.jointVelocities = arm.jointVelocities + timeStep*arm.jointAccelerations;
    arm.jointAngles = arm.jointAngles + timeStep*arm.jointVelocities + timeStep^2*arm.jointAccelerations;
    arm.updateInternals;
    
    % update eef velocity/acceleration visualization
    joint_acceleration_from_eef_wrench = arm.inertiaMatrix^(-1) * torques_from_eef_wrench;
    eef_acceleration_from_eef_wrench = arm.endEffectorJacobian * joint_acceleration_from_eef_wrench;
    arm.miscellaneousLinesStartPoints(:, 1) = arm.endEffectorPosition;
    arm.miscellaneousLinesEndPoints(:, 1) = arm.endEffectorPosition + arm.endEffectorVelocity;
    arm.miscellaneousLinesStartPoints(:, 2) = arm.endEffectorPosition + arm.endEffectorVelocity;
    arm.miscellaneousLinesEndPoints(:, 2) = arm.endEffectorPosition + arm.endEffectorVelocity + arm.endEffectorAcceleration;
    arm.miscellaneousLinesStartPoints(:, 3) = arm.endEffectorPosition + arm.endEffectorVelocity;
    arm.miscellaneousLinesEndPoints(:, 3) = arm.endEffectorPosition + arm.endEffectorVelocity + eef_acceleration_from_eef_wrench;
    
    
    counter = counter+1;
    if counter == 10
        counter = 0;
        stickFigure.update();
        drawnow;
    end
end    
    
    
    
    
    
    
    
    
