
arm_template = planarHumanArmTwoDof(80, 1.8);

joint_positions = arm_template.jointPositions;
joint_axes = {[0; 0; 1], [0; 0; 1]};
joint_types = [1 1];
end_effector = arm_template.endEffectorPosition;
link_positions = {[arm_template.linkComDistancesFromJoint(1); 0; 0]; ...
                  [arm_template.linkLengths(1) + arm_template.linkComDistancesFromJoint(2); 0; 0]};
link_masses = arm_template.linkMasses;
link_moments_of_inertia = arm_template.linkMomentsOfInertia;

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

arm.jointAngles = [-pi/4; pi/2];
arm.jointVelocities = [0; 0];
arm.jointAccelerations = [0; 0];
arm.updateInternals;

stickFigure = KinematicChainStickFigure(arm, [-1, 1, -1, 1, -1, 1]);
stickFigure.update();

external_wrench = [-5; 0; 0; 0; 0; 0];

timeStep = 0.001;
counter = 1;
while true
    % transform the external wrench into end-effector coordinates
    R_world_eef = arm.endEffectorTransformation(1:3, 1:3);
    T_eefRotation = [R_world_eef zeros(3,1); 0 0 0 1];
    A_eefRotation = createAdjointTransformation(T_eefRotation);
    eef_wrench = A_eefRotation' * external_wrench;
    
    % transform the wrench into joint torques
    eef_transformation_adjoint = createAdjointTransformation(arm.endEffectorTransformation);
    body_jacobian = invertAdjoint(eef_transformation_adjoint) * arm.spatialJacobian;
    torques_from_eef_wrench = body_jacobian' * eef_wrench;
    
    % apply torques and make euler step
    arm.externalTorques = torques_from_eef_wrench;
    arm.calculateAccelerationsFromExternalTorques;
    arm.jointVelocities = arm.jointVelocities + timeStep*arm.jointAccelerations;
    arm.jointAngles = arm.jointAngles + timeStep*arm.jointVelocities + timeStep^2*arm.jointAccelerations;
    arm.updateInternals;
    
    counter = counter+1;
    if counter == 10
        counter = 0;
        stickFigure.update();
        drawnow;
    end
end    
    
    
    
    
    
    
    
    
