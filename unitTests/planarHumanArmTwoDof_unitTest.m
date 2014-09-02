% script to test the planarHumanArmTwoDof

arm = planarHumanArmTwoDof(80, 1.8);
% disp('joint angles:')
% disp(num2str(arm.jointAngles))
arm.jointAngles(1) = pi/4;
arm.jointAngles(2) = -pi/4;
arm.jointVelocities(1) = 1;
arm.jointVelocities(2) = -1;
% arm.jointAngles(1) = 0;
% disp('joint angles:')
% disp(num2str(arm.jointAngles))

arm.updateInternals();
% disp('end-effector position:')
% disp(num2str(arm.endEffectorPosition))


% disp('M:')
% disp(num2str(arm.inertiaMatrix))
% disp('C:')
% disp(num2str(arm.coriolisMatrix))


stickFigure = KinematicChainStickFigure(arm);
stickFigure.update();


% check velocity and acceleration by numerically integrating
arm.jointAngles = randn(2, 1);
arm.jointVelocities = randn(2, 1);
arm.jointAccelerations = randn(2, 1);
% arm.jointAngles = [pi/2; 0];
% arm.jointVelocities = [1; 0];
% arm.jointAccelerations = [0; 0];
arm.updateInternals();
p = arm.endEffectorPosition;
v_ana = arm.endEffectorVelocity;
a_ana = arm.endEffectorAcceleration;
J = arm.endEffectorJacobian;

stickFigure.update();

% disp('arm.jointVelocities:')
% disp(num2str(arm.jointVelocities))
% disp('v_ana:')
% disp(num2str(v_ana))
% disp('a_ana')
% disp(num2str(a_ana))
% disp('J:')
% disp(num2str(arm.endEffectorJacobian))
disp('J_dot:')
disp(num2str(arm.endEffectorJacobianTemporalDerivative))


delta_t = 0.000000001;
theta = arm.jointAngles;
theta_dot = arm.jointVelocities;
theta_two_dot = arm.jointAccelerations;
theta_dot_new = theta_dot + delta_t*theta_two_dot;
theta_new = theta + delta_t*theta_dot + delta_t^2*theta_two_dot;

arm.jointAngles = theta_new;
arm.jointVelocities = theta_dot_new;
arm.updateInternals;

p_new = arm.endEffectorPosition;
v_num = (p_new - p) / delta_t;
v_new = arm.endEffectorVelocity;
a_num = (v_new - v_ana) / delta_t;

J_new = arm.endEffectorJacobian;
J_dot_num = (J_new - J) / delta_t;


% disp('v_num:')
% disp(num2str(v_num))
% disp('a_num:')
% disp(num2str(a_num))
disp('J_dot_num:')
disp(num2str(J_dot_num))

arm.jointAngles = [0; 0];
arm.jointVelocities = [2; 2];
arm.jointAccelerations = [0; 0];

return

timeStep = 0.001;
counter = 1;
while true
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






















