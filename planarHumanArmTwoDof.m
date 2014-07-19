classdef planarHumanArmTwoDof < KinematicChain
    properties
        mass;
        height;
        linkMomentsOfInertia;
        linkComDistancesFromJoint;
        linkLengths;
    end
    methods
        function obj = planarHumanArmTwoDof(mass, height)
            obj = obj@KinematicChain(2);
            obj.mass = mass;
            obj.height = height;
            
            l1 = 0.186*height;  l2 = 0.146*height;  l3 = 0.108*height;  % segment length
            m1 = 0.028*mass;    m2 = 0.016*mass;    m3 = 0.006*mass;    % segment mass
            c1 = 0.436*l1;      c2 = 0.430*l2;      c3 = 0.506*l3;      % segment center of mass distance from proximal joint
            r1 = 0.322*l1;      r2 = 0.303*l2;      r3 = 0.297*l3;      % segment radius of gyration from center of mass
            I1 = m1*r1^2;       I2 = m2*r2^2;       I3 = m3*r3^2;       % segment moments of inertia around z-axis
            
            
            l23 = 0.254*height;   m23 = 0.022*mass; c23 = 0.682*l2;   r23=.468*l2; I23 = m23*r23^2; % values for the combined forearm and hand
            
            % transform moments of inertia of lower arm and shank into
            % common values around a common CoM
%             c23 = 1/(m2+m3) * (m2*c2 + m3*(l2+c3)); % common center of mass
%             I2_23 = I2 + m2*(c23-c2)^2;
%             I3_23 = I3 + m3*(c23-c3-l2)^2;
%             I23 = I2_23 + I3_23;
            
            obj.linkComDistancesFromJoint = [c1, c23];
            obj.linkMasses = [m1, m23];
            obj.linkLengths = [l1, l23];
            obj.linkMomentsOfInertia = ones(obj.numberOfJoints, 3);
            obj.linkMomentsOfInertia(1, 2) = I1;
            obj.linkMomentsOfInertia(2, 2) = I23;
            
            obj.jointPositions{1} = [0; 0; 0];
            obj.updateInternals;
        end
        function updateInternals(obj)
            % extract variables for convenience
            m1 = obj.linkMasses(1);
            m2 = obj.linkMasses(2);
            I1 = obj.linkMomentsOfInertia(1);
            I2 = obj.linkMomentsOfInertia(2);
            r1 = obj.linkComDistancesFromJoint(1);
            r2 = obj.linkComDistancesFromJoint(2);
            l1 = obj.linkLengths(1);
            l2 = obj.linkLengths(2);

            c1 = cos(obj.jointAngles(1));
            c2 = cos(obj.jointAngles(2));
            c12 = cos(obj.jointAngles(1) + obj.jointAngles(2));
            s1 = sin(obj.jointAngles(1));
            s2 = sin(obj.jointAngles(2));
            s12 = sin(obj.jointAngles(1) + obj.jointAngles(2));
            td1 = obj.jointVelocities(1);
            td2 = obj.jointVelocities(2);
            t2d1 = obj.jointAccelerations(1);
            t2d2 = obj.jointAccelerations(2);
            tau1 = td1;
            tau2 = td1+td2;
            rho1 = t2d1;
            rho2 = t2d1+t2d2;


            % inertia matrix
            M11 = I1 + I2 + m1*r1^2 + m2*(l1^2 + 2*l1*r2*c2 + r2^2);
            M12 = I2 + m2*(r2^2 + l1*r2*c2);
            M22 = I2 + m2*r2^2;
            obj.inertiaMatrix = [M11 M12; M12 M22];

            % Coriolis matrix
            C11 = -td2*m2*l1*r2*s2;
            C12 = -(td2 + td1) * m2*l1*r2*s2;
            C21 = td1*(l1*m2*r2*s2);
            C22 = 0;
            obj.coriolisMatrix = [C11 C12; C21 C22];

            % gravitational torques matrix
            obj.gravitationalTorqueMatrix = [0; 0];

            % end-effector Jacobian
            J11 = -l2*s12 - l1*s1;
            J12 = -l2*s12;
            J21 = l2*c12 + l1*c1;
            J22 = l2*c12;
            J31 = 0;
            J32 = 0;
            obj.endEffectorJacobian = [J11 J12; J21 J22; J31 J32];
            
            J11_dot = - l1*c1*tau1 - l2*c12*tau2;
            J12_dot = - l2*c12*tau2;
            J21_dot = - l1*s1*tau1 - l2*s12*tau2;
            J22_dot = - l2*s12*tau2;
            
            
            obj.endEffectorJacobianTemporalDerivative(1, 1) = J11_dot;
            obj.endEffectorJacobianTemporalDerivative(1, 2) = J12_dot;
            obj.endEffectorJacobianTemporalDerivative(2, 1) = J21_dot;
            obj.endEffectorJacobianTemporalDerivative(2, 2) = J22_dot;

            % update functionals
            obj.jointPositions{2} = obj.jointPositions{1} + [c1*l1; s1*l1; 0];
            obj.endEffectorPosition = obj.jointPositions{2} + [c12*l2; s12*l2; 0];
            obj.endEffectorVelocity = [-s1*l1*tau1 - s12*l2*tau2; c1*l1*tau1 + c12*l2*tau2];
            
            obj.endEffectorAcceleration = ...
                [- l1*(rho1*s1+tau1^2*c1) - l2*(rho2*s12+tau2^2*c12); ...
                 + l1*(rho1*c1-tau1^2*s1) + l2*(rho2*c12-tau2^2*s12); ...
                 0];
            


        end
        
        
    end
    
end
    
    
    