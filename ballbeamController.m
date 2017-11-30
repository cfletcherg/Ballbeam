classdef ballbeamController < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        zCtrl
        thetaCtrl
        m1
        m2
        g
        length
        theta_max
        Fmax
        K
        Ts
        z0
        theta0
        zdot
        thetadot
        beta
        integrator
        ki
        error_d1
        
    end
    %----------------------------
    methods
        %----------------------------
        function self = ballbeamController(P)
            self.m1 = P.m1;
            self.m2 = P.m2;
            self.g = P.g;
            self.length = P.length;
            self.theta_max = P.theta_max;
            self.Fmax = P.Fmax;
            self.K = P.K;
            self.Ts = P.Ts;
            self.z0 = P.z0;
            self.theta0 = P.theta0;
            self.zdot = P.zdot0;
            self.thetadot = P.thetadot0;
            self.beta = P.beta;
            self.integrator = 0.0;
            self.ki = P.ki;
            self.error_d1 = 0.0;
            
            % Instantiates the PD control objects
            self.zCtrl = PIDControl(P.kp_z, P.kd_z, P.ki_z, P.theta_max, P.beta, P.Ts);
            self.thetaCtrl = PIDControl(P.kp_th, P.kd_th, 0.0, P.Fmax, P.beta, P.Ts);
        end
        %----------------------------
        function F = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            theta = y(2);
            
            Fe = (self.m1*self.g*z + .5*self.m2*self.g*self.length)/(self.length);
            
            self.differentiateZ(z);
            self.differentiateTheta(theta);
            
            % integrate error
            error = z_r - z;
            self.integrateError(error);
            
            % construct state
            x = [z; theta; self.zdot; self.thetadot];
            F_tilda = -self.K*x - self.ki*self.integrator;
            % Implement your controller here...
            
            % You may choose to implement the PD control directly or call the 
            % PDControl class.  The PDControl class will return a force output 
            % for the given reference input and current state.
            % i.e. for the z-controller (already set up in the constructor)
            % call: z_force = self.zCtrl.PD(z_r, z, false);
            % For the theta controller call: 
            %       theta_force = self.thetaCtrl.PD(theta_r, theta, false);
            % You will need to determine what the output is for these
            % controllers in relation to the block diagrams derived for the
            % inner and outer loop control.
            F_unsat = Fe + F_tilda;
            F = self.saturateF(F_unsat);
            self.integratorAntiWindup(F, F_unsat);
            
        end
        %-------------------------
        function self = differentiateZ(self, z)
            self.zdot = ...
                self.beta*self.zdot...
                + (1-self.beta)*((z-self.z0) / self.Ts);
            self.z0 = z;
        end
        %-------------------------
        function self = integratorAntiWindup(self, u_sat, u_unsat)
            self.integrator = self.integrator + self.Ts/self.ki*(u_sat-u_unsat);
        end
        %-------------------------
        function self = differentiateTheta(self, theta)
            self.thetadot = ...
                self.beta*self.thetadot...
                + (1-self.beta)*((theta-self.theta0) / self.Ts);
            self.theta0 = theta;
        end
        %-------------------------
        function self = integrateError(self, error)
            self.integrator = self.integrator + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end
        %-------------------------
        function out = saturateF(self, u)
            if abs(u) > self.Fmax
                u = self.Fmax*sign(u);
            end
            out = u;
        end
    end
end