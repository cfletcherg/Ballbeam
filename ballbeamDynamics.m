classdef ballbeamDynamics < handle
    %  Model the physical system
    %----------------------------
    properties
        state
        m1
        m2
        L
        g
        Ts
    end
    %----------------------------
    methods
        %---constructor-------------------------
        function self = ballbeamDynamics(P)
            % Initial state conditions
            self.state = [...
                        P.z0;...          % z initial position
                        P.theta0;...      % Theta initial orientation
                        P.zdot0;...       % zdot initial velocity
                        P.thetadot0;...   % Thetadot initial velocity
                        ];     
           
            self.m1 = P.m1;  
            self.m2 = P.m2;  
            self.L = P.length;  
            self.g = P.g;  
            self.Ts = P.Ts; 
          
        end
        %----------------------------
       function self = propagateDynamics(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            
            % Note: this is where you would make the call to propogate 
            % the dynamics using either Runge-Kutta or ODE45.  Both methods
            % will call the function self.derivatives.  In order to fit the
            % ODE45 method signature, the first parameter in the function
            % is the current time 't'.  This isn't used in either method,
            % but needs to be there.  If you use Runge-Kutta then you
            % should remove the 't' parameter and similarly remove the '0'
            % when the function gets called.
            
            %% Use either Runge-Kutta or ODE45 by uncommenting the 
            % corresponding code.  Do not use both.
            
            %Integrate ODE using Runge-Kutta RK4 algorithm
            k1 = self.derivatives(0,self.state, u);
            k2 = self.derivatives(0,self.state + self.Ts/2*k1, u);
            k3 = self.derivatives(0,self.state + self.Ts/2*k2, u);
            k4 = self.derivatives(0,self.state + self.Ts*k3, u);
            self.state = self.state + self.Ts/6 * (k1 + 2*k2 + 2*k3 + k4);
            
            % Integrate ODE using ODE45 algorithm
            % [t xdot]= ode45(@self.derivatives,[0:self.Ts:self.Ts],self.state',[],u);
            % self.state = xdot(end,:)';
            
        end
        %----------------------------
        function xdot = derivatives(self,t, state, u)
            %
            % Return xdot = f(x,u), the derivatives of the continuous states, as a matrix
            % 
            
            % Put your equations of motion here...
            z = state(1);
            theta = state(2);
            zdot = state(3);
            thetadot = state(4);
            
            zddot = (z*(thetadot)^2)-(self.g*sin(theta));
            
            force = u*self.L*cos(theta);
            beam_f = .5*self.m2*self.g*self.L*cos(theta);
            ball_f = self.m1*self.g*z*cos(theta);
            theta_acc = 2*self.m1*z*zdot*thetadot;
            divisor = self.m1*z^2+self.m2*(self.L^2/3);
            thetaddot = (force-theta_acc-ball_f-beam_f)/(divisor);
            
            
            xdot = [zdot; thetadot; zddot; thetaddot];

        end
        %----------------------------
        function y = outputs(self)
            %
            % Returns the measured outputs as a list
            % [z, theta] with added Gaussian noise
            % 
            z = self.state(1);
            theta = self.state(2);
            y = [z; theta];

        end
        %----------------------------
        function x = states(self)
            %
            % Returns all current states as a list
            %
            x = self.state;
        end
    end
end


