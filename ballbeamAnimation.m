classdef ballbeamAnimation
    %
    %    Ballbeam animation
    %
    %--------------------------------
    properties
        ball_handle
        beam_handle
        length
        radius
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = ballbeamAnimation(P)
            self.length = P.length;
            self.radius = P.radius;
            
            
            figure(1), clf
            plot([0,0],[0,0],'k');
            hold on
            % initialize the ball and beam to initial conditions
            self=self.drawBall(P.z0, P.theta0);
            self=self.drawBeam(P.theta0);
            axis([-.25, .75, -.5, .5]);
        end
        %---------------------------
        function self=drawBallbeam(self, x)
            % Draw ballbeam is the main function that will call the functions:
            % drawBall and drawBeam to create the animation.
            % x is the system state
            z = x(1);       % Horizontal position of ball, m
            theta = x(2);   % Angle of beam, rads
            
            self=self.drawBall(z, theta);
            self=self.drawBeam(theta);
            drawnow
        end
        %---------------------------
        function self=drawBall(self, z, theta)
            
            % Put code here to draw your ball.
            % Save your data points into the X and Y vectors to draw below
            r = .03;
            c = [z r];
            n = 1000;
            newc = [cos(theta) -sin(theta); sin(theta) cos(theta)]*c';
            %// running variable
            q = linspace(0,2*pi,n);
            X = newc(1) + r*sin(q);
            Y = newc(2) + r*cos(q);
            
            % this will only 'draw' the data points if needed, otherwise it
            % will just change the values in the handle.  It will still
            % update the animation, but is faster than a redraw).
            if isempty(self.ball_handle)
                self.ball_handle = fill(X, Y, 'r');
            else
                set(self.ball_handle, 'XData', X, 'YData', Y);
            end
        end
        %---------------------------
        function self=drawBeam(self, theta)
            % Put code here to draw your beam.
            % Save your data points into the X and Y vectors to draw below
            xbeam = [0 self.length self.length 0];
            ybeam = [0 0 -.03 -.03];
            beam = [xbeam; ybeam];
            newbeam = [cos(theta) -sin(theta); sin(theta) cos(theta)]*beam;
            X = newbeam(1,:);
            Y = newbeam(2,:);
            % this will only 'draw' the data points if needed, otherwise it
            % will just change the values in the handle.  It will still
            % update the animation, but is faster than a redraw).
            if isempty(self.beam_handle)
                self.beam_handle = fill(X, Y, 'b');
            else
                set(self.beam_handle,'XData', X, 'YData', Y);
            end
        end
    end
end