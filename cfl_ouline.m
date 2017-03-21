function [ddx,ddy,dda,C] = Cox_LineFit(ANG, DIS, RobotPose, SensorPose, LINEMODEL)

    % robot and sensor pose
    Rx = RobotPose(1,1); Ry = RobotPose(2,1); Ra = RobotPose(3,1); 
    sALFA = SensorPose(1); sBETA = SensorPose(2); sGAMMA = SensorPose(3);

    % pose correction to be estimated - update through the loop
    ddx = 0; ddy = 0; dda = 0;

    % maximum number of iterations allowed
    max_iterations = 15;
    
    % flag, if linear equation system is not solvable, set to 1
    no_update = 0;
  
    % 0) Normal vectors to the line segments
    % U: unit (normal) vector to each line located at the origin (n_line x 2)
    % RI: distance to origin for each line (n_line x 1)
    
    
    
    % REPEAT UNTIL THE PROCESS CONVERGE
    for iteration = 1:max_iterations
        
        % 1) Translate and rotate data points (from angs, meas to world coordinate 
              
        % 2) Find targets for data points
        % per point, distance and index to target line (n_points x 2)
        
        % 3) Set up system of linear equation, find b = (dx,dy,da)' from the LS
        % define the martrix A

        % if det(A'*A) == 0 means the inverse is not computable
        % it would happen in case of dependant or inconsistant equation
        if det(A'*A) == 0
            % this is very important! make sure you understand how this works
            no_update = 1;
            break; 
        end
                
        % b:[dx,dy,da] "position fix" - solution to linear equation system
        % TODO: solve the system of linear equations for b

        % s2: see cox paper - page 8 - section D.1
        % TODO: find the variance s2
        
        % C: (3x3) uncertainty of the the position fix
        % TODO: find the covariance matrix
        

        % 4) Add latest contribution to the overall congruence
        % TODO: update position fix (ddx, ddy, dda)
        
        % 5) Check if the process has converged
        if (sqrt(b(1)^2 + b(2)^2) < 5) && (abs(b(3)) < 0.10*pi/180)
            % did you already read about the "break"? you should
            % make sure you understand this
            break;
        end
    end
    
    % checking the validity of the result
    % if you understand how "break" works, you'll understand how this is working
    % read about what does || mean.
    if (iteration == max_iterations) || (no_update == 1)
        ddx = 0;
        ddy = 0;
        dda = 0;
        C = [1000^2 0 0;0 1000^2 0;0 0 (25*pi/180)^2];
        disp('SINGULAR');
    end
    
end
