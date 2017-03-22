function [ddx,ddy,dda,C] = Cox_LineFit(ANG, DIS, RobotPose,SensorPose, LINEMODEL)

    % robot and sensor pose  ...
    Rx_ = RobotPose(1,1); Ry_ = RobotPose(2,1); Ra_ = RobotPose(3,1); 
    sALFA = SensorPose(1); sBETA = SensorPose(2); sGAMMA = SensorPose(3);
    U= LINEMODEL(:,1:2);
    RI = LINEMODEL(:,3);
    % pose correction to be estimated - update through the loop
    ddx = 0; ddy = 0; dda = 0;

    % maximum number of iterations allowed
    max_iterations = 15;
    
    % flag, if linear equation system is not solvable, set to 1
    no_update = 0;
  
    % 0) Normal vectors to the line segments
    % U: unit (normal) vector to each line located at the origin (n_line x 2)
    % RI: distance to origin for each line (n_line x 1)
     %[U RI] = get_normal_and_distance(LINEMODEL);
    
    
    
    % REPEAT UNTIL THE PROCESS CONVERGE
    for iteration = 1:max_iterations
        
        % 1) Translate and rotate data points (from angs, meas to world coordinate 
        
        Rx = Rx_+  ddx;
        Ry = Ry_+  ddy;
        Ra = Ra_+ dda;
        
        x = DIS.*cos(ANG);
        y = DIS.*sin(ANG);
        
        
        R = [cos(sGAMMA) -sin(sGAMMA) sALFA;sin(sGAMMA) cos(sGAMMA) sBETA;0 0 1];
        Xs = R*[x' y' ones(1,length(x))']';
        
        
        R=[cos(Ra) -sin(Ra) Rx;sin(Ra) cos(Ra) Ry;0 0 1];
        Xw=R*[Xs(1,:)' Xs(2,:)' ones(1,length(x))']';
        

        % 2) Find targets for data points
        % per point, distance and index to target line (n_points x 2)
        TargetT =[];
        YTargetT = [];
        for j=1:length(Xw(1,:))
            for i=1:length(U(:,1))
                Y(i) = RI(i) - dot(U(i,:),Xw(1:2,j));
            end
            [minY indexL] = min(abs(Y));
            TargetT =[TargetT indexL];
            YTargetT = [YTargetT minY];
        end 
        
        % check the Th and reject all outliers
        Target =[];
        YTarget = [];      
        Th = 10;%min([1000^2,median(abs(Y))]);
        for i=1:length(TargetT)
          if(YTargetT(i) <Th)
                Target =[Target TargetT(i)];
                YTarget = [YTarget YTargetT(i)];
          end
        end
        % 3) Set up system of linear equation, find b = (dx,dy,da)' from the LS
        % define the martrix A
        if(length(Target)==0)
            ddx = 0;
            ddy = 0;
            dda = 0;
            C = [1000^2 0 0;0 1000^2 0;0 0 (25*pi/180)^2];
            disp('SINGULAR');
            break;
        end
        vm = [Rx Ry];%mean(Xw(1:2,:)');
        X1=[];
        X2=[];
        X3=[];
        for i = 1:length(Target)
            X1(i) = U(Target(i),1);
            X2(i) = U(Target(i),2);
            X3(i) = U(Target(i),:)*[0 -1;1 0]*(Xw(1:2,i)-vm');
        end
        A = [X1' X2' X3'];
        % if det(A'*A) == 0 means the inverse is not computable
        % it would happen in case of dependant or inconsistant equation
        if det(A'*A) == 0
            % this is very important! make sure you understand how this works
            no_update = 1;
            break; 
        end
          
        
        % b:[dx,dy,da] "position fix" - solution to linear equation system
        % TODO: solve the system of linear equations for b
        b = inv(A'*A)*A'*YTarget';
        % s2: see cox paper - page 8 - section D.1
        % TODO: find the variance s2
        s2 = (YTarget'-A*b)'*(YTarget'-A*b)/(length(YTarget)-4);
        % C: (3x3) uncertainty of the the position fix
        % TODO: find the covariance matrix
        
        C = s2*inv(A'*A);
        % 4) Add latest contribution to the overall congruence
        % TODO: update position fix (ddx, ddy, dda)
        
        ddx =   b(1);
        ddy =   b(2);
        dda =  b(3); 
        

        

        
        

        
%         R = [cos(sGAMMA) -sin(sGAMMA) sALFA;sin(sGAMMA) cos(sGAMMA) sBETA;0 0 1];
%         Xs = R*[Rx Ry 1]';
%         Rx = Xs(1);
%         Ry = Xs(2);
        
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
