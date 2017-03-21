function [U RI] = get_normal_and_distance(REF,LINES)

    LINEMODEL = [REF(LINES(:,1),1:2) REF(LINES(:,2),1:2)];
    nolines = length(LINEMODEL);
    U = zeros(nolines,2);
    RI = zeros(nolines,1);
    for l_idx = 1:nolines
        
       %rotate (pi/2) to get normal vector
       R = [0 -1;1 0];
       %translation vector to move p of line to the origin
       % T = [p1.x, p2.x;
       %       p1.y, p2.y]
       T(1,1:2) = LINEMODEL(l_idx,1); %p1.x
       T(2,1:2) = LINEMODEL(l_idx,2); %p1.y
       
        % V = [p1.x, p2.x;
        %       p1.y, p2.y]
        V = [LINEMODEL(l_idx,1:2:3); LINEMODEL(l_idx,1:2:4)];
        
        %V(1:2,:)-T moves the points to the origin
        %R*(V(1:2,:)-T) -> rotate around origin to get the normal vector
        
        V2 = R*(V(1:2,2)- T);
        
        %scaling the unt vector
        U(l_idx,1:2)= V2(1:2,2)' / sqrt(V2(1,2)^2 + V2(2,2)^2);
        
        %line distance to the origin
        
        RI(l_idx,1) = dot(U(l_idx,1:2)', LINEMODEL(l_idx,1:2)');
    end;
    
    




end
