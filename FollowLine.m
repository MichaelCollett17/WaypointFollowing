function [ state ] = FollowLine( state, lineToF, P, n, Wi)
    v = P.v_const;  %constant velocity of 50 cm/sec
    
    %unpack pose from state
    pose = state(1:3);
    
    kh = .05;
    
    chi = lineToF(3);
    while(~((([state(1);state(2)]-Wi).' * n)>=0))
    %while(~((((state(1)-Wi(1))* n(1))>=0)&&(((state(2)-Wi(2))* n(2))>=0)))
        %distance from the line
        R = [cos(chi), -sin(chi); sin(chi), cos(chi)].';
        l_t = -R * [lineToF(1); lineToF(2)];
        l_t_w = [R(1,1) R(1,2) l_t(1);...
            R(2,1) R(2,2) l_t(2);...
            0 0 1];
        l_pose = (l_t_w * pose);
        ey = l_pose(2);
        s = sign(ey);
        ey = min(abs(ey),50);
        ey = s* ey;
        
        theta_c = chi - deg2rad(55) * (2/pi) * atan(kh * ey);
        w = wrapToPi(theta_c - pose(3));
        
        pose = propagatePose(pose, v, w, P.delta_t);
        state = [pose; v; w; state(6)+P.delta_t];
        drawCar(state, P);
    end
end

