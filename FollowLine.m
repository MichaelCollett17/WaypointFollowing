function [ state ] = FollowLine( state, lineToFollow, P, n, Wi)
%FollowLine inputs:
% state of the vehicle at the beginning
% line you wish the vehicle to follow
% the time to follow the line
% the parameters set P
% outputs are:  the state after the time has elapsed.
    v = P.v_const;  %constant velocity of 50 cm/sec
    
    %unpack pose from state
    pose = state(1:3);
    
    kh = 15;
    
    chi = lineToFollow(3);
    while(~((([state(1);state(2)]-Wi).' * n)>=0))
    
        %TODO define the function to set the angular velocity (w) based on
        %distance from the line
        R = [cos(chi), -sin(chi); sin(chi), cos(chi)]';
        l_t = -R * [lineToFollow(1); lineToFollow(2)];
        l_t_w = [R(1,1) R(1,2) l_t(1);...
            R(2,1) R(2,2) l_t(2);...
            0 0 1];
        l_pose = (l_t_w * pose);
        ey = l_pose(2);
        s = sign(ey);
        ey = min(abs(ey),40);
        ey = s* ey;
        
        theta_c = chi - 45 * (2/pi) * atan(kh * ey);
        w = wrapToPi(theta_c - pose(3));
        
        pose = propagatePose(pose, v, w, P.delta_t);
        state = [pose; v; w; state(6)+P.delta_t];
        drawCar(state, P);
    end
end

