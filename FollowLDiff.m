function [ state ] = FollowLDiff( state, lineToFollow, Wi, n, P)
    v = P.v_const;  %constant velocity of 50 cm/sec
    kh = 0.09;
    pose = state(1:3);
    while(~((([state(1);state(2)]-Wi).' * n)>=0))
        x = lineToFollow(1);
        y = lineToFollow(2);
        w_b = [pose(1); pose(2); 1];
        w_T_l = [...
            cos(lineToFollow(3)), -sin(lineToFollow(3)), x;...
            sin(lineToFollow(3)),  cos(lineToFollow(3)), y;...
            0                   ,  0                   , 1 ;...
            ];
        l_T_w = inv(w_T_l);
        l_b = l_T_w*w_b;         
        theta_c = lineToFollow(3)-((2/pi)*(deg2rad(55))*atan(kh*l_b(2)));       
        theta_diff = theta_c - pose(3);
        
        pose = propagatePose(pose, v, theta_diff, P.delta_t);
        state = [pose; v; theta_diff; state(6)+P.delta_t];
        drawCar(state, P);
    end
end
