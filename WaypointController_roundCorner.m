function [ state ] = WaypointController_roundCorner( state, W, P )
    wCurr = 2;
W_2 = [[state(1); state(2)], W(:,1),W(:,2), W(:,3), W(:,4), [W(1,4)+ 10; W(2,4)]]
R = 100;
[WrowLength WColLength] = size(W_2);
    while(wCurr < WColLength)
        Wimin1 = W_2(:,wCurr-1)
        Wi = W_2(:,wCurr)
        Wiplus1 = W_2(:,wCurr+1)

        qimin1 = (Wi - Wimin1)/norm(Wi - Wimin1);
        qi = (Wiplus1 - Wi)/norm(Wiplus1 - Wi);
        
        alpha = acos((-1*qimin1).' * qi)
        H = (R/(sin(alpha/2)));
        Z = H-R;
        M = (qi - qimin1)/norm(qi-qimin1);
        ci = Wi + M * H;
        lambda = sign((qimin1(1) * qi(2)) - (qimin1*qi(1))); % clockwise if -1 counterclock if 1
        if(wCurr<3)
            lambda = -1*lambda;
        end
        n1 = qimin1;
        n2 = qi;
        d1 = R/(tan(alpha/2));
        r1 = Wi - qimin1*d1;
        r2 = Wi + qi*d1;
        line1 = [Wimin1(1); Wimin1(2); atan2(qimin1(2),qimin1(1))];
        state = FollowLDiff( state, line1, r1, n1, P);
        circle = [ci(1);ci(2);R];
        drawCircle(circle,'r');
        state = FollowCircle(state, circle, lambda, r2, n2, P);
        line2 = [r2(1);r2(2);atan2(qi(2),qi(1))];
        state = FollowLDiff(state, line2, Wiplus1, n2, P);
        wCurr = wCurr+2;
    end
end