function [ state ] = WaypointController_throughPoint( state, W, P)
wCurr = 2;
W_2 = [[state(1); state(2)], W(:,1),W(:,2), W(:,3), W(:,4), [W(1,4)+ 10; W(2,4)]]
[WrowLength WColLength] = size(W_2);
    while(wCurr < WColLength)
        Wimin1 = W_2(:,wCurr-1)
        Wi = W_2(:,wCurr)
        Wiplus1 = W_2(:,wCurr+1)

        %line = [Wimin1(1); Wimin1(2); atan2(Wi(2)-Wimin1(2), Wi(1)-Wimin1(1))];
        qimin1 = (Wi - Wimin1)/norm(Wi - Wimin1);
        qi = (Wiplus1 - Wi)/norm(Wiplus1 - Wi);

        n = (qimin1 + qi)/norm(qimin1 + qi);

        line = [Wimin1(1); Wimin1(2); atan2(qimin1(2),qimin1(1))]

        state = FollowLDiff( state, line, Wi, n, P);
        wCurr = wCurr + 1;

    end
end

