function [ state ] = WaypointController_throughPoint( state, W, P)
%WaypointController
%TODO - Develop the code to move from waypoint to waypoint going through
% the waypoint (or the halfplane at the waypoint) before moving to the next
%Utilize your line following functions from the previous projects.

% loop through W and go line by line with line following algorithm and
% half plane algorithm

% define line and halfplane 
wCurr = 1;
[WrowLength WColLength] = size(W);

Wimin1 = [state(1); state(2)]
while(wCurr < WColLength)
    Wi = W(:,wCurr)
    Wiplus1 = W(:,wCurr+1)
    
    line = [Wimin1(1); Wimin1(2); atan2(Wi(2)-Wimin1(2), Wi(1)-Wimin1(1))];
    
    qimin1 = (Wi - Wimin1)/norm(Wi - Wimin1);
    qi = (Wiplus1 - Wi)/norm(Wiplus1 - Wi);
    
    n = (qimin1 + qi)/norm(qimin1 + qi);
    
    state = FollowLine(state, line, P, n, Wi);
    wCurr = wCurr + 1;
    state
    Wimin1 = W(:,wCurr-1)
end
% final pass


end

