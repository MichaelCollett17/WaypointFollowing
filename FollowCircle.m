function [ state ] = FollowCircle( state, circleToFollow, lambda, time, P )
%FollowCircle inputs:
% state of the vehicle at the beginning
% circle you wish the vehicle to follow
% the direction (lambda) = -1 for clockwise, 1 for clockwise
% the time to follow the circle
% the parameters set P
%outputs are:  the state after the time has elapsed.
    

    %set parameters
    v = P.v_const;
    
    %unpack pose
    pose = state(1:3);
    circlex = circleToFollow(1);
    circley = circleToFollow(2);
    radius = circleToFollow(3);
    for t = 0:P.delta_t:time
        bodyx = state(1);
        bodyy = state(2);
        bodytheta = state(3);
        d = dist(circlex, circley, bodyx, bodyy) - radius;
        phi = atan2(bodyy-circley, bodyx-circlex);
        ideal = wrapToPi(phi+lambda*pi/2);
        correct = wrapToPi(ideal + lambda * atan(.1 * d));
        diff = wrapToPi(correct-bodytheta);
        w = diff;

        pose = propagatePose(pose, v, w, P.delta_t);
        state = [pose; v; w; state(6)+P.delta_t];
        drawCar(state, P);
    end

end

