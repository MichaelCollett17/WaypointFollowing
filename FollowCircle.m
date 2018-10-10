function [ state ] = FollowCircle( state, circleToFollow, lambda, Wi, n, P )    
    v = P.v_const;
    pose = state(1:3);
    circlex = circleToFollow(1);
    circley = circleToFollow(2);
    radius = circleToFollow(3);
    while(~((([state(1);state(2)]-Wi).' * n)>=0))
        bodyx = state(1);
        bodyy = state(2);
        bodytheta = state(3);
        d = dist(circlex, circley, bodyx, bodyy) - radius;
        phi = atan2(bodyy-circley, bodyx-circlex);
        ideal = wrapToPi(phi+lambda*pi/2);
        control = wrapToPi(ideal + lambda * atan(.25 * d));
        diff = wrapToPi(control-bodytheta);
        w = diff;

        pose = propagatePose(pose, v, w, P.delta_t);
        state = [pose; v; w; state(6)+P.delta_t];
        drawCar(state, P);
    end

end

