% Controller program that satisfies that safety properties

initDist = 5;
initVelocity = 10;
maxBrake = -4;
maxAcceleration = 5;
N = 10;
delta = 1;
minDist = 10;

acceleration = control(initDist, initVelocity, maxBrake, maxAcceleration, N, delta, minDist)

function acceleration = control(x0, v0, brake, acc, N, delta, minDist) 
   
    cvx_begin
        variable x(N);
        variable v(N);
        variable u(N);
        whos
        minimize(1/2 * sum(u.^2) + 1/2 * sum(v.^2));
        subject to
            norm(brake) <= u <= norm(acc);
            x >= norm(minDist);  
            
            x(1) == x0;
            v(1) == v0;

            for i = 1:N-1
                x(i+1) == x(i) + delta * v(i)  + 0.5 * delta.^2 * u(i);
                v(i+1) == v(i) + delta * u(i);
            end
        
    cvx_end
    plot(u)
    acceleration = u;
end
