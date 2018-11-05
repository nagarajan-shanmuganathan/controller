% Controller program that satisfies that safety properties

initDistance = 18;
initVelocity = 5;
maxBrake = -1;
maxAcceleration = 5;
N = 10;
delta = 1;
minDistance = 15;
currentDistance = initDistance
leadAcceleration = randi([maxBrake, maxAcceleration], 1, 1) %Lead car's initial acceleration


while currentDistance > minDistance %Need to add other cases
    [acceleration, distance] = control(initDistance, initVelocity, maxBrake, maxAcceleration, N, delta, minDistance)
    
    
end

function [acceleration, distance] = control(x0, v0, brake, acc, N, delta, minDistance)
   
    cvx_begin
        variable x(N);
        variable v(N);
        variable u(N);
        whos
        minimize(1/2 * sum(u.^2) + 1/2 * sum(v.^2));
        subject to
            norm(brake) <= u <= norm(acc);
            x >= norm(minDistance);  
            
            x(1) == x0;
            v(1) == v0;

            for i = 1:N-1
                x(i+1) == x(i) + delta * v(i)  + 0.5 * delta.^2 * u(i);
                v(i+1) == v(i) + delta * u(i);
            end
        
    cvx_end
    hold on
    plot(u)
    plot(x)
    acceleration = u(1);
    distance = x(1);
end
