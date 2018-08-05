%attempt 1 to control the robot
%uses a speed and a steering state
%speed is ramped up with a PD loop
%steering is fed its change from the change demanded by the path
%at its current point at the current speed as well as any error correction

%wheel velocities are determined by the current 
%steering at the current speed

%change in steering at current speed means
%dUl = speed * (cos(steering+Dsteering +pi/4) - cos(steering + pi/4))
%dUr = speed * (sin(steering+Dsteering +pi/4) - sin(steering + pi/4))
%pi/4 is because steering=0 is at 45 deg,
%   might not be needed for calculations

%wheel velocities are tracked and used to estimate actual speed
%and actual steering



%first, compute tables for path
P0 = [1,1];
P1 = [2,1];
P2 = [2.5, .3];
P3 = [3.3, -.3];
chopsize = 100;
%get path points for plotting
index =0;
pathX = zeros(100,1);
pathY = zeros(100,1);
for t = linspace(0,1,100)
    index = index +1;
    pathX(index) = (1-t)^3 * P0(1) + 3*(1-t)^2 *t*P1(1) + 3*(1-t)*t^2 *P2(1) + t^3 * P3(1);
    pathY(index) = (1-t)^3 * P0(2) + 3*(1-t)^2 *t*P1(2) + 3*(1-t)*t^2 *P2(2) + t^3 * P3(2);
end

[theta,length] = getAngleInfo(P0, P1, P2, P3, chopsize);

robotStates = [1,1,0,0,0];
setSpeed = .7;
pathErrGain = 0;

dt = .02;
totalTime = 10;
figure();

wheelVelPlot = zeros(2, totalTime/dt);
wheelCmdPlot = zeros(2, totalTime/dt);
pathErrPlot = zeros(2,totalTime/dt);
closestTPlot = zeros(1,totalTime/dt);
timePlot = zeros(1,totalTime/dt);
plotIndex =0;


last_closest_t =0;
closest_t =0;
u_pathErr =0;
pathErr =0;
for t = 0:dt:10
    plotIndex = plotIndex + 1;
    clf;
    plot(pathX, pathY, 'b--');
    robotdraw(robotStates(1), robotStates(2), robotStates(3), 0,0,0);
    hold off

    
    speed = sqrt(robotStates(4)^2 + robotStates(5)^2);
    
    last_closest_t =closest_t;
    [closest_t,u_pathErr] = findCPP2019(robotStates(1), robotStates(2),P0, P1, P2, P3, chopsize);
    if (closest_t < last_closest_t)
        closest_t = last_closest_t;
    end
    
    %negative path error means turn left
    pathErr = u_pathErr * sign( atan2(robotStates(2) - pathY(int32(closest_t * chopsize)), robotStates(1) - pathX(int32(closest_t * chopsize))) ...
                                - theta(1,int32(closest_t * chopsize)));
                            
% %     %for this speed at this point on the path, find change in steering
% %     %speed is m/s, dtheta is rad/meter. so speed * dtheta is rad/s
% %     %want rad/s2, so take dt of speed*dtheta
% %     %thats accel*dtheta + dtheta/dt * speed
% %     %how is dtheta changing with respect to time?
% %     theta(3, int32(closest_t))
% %     %integrate change in steering
    %speed * dtheta is rad/s is desired steering at that point
    %%%steeringCmd = speed * theta(2, int32(closest_t));
    
    speed = speed - 1*(speed - setSpeed)*dt;
    
    %for speed and steering, get wheel velocities

    %wheel commands are those looked up from the path
    %as well as path error compensation term
    %and speed projected along path error vector (line from cpp to robot) term
    %  (this should be the derivitive of the path error)
    dPathError = speed * cos(robotStates(3) - theta(1,int32(closest_t*chopsize))) * dt;
    
    leftCommand = speed * cos(theta(2, int32(closest_t*chopsize)) + pi/4) + pathErrGain * pathErr;
    rightCommand = speed * sin(theta(2, int32(closest_t*chopsize)) + pi/4) - pathErrGain * pathErr;
    robotStates = robotdynamics(robotStates, leftCommand, rightCommand, dt, .5, 8);
    
    
    wheelCmdPlot(:, plotIndex) = [leftCommand, rightCommand];
    wheelVelPlot(:, plotIndex) = [robotStates(4); robotStates(5)];
    pathErrPlot(1, plotIndex) = pathErr;
    pathErrPlot(2, plotIndex) = dPathError;
    closestTPlot(1, plotIndex) = closest_t;
    timePlot(1,plotIndex) = t;

end

figure();
subplot(3,1,1);
plot(timePlot, wheelVelPlot(1,:),timePlot, wheelCmdPlot(1,:), '--',timePlot, wheelVelPlot(2,:),timePlot, wheelCmdPlot(2,:), '--');
legend("leftWheel", "leftWheelCmd", "rightWheel", "rightWheelCmd");
subplot(3,1,2);
plot(timePlot, closestTPlot);
legend("closest_t");
subplot(3,1,3);
plot(timePlot, pathErrPlot(1,:), timePlot, pathErrPlot(2,:), 'b--');
legend("pathErr", "dPathErr");

