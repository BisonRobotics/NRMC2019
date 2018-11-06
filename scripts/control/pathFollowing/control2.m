%attempt 2 to control the robot
%uses a speed and a steering state
%speed is ramped up
%steering is determined by what is demanded from the path
%at its current point at the current speed

%wheel velocities are determined by the current 
%steering at the current speed

%wheel velocities are tracked and used to estimate actual speed
%and actual steering

%first, compute tables for path
P0 = [1,1];
P1 = [2,1];
P2 = [2.5, .3];
P3 = [2.0, -.3];
chopsize = 100;

robotAxelLength = .5;

%get path points for plotting
index =0;
pathX = zeros(100,1);
pathY = zeros(100,1);
for t = linspace(0,1,100)
    index = index +1;
    pathX(index) = (1-t)^3 * P0(1) + 3*(1-t)^2 *t*P1(1) + 3*(1-t)*t^2 *P2(1) + t^3 * P3(1);
    pathY(index) = (1-t)^3 * P0(2) + 3*(1-t)^2 *t*P1(2) + 3*(1-t)*t^2 *P2(2) + t^3 * P3(2);
end



[othertheta,omega, alpha, lengths] = getAngleInfo(P0, P1, P2, P3, chopsize);
theta(1,:) = othertheta;
theta(2,:) = omega;
theta(3,:) = alpha;
length = sum(lengths(1,:));

robotStates = [1,1.2,0,0,0]; %x, y, theta, left, right
setSpeed = .7;
speedCmd = 0;
speedGain = 3;

angle_gain = 20;
path_gain = 100;

dt = .02;
totalTime = 5;
figure();

wheelVelPlot = zeros(2, totalTime/dt);
wheelCmdPlot = zeros(2, totalTime/dt);
closestTPlot = zeros(1, totalTime/dt);
steeringPlot = zeros(2, totalTime/dt);
speedPlot    = zeros(2, totalTime/dt);
anglePlot    = zeros(3, totalTime/dt);
pathErrorPlot= zeros(1, totalTime/dt);
timePlot     = zeros(1, totalTime/dt);
plotIndex    = 0;

last_closest_t =0;
closest_t =0;
pathErr =0;
for t = 0:dt:totalTime
    plotIndex = plotIndex + 1;
    clf;
    plot(pathX, pathY, 'b--');
    robotdraw(robotStates(1), robotStates(2), robotStates(3), 0,0,0);
    hold off

    speedMes = .5*(robotStates(4) + robotStates(5));
    %turn radius is DistanceBetweenWheels/2 * (LeftAngularVel + RightAngularVel)/(RightAngularVel - LeftAngularVel)
    if (abs(robotStates(4) + robotStates(5)) < .01)
        steering =  (2/robotAxelLength) * (robotStates(5) - robotStates(4))/.01;
    else
        steering = (2/robotAxelLength) * (robotStates(5) - robotStates(4))/(robotStates(4) + robotStates(5));
    end
    
    last_closest_t =closest_t;
    [closest_t,pathErr] = findCPP2019(robotStates(1), robotStates(2),P0, P1, P2, P3, chopsize);
    if (closest_t < last_closest_t)
        closest_t = last_closest_t;
    end
                                
    angle_error = angleDiff(robotStates(3),theta(1,int32(closest_t*chopsize)));
    
    speedCmd = speedCmd - speedGain*(speedCmd - setSpeed)*dt;
    
    %for speed and steering, get wheel velocities
    
    %take steering from point in path speed * dt meters ahead
    %this decreases error slightly
    meters_to_jump = speedCmd * dt;
    jumped_meters =0;
    t_jumps = 0;
    while (jumped_meters < meters_to_jump && (int32(closest_t*chopsize) + t_jumps) < chopsize)
        jumped_meters = jumped_meters + lengths(1,int32(closest_t*chopsize) + t_jumps);
        t_jumps = t_jumps+1;
    end
    
    [leftCommand, rightCommand] = sscv4(speedCmd, theta(2,int32(closest_t*chopsize) + t_jumps)...
                                        - angle_gain*angle_error - path_gain*pathErr, ...
                                          robotAxelLength, 2);
    
    %TODO add noise and disturbances
    disturbanceL = 1;
    disturbanceR = 1;
    if (t >= 3 && t <=3.3)
        disturbanceL = .1;
    end
    if (t >= 2.2 && t <=2.6)
        disturbanceR = 0;
    end
    robotStates = robotdynamics(robotStates, disturbanceL*leftCommand, disturbanceR*rightCommand, dt, robotAxelLength, 8);
    
    wheelCmdPlot(:, plotIndex) = [leftCommand, rightCommand];
    wheelVelPlot(:, plotIndex) = [robotStates(4); robotStates(5)];
    steeringPlot(:, plotIndex) = [steering, theta(2,int32(closest_t*chopsize))];
    speedPlot(:, plotIndex) = [speedCmd, speedMes];
    anglePlot(:, plotIndex) = [theta(1,int32(closest_t*chopsize)), robotStates(3), angle_error];
    pathErrorPlot(1, plotIndex) = pathErr;

    closestTPlot(1, plotIndex) = closest_t;
    timePlot(1,plotIndex) = t;
end

figure();
subplot(6,1,1);
plot(timePlot, wheelVelPlot(1,:),timePlot, wheelCmdPlot(1,:), '--',timePlot, wheelVelPlot(2,:),timePlot, wheelCmdPlot(2,:), '--');
legend("leftWheel", "leftWheelCmd", "rightWheel", "rightWheelCmd");
subplot(6,1,2);
plot(timePlot, closestTPlot);
legend("closest_t");
subplot(6,1,3);
plot(timePlot, steeringPlot(1,:), timePlot, steeringPlot(2,:), 'b--');
legend("steering measured", "steering desired from path");
subplot(6,1,4);
plot(timePlot, speedPlot(1,:), timePlot, speedPlot(2,:), 'b--');
legend("speedCmd", "speedMes");
subplot(6,1,5);
plot(timePlot, anglePlot(1,:), 'r-', timePlot, anglePlot(2, :), 'b-', timePlot, 100*anglePlot(3,:), 'g--');
legend("path angle", "robot_angle", "100x error");
subplot(6,1,6);
plot(timePlot, pathErrorPlot);
legend("path_error");