%robotdynamics
%takes an array containing the robot's states
%robotStates = [x,y,theta,leftSpeed,rightSpeed]
%and the two desired wheel speeds leftSpeed, rightSpeed
%   (in meters/s travelled by the wheel)
%and dt and AxelLength
%and a gain for the VESCS
%returns the new robotStates

%TODO add turn dynamics (tighter turn ~> more disturbance)
function newRobotStates = robotdynamics(robotStates, leftSpeed,rightSpeed,dt, AxelLen, Pgain)
    worldTheta = robotStates(3);
    Ul = robotStates(4) - Pgain*(robotStates(4) - leftSpeed)*dt;
    Ur = robotStates(5) - Pgain*(robotStates(5) - rightSpeed)*dt;

    w = (Ur - Ul)/AxelLen; %positive is CCW, i.e. if right wheel is faster, angle is positive
    if (w~=0)
      R = AxelLen/2 * (Ur + Ul)/(Ur - Ul);
      %let R be on the robots Y axis
      rot = [cos(w*dt), -sin(w*dt);sin(w*dt),cos(w*dt)];
      dPos = rot*[0;-R] + [0;R];
      dTheta = w*dt;
    else
      dPos = [Ul;0;]*dt;
      dTheta = 0;
    end

    %dPos is in robot coordinates, must transform to world.
    %rotate by worldrobot theta

    wrot = [cos(worldTheta), -sin(worldTheta);sin(worldTheta),cos(worldTheta)];
    dPosW = wrot*dPos;

    newRobotStates = [robotStates(1) + dPosW(1);
                      robotStates(2) + dPosW(2);
                      robotStates(3) + dTheta;
                      Ul;
                      Ur]; 
end