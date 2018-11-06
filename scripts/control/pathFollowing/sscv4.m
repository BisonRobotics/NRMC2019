%if speed is negative
%positve radius is backwards and to the right (same circle as positive
%speed)
%negative radius is backwards and to the left (same circle as positive
%speed)

%max speed is absolute value
%sign of speed indicates direction

%Speed is .5*(LeftAngularVel + RightAngularVel)*WheelRadius; (angularVel in Rad)
%Turn Radius is DistanceBetweenWheels/2 * (LeftAngularVel + RightAngularVel)/(RightAngularVel - LeftAngularVel)


function [Ul,Ur] = sscv4(speed, steering, AxelLen, MaxSpeed)

if (abs (steering) < 1000 && abs(steering) > .01)
  Ul = (4*(1/steering)*speed/(AxelLen) - 2*speed)*AxelLen/((1/steering)*4);
  Ur = 2*(speed) - Ul;
  
  maxabs = max(abs(Ul), abs(Ur));
  if (maxabs > MaxSpeed)
      speed = speed * MaxSpeed/maxabs;
      Ul = (4*(1/steering)*speed/(AxelLen) - 2*speed)*AxelLen/((1/steering)*4);
      Ur = 2*(speed) - Ul;
  end
elseif (abs (steering) <= 1000)
  Ul = -.5*speed;
  Ur = .5*speed;
elseif (abs(steering) <= .01)
  Ul = speed;
  Ur = speed;
end

end