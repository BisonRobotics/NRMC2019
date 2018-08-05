%findCPP2019
%given a point and the four points which define a bezier curve
%finds the closest point on the curve (furthest along the curve)
%and returns the parameter value for that point
%as well as the magitude of the error

%for now, just does a comparison at each point on the path (gross)

function [t, pathError] = findCPP2019(xp, yp, P0, P1, P2, P3, chopsize)

index =0;
x = zeros(chopsize,1);
y = zeros(chopsize,1);
xd1 = zeros(chopsize,1);
yd1 = zeros(chopsize,1);

for t = linspace(0,1,chopsize)
    index = index +1;
    x(index) = (1-t)^3 * P0(1) + 3*(1-t)^2 *t*P1(1) + 3*(1-t)*t^2 *P2(1) + t^3 * P3(1);
    y(index) = (1-t)^3 * P0(2) + 3*(1-t)^2 *t*P1(2) + 3*(1-t)*t^2 *P2(2) + t^3 * P3(2);
    
    xd1(index) = 3*(1-t)^2 * (P1(1) - P0(1)) + 6*(1-t)*t*(P2(1) - P1(1)) + 3*t^2*(P3(1) - P2(1));
    yd1(index) = 3*(1-t)^2 * (P1(2) - P0(2)) + 6*(1-t)*t*(P2(2) - P1(2)) + 3*t^2*(P3(2) - P2(2));
end

dists = zeros(chopsize,1);
smallestIndex =chopsize;
smallestDist = 10000;
for index = 1:chopsize
    dists(index,1) = (x(index) - xp).^2 + (y(index) - yp).^2;
    if (dists(index,1) < smallestDist)
       smallestDist = dists(index,1);
       smallestIndex = index; 
    end
end

%need to sign path error based on to the left or right of the path
%find angle between current point and CPP and 
%take difference between that and angle of path at that point
%if its between 0 and pi, its above the path
%between -pi and -0, its below the path
pos_and_cpp_angle = atan2((yp - y(smallestIndex)), (xp - x(smallestIndex)));

pathError = sign(angleDiff(pos_and_cpp_angle, atan2(yd1(smallestIndex), xd1(smallestIndex)))) * sqrt(smallestDist);
t = smallestIndex/chopsize;

end