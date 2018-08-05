%getAngleInfo
%given the four points which define a bezier curve and a chopsize
%returns an array of theta of chopsize many points where the first
%row is theta at each point of the path
%second row is dtheta/ddist
%third row is d2theta/ddist2

function [theta,length] = getAngleInfo(P0, P1, P2, P3, chopsize)
 
    theta = zeros(3,chopsize);
    index = 0;
    length =0;
    for t = linspace(0,1,chopsize)
        index = index +1;
        xd1 = 3*(1-t)^2 * (P1(1) - P0(1)) + 6*(1-t)*t*(P2(1) - P1(1)) + 3*t^2*(P3(1) - P2(1));
        yd1 = 3*(1-t)^2 * (P1(2) - P0(2)) + 6*(1-t)*t*(P2(2) - P1(2)) + 3*t^2*(P3(2) - P2(2));
        %we already have all the d1's, might as well do length here
        %since we will need it
        length = length + sqrt((xd1/chopsize)^2 + (yd1/chopsize)^2);
        theta(1,index) = atan2(yd1, xd1);
    end
    
    for index = 1:(chopsize-1)
        theta(2,index) = (theta(1,index+1) - theta(1,index))*chopsize/length; %normalized by path length
    end  
    %repeat final values
    theta(2,chopsize) = theta(2,chopsize-1);
    
    for index = 1:(chopsize-2)
        theta(3,index) = (theta(2,index+1) - theta(2,index))*chopsize/length;
    end
    %repeat final values
    theta(3,chopsize-1) = theta(3, chopsize-2);
    theta(3,chopsize) = theta(3, chopsize-1);
end