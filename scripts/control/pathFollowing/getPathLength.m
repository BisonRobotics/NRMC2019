%getPathLength
%computes the lenght of a bezier curve given its four points

function length = getPathLength(P0, P1, P2, P3)
    index = 0;
    length =0;
    for t = linspace(0,1,100)
        index = index +1;
        xd1 = 3*(1-t)^2 * (P1(1) - P0(1)) + 6*(1-t)*t*(P2(1) - P1(1)) + 3*t^2*(P3(1) - P2(1));
        yd1 = 3*(1-t)^2 * (P1(2) - P0(2)) + 6*(1-t)*t*(P2(2) - P1(2)) + 3*t^2*(P3(2) - P2(2));
        length = length + sqrt(xd1^2 + yd1^2);
    end
end