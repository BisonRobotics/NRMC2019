%solves the 2D IMU transform problem
%given an angle of CCW rotation offset (in radians)
%a displacement vector to the imu in the base frame, ([x;y])
%the acceleration measurements for x,y at the imu
%and the rotation rate w 
%as well as its derivitive alpha
%returns the compensated x and y accelerations of the base

function [x_accel, y_accel] = imutransform(angle, d, xm, ym, wm, am)
centripetal_accel = wm*wm *d;% * norm(d) * d/norm(d); %in direction of d
tangential_accel = am * [cos(-sign(wm)*pi/2), -sin(-sign(wm)*pi/2);
                         sin(-sign(wm)*pi/2), cos(-sign(wm)*pi/2)] * d;% * norm(d) * d/norm(d); %ortho to d
%rotate linear measurements to be aligned with base frame
lin_mea = [cos(angle), -sin(angle); sin(angle), cos(angle)] * [xm;ym];

lin_comp = lin_mea - centripetal_accel - tangential_accel;
    
x_accel = lin_comp(1);
y_accel = lin_comp(2);
end