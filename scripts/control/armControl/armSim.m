%updates the simulated massive arm given the previous state vector (theta,
%theta', theta'') the torque on the motor, dt, and the length and mass of
%the arm

function [x] = armSim(x_prev, torque, dt, length, m)
Tg = -mass * length * cos(x_prev(1)); %torque due to gravity
x(3) = (Tg - torque) / mass;
x(2) = x_prev(2) + x(3) * dt;
x(1) = x_prev(1) + x(2)*dt;
x = x(1:2);
end