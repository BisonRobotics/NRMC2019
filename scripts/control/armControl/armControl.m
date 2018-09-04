%attempt to control arm using tight feed forward model  with small feedback
%gains

figure();
x = zeros(3,1);
x(1) = -3*pi/4;
x_prev = x;
dt = .02;
time = 0:dt:6;

arm_length = .5;
arm_mass = 5;

U = 0;

UPlot = zeros(1,length(time));
XPlot = zeros(3,length(time));
plotIndex = 1;

for t = time
    subplot(3,1,1);
    cla;
    x = armSim(x, U, dt, arm_length, arm_mass);
    armDraw(x, arm_length);
    
    U = 24*(x(1) - (pi/6 - pi/2))  + 15*x(2) + -arm_length * cos(x_prev(1));
    
    UPlot(1,plotIndex) = U;
    XPlot(:,plotIndex) = x;
    subplot(3,1,2);
    plot(0:dt:t, UPlot(1,1:plotIndex));
    legend('U');
    subplot(3,1,3);
    plot(0:dt:t, XPlot(1,1:plotIndex));
    legend('X');
    plotIndex = plotIndex +1;

end