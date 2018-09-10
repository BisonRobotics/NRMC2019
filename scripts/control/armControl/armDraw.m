function [] = armDraw(x, arm_length)
length = 1;
width = 1;
%clf
hold on;
plot([-width/2, width/2,width/2, -width/2, -width/2],[-length/2,-length/2,length/2,length/2, -length/2]);

plot([0,arm_length * cos(x(1))], [0,arm_length * sin(x(1))])

xlim([-width/2,width/2]);
ylim([-length/2,length/2]);
pbaspect([width, length,1]);
hold off;
pause(.01)
end
