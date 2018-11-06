%Plots worst case distance travelled based on max speed, decelleration
%rate, and sensor sampling

stopline = .023;% meters
targetZone = .0127; %meters: .5 inch margins
sensorErrStdDev = .001; % 95 percent of error under .5 cm
pGain = 4; %speed_error * dt multiplier for acceleration /dt
           %so setpoint of -.0635 * .01 = -.000635;
           %-.000635 *2 => .127 m/s^2 accel initial
           %               = 5"/s^2 accel initial
MaxVel = -.0635; %m/s from Sam 2.5"
sensorSamplePeriod = 1/20; %sensor freq^-1
InitialAccel = MaxVel * pGain;
fprintf('InitialAccel = %.4f m/s^2 \n             = %.5f in/s^2 \n', InitialAccel, InitialAccel/.0254);
fprintf('Settling Time = %.4f s\n', 4/pGain);
dt = .01; %simulation/control time for controller
time = 0:dt:8;

posPlot = zeros(1,length(time),11,3); %11 is number of different init values
velPlot = zeros(1,length(time),11,3); %for the sensor sampling, 3 is noise test values

figure();
plot(time, stopline + zeros(1,length(time)), 'c--', time, 0  + zeros(1,length(time)), 'r--',...
     time, targetZone + zeros(1,length(time)), 'y--');
hold on;

sampleReadyInit = linspace(0,sensorSamplePeriod,11);
noiseExtremes = [-2 * sensorErrStdDev, 0, 2*sensorErrStdDev];

for sampleTimeIndex = 1:11
sampleReady = sampleReadyInit(sampleTimeIndex);
samplePeriod = sensorSamplePeriod; %sample every .1 s from limit sensor

for noiseIndex = 1:3
noise = noiseExtremes(noiseIndex);

position = .2; %initial position
velocity = 0; %initial vel
positionMea = .2;
speedSet = MaxVel;

plotIndex = 1;
for t = time
    velocity = velocity + pGain*(speedSet - velocity)*dt;
    position = position + velocity*dt;
    if (positionMea < stopline)
        speedSet = 0;
    end
    
    sampleReady = sampleReady + dt;
    if (sampleReady >= samplePeriod)
        sampleReady = 0;
        positionMea = position + noise; %todo NOISE
    end
    %TODO observer for position which incorporates velocity measurements
    
    posPlot(1,plotIndex, sampleTimeIndex, noiseIndex) = position;
    velPlot(1,plotIndex, sampleTimeIndex, noiseIndex) = velocity;
    plotIndex = plotIndex +1;
end
pplot = posPlot(1,:, sampleTimeIndex, noiseIndex);
vplot = velPlot(1,:, sampleTimeIndex, noiseIndex);
plot(time, pplot,'b--', time, vplot,'g--');
end
end
legend('Stop', 'Zero', 'Margin','Pos', 'Vel');
hold off

xlabel('Time (s or s^2)');
ylabel('Meters (m or m^2)');
