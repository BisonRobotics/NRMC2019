close
clear
clc

V1=3; %Vertical distance from central axis to linear actuator - monoboom pivot.
H1=1.837; %Horizontal distance from central axis to linear actuator - monoboom pivot.
V2=1; %Vertical distance from central axis to drive link - monoboom pivot.
H2=17.748; %Horizontal distance from central axis to drive link - monoboom pivot.
H6=21.75; %Monoboom length
L23=4.119; %Distance between bottom two pivots on drive link
L34=1.631; %Distance between top two pivots on drive link.
L56=4.5; %Distance between bucket pivots.
L45=6.066; %Connecting link length
L67=9; %Backhoe bucket length
Phi3=115; %Angle between line connecting bucket pivots and line connecting backhoe tip with monoboom - bucket pivot.

%Linear force of linear actuator.
F6=500;

n=200;

%Array of lengths for linear actuator, symobolizes actuator extending.
A6=linspace(13.15,19.45,n);
A7=linspace(0,1,n);

for i=1:1:n
    L13=A6(i);
        
        L12=sqrt((V2+V1)^2+(H2-H1)^2);
        Theta1(i)=180-acosd(((L12^2)-(L13^2)-(L23^2))/(-2*L13*L23));
        Phi1(i)=acosd((L13^2-L12^2-L23^2)/(-2*L12*L23))+atand((V2+V1)/(H2-H1));
        V3(i)=L23*sind(Phi1(i))-V2;
        H3(i)=H2-L23*cosd(Phi1(i));
        V4(i)=(L23+L34)*sind(Phi1(i))-V2;
        H4(i)=H2-(L23+L34)*cosd(Phi1(i));
        L04(i)=sqrt(V4(i)^2+H4(i)^2);
        L46(i)=sqrt(V4(i)^2+(H6-H4(i))^2);
        Theta3(i)=180-acosd(((L46(i)^2)-(L45^2)-(L56^2))/(-2*L45*L56));
        Phi2(i)=acosd((L04(i)^2-(H6^2)-(L46(i)^2))/(-2*H6*L46(i)))+acosd((L45^2-L46(i)^2-L56^2)/(-2*L46(i)*L56));
        V5(i)=L56*sind(Phi2(i));
        H5(i)=H6-L56*cosd(Phi2(i));
        L35(i)=sqrt((H5(i)-H3(i))^2+(V5(i)-V3(i))^2);        
        Theta2(i)=acosd((L35(i)^2-L45^2-L34^2)/(-2*L45*L34));
        
        M6(i)=((sind(Theta1(i))*L23)/(sind(Theta2(i))*(L23+L34)))*sind(Theta3(i))*(L56/12)*F6;
        
        V7(i)=-L67*sind((Phi2(i)+Phi3)-180);
        H7(i)=L67*cosd((Phi2(i)+Phi3)-180)+H6;

end

%Plot of angle bucket has swung vs. extension of linear actuator.
figure
p = polyfit(A7,(Phi2 - Phi2(1)).*(pi/180),6);
y = polyval(p, A7);
plot(A6, Phi2, A6, y.*(180/pi));
title('Bucket Angle as a Function of Actuator Extension')
xlabel('7" Actuator Extension (in)')
ylabel('Bucket Angle (Degrees)')