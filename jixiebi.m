clc ;clear
%syms theta1 theta2 theta3 theta4 theta5 theta6 d4 d5 d6 a2 a3
syms theta5
a2 = -0.425;
a3 = -0.3922;

d1 = 0.1625;
d2 = 0;
d3 = 0;
d4 = 0.1333;
d5 = 0.0997;
d6 = 0.0996;
theta1=pi/4;
theta2=pi/4;
theta3=pi/4;
theta4=pi/4;
%theta5=pi/4;
theta6=pi/4;

c1 = cos(theta1);
s1 = sin(theta1);
c2 = cos(theta2);
s2 = sin(theta2);
c3 = cos(theta3);
s3 = sin(theta3);
c4 = cos(theta4);
s4 = sin(theta4);
c5 = cos(theta5);
s5 = sin(theta5);
c6 = cos(theta6);
s6 = sin(theta6);
c23 = cos(theta2+theta3);
s23 = sin(theta2+theta3);
c34 = cos(theta4+theta3);
s34 = sin(theta4+theta3);
c234 = cos(theta2+theta3+theta4);
s234 = sin(theta2+theta3+theta4);
d234 = d2+d3+d4;

R01=[c1 0 s1;s1 0 -c1;0 1 0];
R12 = [c2 -s2 0; s2 c2 0; 0 0 1];
R23 = [c3 -s3 0; s3 c3 0; 0 0 1];
R34 = [c4 0 s4; s4 0 -c4; 0 1 0];
R45 = [c5 0 -s5; s5 0 c5; 0 -1 0];
R56 = [c6 -s6 0; s6 c6 0; 0 0 1];
R03 = R01*R12*R23;
R06 = R01*R12*R23*R34*R45*R56;



j11 = c1*d234 - s1*c2*a2 - s1*c23*a3 - s1*s234*d5 + (s1*c234*s5 + c1*c5)*d6;
j21 = s1*d234 + c1*c2*a2 + c1*c23*a3 + c1*s234*d5 + (s1*c5 - c1*c234*s5)*d6;
j31 = 0;
j41 = 0;
j51 = 0;
j61 = 1;

j12 = -c1*s2*a2 - c1*s23*a3 + c1*c234*d5 + c1*s234*s5*d6;
j22 = -s1*s2*a2 - s1*s23*a3 + s1*c234*d5 + s1*s234*s5*d6;
j32 = c2*a2 + c23*a3 + s234*d5 - c234*s5*d6;
j42 = s1;
j52 = -c1;
j62 = 0;

j13 = -c1*s23*a3 + c1*c234*d5 + c1*s234*s5*d6;
j23 = -s1*s23*a3 + s1*c234*d5 + s1*s234*s5*d6;
j33 = c23*a3 + s234*d5 - c234*s5*d6;
j43 = s1;
j53 = -c1;
j63 = 0;

j14 = c1*c234*d5 + c1*s234*s5*d6;
j24 = s1*c234*d5 + s1*s234*s5*d6;
j34 = s234*d5 - c234*s5*d6;
j44 = s1;
j54 = -c1;
j64 = 0;

j15 = -(s1*s5 + c1*c234*c5)*d6;
j25 = (c1*s5 - s1*c234*c5)*d6;
j35 = -s234*c5*d6;
j45 = c1*s234;
j55 = s1*s234;
j65 = -c234;

j16 = 0;
j26 = 0;
j36 = 0;
j46 = -c1*c234*s5 + s1*c5;
j56 = -s1*c234*s5 - c1*c5;
j66 = -s234*s5;

J = [j11 j12 j13 j14 j15 j16; j21 j22 j23 j24 j25 j26 ;j31 j32 j33 j34 j35 j36;
    j41 j42 j43 j44 j45 j46; j51 j52 j53 j54 j55 j56; j61 j62 j63 j64 j65 j66];
d=det(J);
vpa(solve(d==0))

theta = -pi;
L(1) = Link( [0, 0.1625, 0, 90] );
L(2) = Link( [0, 0, -0.425, 0] );
L(3) = Link( [0, 0, -0.3922, 0] );
L(4) = Link( [0, 0.1333, 0, 90] );
L(5) = Link( [0, 0.0997, 0, -90] );
L(6) = Link( [0, 0.0996, 0, 0] );

Six_link = SerialLink( L, 'name', 'Sixlink' );

theta = -185*pi/180:pi/1440:-175*pi/180;
theta1 = pi/4*ones(length(theta), 1);
theta2 = pi/4*ones(length(theta), 1);
theta3 = pi/4*ones(length(theta), 1);
theta4 = pi/4*ones(length(theta), 1);
theta6 = pi/4*ones(length(theta), 1);


for i=1:length(theta1)
    k = Six_link.fkine([theta1(i) theta2(i) theta3(i) theta4(i) theta(i) theta6(i)]);
    x(i) = k.t(1);
    y(i) = k.t(2);
    z(i) = k.t(3);
end

v = [0.1; 0.1; 0.1; 0.1; 0.1; 0.1;];
Lambda = 1;
eps = 1.5e-3;

plot3(x,y,z,'b','linewidth',1.5)
hold on
for i=1:length(theta1)
    alpha = [theta1(i) theta2(i) theta3(i) theta4(i) theta(i) theta6(i)];
    J=jacob(alpha);
    det(J);
    if (abs(det(J))<eps)
        sing = svd(J);
        lambda = sqrt((1-(sing(6)/eps)^2)*Lambda^2);
        omega = inv(transpose(J)*J+lambda^2*eye(6))*transpose(J)*v;
        omega1(i) = omega(4)
        plot3(x(i), y(i), z(i),'r.','MarkerSize',10)
        title('轨迹线');
        xlabel('x axis');
        ylabel('y axis');
        zlabel('z axis');
        grid on;
    else
        omega = inv(J)*v;
        omega1(i) = omega(4)
    end
end
legend('轨迹','奇异点')



for i=1:length(theta1)
    alpha = [theta1(i) theta2(i) theta3(i) theta4(i) theta(i) theta6(i)];
    J=jacob(alpha);
    omega = inv(J)*v;
    omega2(i) = omega(4);
end
    
figure
plot(theta, omega2,'linewidth',1.5)
hold on
plot(theta,omega1,'r.','MarkerSize',10)
axis([-3.2  -3.1 -50 30]) 
title('第四关节角速度');
xlabel('\theta_5（rad）');
ylabel('角速度（rad/s）');
legend('直接求逆','阻尼法求逆')
grid on

