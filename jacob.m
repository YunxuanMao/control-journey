function [J] = jacob(theta)
a2 = -0.425;
a3 = -0.3922;

d1 = 0.1625;
d2 = 0;
d3 = 0;
d4 = 0.1333;
d5 = 0.0997;
d6 = 0.0996;
theta1=theta(1);
theta2=theta(2);
theta3=theta(3);
theta4=theta(4);
theta5=theta(5);
theta6=theta(6);

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
end

