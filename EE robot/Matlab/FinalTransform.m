function [eulerangles] = FinalTransform(t1,t2,t3,t4)
d1 = 9.1;
d2 = 10.125;
d3 = 8.3125;
d4 = 16.125;

s1 = sin(t1);
s2 = sin(t2);
s3 = sin(t3);
s4 = sin(t4);
c1 = cos(t1);
c2 = cos(t2);
c3 = cos(t3);
c4 = cos(t4);

T11 = [c1*c2*c3*c4 - c1*c2*s3*s4 - c1*c4*s2*s3 - c1*c3*s2*s4];
T12 = [-c1*c2*c3*s4 - s3*c1*c2*c4 + c1*s2*s3*s4 - c1*c3*c4*s2];
T13 = [s1];
T14 = [(c1*c2)*(d4*c3*c4 - d4*s3*s4 + d3*c3 + d2) + (-c1*s2)*(d4*s3*c4 + d4*c3*s4 + d3*s3)];

T21 = [(s1*c2)*(c3*c4 - s3*s4) - (s1*s2)*(s3*c4 + c3*s4)];
T22 = [(s1*c2)*(-c3*s4 - s3*c4) - (s1*s2)*(-s3*s4 + c3*c4)];
T23 = [-c1];
T24 = [(s1*c2)*(d4*c3*c4 - d4*s3*s4 + d3*c3 + d2) - (s1*s2)*(d4*s3*c4 + d4*c3*s4 + d3*s3)];

T31 = [s2*(c3*c4 - s3*s4) + c2*(s3*c4 + c3*s4)];
T32 = [s2*(-c3*s4 - s3*c4) + c2*(-s3*s4 + c3*c4)];
T33 = 0;
T34 = [s2*(d4*c3*c4 - d4*s3*s4 + d3*c3 + d2) + c2*(d4*s3*c4 + d4*c3*s4 + d3*s3) + d1];

T41 = 0;
T42 = 0;
T43 = 0;
T44 = 1;

T = [T11, T12, T13, T14; T21, T22, T23, T24; T31, T32, T33, T34; T41, T42, T43, T44];

R = T;
R(4,:) = [];
R(:,4) = [];
eulerangles = rotm2eul(R);

end

function [T] = Transform(alpha, a, d, theta)

T1 = [cos(theta), -sin(theta), 0 , a];
T2 = [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha)];
T3 = [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha)];
T4 = [0, 0, 0, 1];

T = [T1; T2; T3; T4];
end

function [Tip] = RRR(W, TIP)
 
alpha = [0,   pi/2,  0,  0,  0];
a     = [0,   0,  10, 10, 10];
d     = [20,  0,   0,  0,  0];
Q     = [W(1), W(2), W(3), W(4), 0];
 
T01a = Transform(alpha(1), a(1), 0,    Q(1));
T01  = Transform(alpha(1), a(1), d(1), Q(1));
T12a = Transform(alpha(2), a(2), 0,    Q(2));
T12  = Transform(alpha(2), a(2), d(2), Q(2));
T23a = Transform(alpha(3), a(3), 0,    Q(3));
T23  = Transform(alpha(3), a(3), d(3), Q(3));
T34a = Transform(alpha(4), a(4), 0,    Q(4));
T34  = Transform(alpha(4), a(4), d(4), Q(4));
T45a = Transform(alpha(5), a(5), 0,    Q(5));
T45  = Transform(alpha(5), a(5), d(5), Q(5));


Origin = [0;0;0;1];

P0 = Origin;
P1 = T01a * Origin;
P2 = T01 * Origin;
P3 = T01*T12a * Origin;
P4 = T01*T12 * Origin;
P5 = T01*T12*T23a * Origin;
P6 = T01*T12*T23 * Origin;
P7 = T01*T12*T23*T34a * Origin;
P8 = T01*T12*T23*T34 * Origin;
P9 = T01*T12*T23*T34*T45a * Origin;
P10 = T01*T12*T23*T34*T45 * Origin;

X = [P0(1); P1(1); P2(1); P3(1); P4(1); P5(1); P6(1); P7(1); P8(1); P9(1); P10(1)];
Y = [P0(2); P1(2); P2(2); P3(2); P4(2); P5(2); P6(2); P7(2); P8(2); P9(2); P10(2)];
Z = [P0(3); P1(3); P2(3); P3(3); P4(3); P5(3); P6(3); P7(3); P8(3); P9(3); P10(3)];

Tip = T01*T12*T23*T34*T45*[0;0;0;1];
Tipx = T01*T12*T23*T34*T45*[10;0;0;1];
Tipy = T01*T12*T23*T34*T45*[0;10;0;1];
Tipz = T01*T12*T23*T34*T45*[0;0;10;1];

% draw the axis
clf;
plot([0, -70],[0, -70], 'r-');
hold on
plot([0, 100],[0, 0], 'r-');
plot([0, 0],  [0, 100], 'r-');

% draw the robot

plot(Y - 0.707*X,  Z - 0.707*X, 'b.-');
plot(TIP(2,:) - 0.707*TIP(1,:),  TIP(3,:) - 0.707*TIP(1,:), 'g');

% draw the tip axis
plot([Tip(2)-0.707*Tip(1), Tipx(2)-0.707*Tipx(1)],[Tip(3)-0.707*Tip(1),Tipx(3)-0.707*Tipx(1)],'g-');
plot([Tip(2)-0.707*Tip(1), Tipy(2)-0.707*Tipy(1)],[Tip(3)-0.707*Tip(1),Tipy(3)-0.707*Tipy(1)],'g-');
plot([Tip(2)-0.707*Tip(1), Tipz(2)-0.707*Tipz(1)],[Tip(3)-0.707*Tip(1),Tipz(3)-0.707*Tipz(1)],'g-');
 
xlim([-150,150]);
ylim([-150,150]);
pause(0.01);


end

