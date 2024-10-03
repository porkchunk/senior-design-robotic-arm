function [Tip] = RRR(W, TIP)
 
alpha = [0,   pi/2,  0,  0,  0, -pi/2, 0];
a     = [0,   0,  10.3, 9, 16.1, 7, 7];
d     = [9.1,  0,   0,  0,  0, 0, 0];
Q     = [W(1), W(2), W(3), W(4), W(5), W(6), 0];
 
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
T56a = Transform(alpha(6), a(6), 0,    Q(6));
T56  = Transform(alpha(6), a(6), d(6), Q(6));
T67a = Transform(alpha(7), a(7), 0,    Q(7));
T67  = Transform(alpha(7), a(7), d(7),  Q(7));

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
P11 = T01*T12*T23*T34*T45*T56a * Origin;
P12 = T01*T12*T23*T34*T45*T56 * Origin;
P13 = T01*T12*T23*T34*T45*T56*T67a * Origin;
P14 = T01*T12*T23*T34*T45*T56*T67 * Origin;

X = [P0(1); P1(1); P2(1); P3(1); P4(1); P5(1); P6(1); P7(1); P8(1); P9(1); P10(1); P11(1); P12(1); P13(1); P14(1)];
Y = [P0(2); P1(2); P2(2); P3(2); P4(2); P5(2); P6(2); P7(2); P8(2); P9(2); P10(2); P11(2); P12(2); P13(2); P14(2)];
Z = [P0(3); P1(3); P2(3); P3(3); P4(3); P5(3); P6(3); P7(3); P8(3); P9(3); P10(3); P11(3); P12(3); P13(3); P14(3)];

T07 = T01*T12*T23*T34*T45*T56*T67;

R = T07;
R(4,:) = [];
R(:,4) = [];
%eulerangles = rotm2eul(R);
Tip = [T07(1,4), T07(2,4), T07(3,4)]';  
%Tip = T01*T12*T23*T34*T45*[0;0;0;1];
Tipx = T01*T12*T23*T34*T45*[10;0;0;1];
Tipy = T01*T12*T23*T34*T45*[0;10;0;1];
Tipz = T01*T12*T23*T34*T45*[0;0;10;1];

%draw the axis
clf;
plot([0, -70],[0, -70], 'r-');
hold on
plot([0, 100],[0, 0], 'g-');
plot([0, 0],  [0, 100], 'b-');

%draw the robot

plot(Y - 0.707*X,  Z - 0.707*X, 'c.-');
plot(TIP(2,:) - 0.707*TIP(1,:),  TIP(3,:) - 0.707*TIP(1,:), 'g');

% draw the tip axis
% plot([Tip(2)-0.707*Tip(1), Tipx(2)-0.707*Tipx(1)],[Tip(3)-0.707*Tip(1),Tipx(3)-0.707*Tipx(1)],'r-');
% plot([Tip(2)-0.707*Tip(1), Tipy(2)-0.707*Tipy(1)],[Tip(3)-0.707*Tip(1),Tipy(3)-0.707*Tipy(1)],'g-');
% plot([Tip(2)-0.707*Tip(1), Tipz(2)-0.707*Tipz(1)],[Tip(3)-0.707*Tip(1),Tipz(3)-0.707*Tipz(1)],'b-');

xlim([-150,150]);
ylim([-150,150]);
pause(0.01);

end