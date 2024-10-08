function [Tip] = ForwardKinematics(theta1, theta2, theta3, theta4, theta5, theta6)

alpha = [0,   pi/2,  0,  0,  0, -pi/2, 0];
a     = [0,   0,  5, 5, 5, 5, 5];
d     = [5,  0,   0,  0,  0, 0, 0];
Q     = [theta1, theta2, theta3, theta4, theta5, theta6, 0];

T01  = Transform(alpha(1), a(1), d(1), Q(1));
T12  = Transform(alpha(2), a(2), d(2), Q(2));
T23  = Transform(alpha(3), a(3), d(3), Q(3));
T34  = Transform(alpha(4), a(4), d(4), Q(4));
T45  = Transform(alpha(5), a(5), d(5), Q(5));
T56  = Transform(alpha(6), a(6), d(6), Q(6));
T67  = Transform(alpha(7), a(7), d(7), Q(7));

T07 = T01 * T12 * T23 * T34 * T45 * T56 * T67;

Tip = T07(1:3, 4);

end