function [T] = Transform(alpha, a, d, theta)

T1 = [cos(theta), -sin(theta), 0 , a];
T2 = [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha)];
T3 = [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha)];
T4 = [0, 0, 0, 1];

T = [T1; T2; T3; T4];
end

