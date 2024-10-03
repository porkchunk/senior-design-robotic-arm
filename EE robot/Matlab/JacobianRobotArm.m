function [J] = JacobianRobotArm(t1,t2,t3,t4)
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


J11 = (-s1*c2)*(d4*c3*c4 - d4*s3*s4 + d3*c3 + d2) + (s1*s2)*(d4*s3*c4 + d4*c3*s4 + d3*s3);

J12 = (-s2*c1)*(d4*c3*c4 - d4*s3*s4 + d3*c3 + d2) - (c1*c2)*(d4*s3*c4 + d4*c3*s4 + d3*s3);

J13 = (c1*c2)*(-d4*s3*c4 - d4*c3*s4 - d3*s3) + (-c1*s2)*(d4*c3*c4 - d4*s3*s4 + d3*c3);

J14 = (c1*c2)*(-d4*c3*s4 - d4*s3*c4) + (-c1*s2)*(-d4*s3*s4 + d4*c3*c4);


J21 = (c1*c2)*(d4*c3*c4 - d4*s3*s4 + d3*c3 + d2) - (c1*s2)*(d4*s3*c4 + d4*c3*s4 + d3*s3);

J22 = (-s1*s2)*(d4*c3*c4 - d4*s3*s4 + d3*c3 + d2) - (s1*c2)*(d4*s3*c4 + d4*c3*s4 + d3*s3);

J23 = (s1*c2)*(-d4*s3*c4 - d4*c3*s4 - d3*s3) - (s1*s2)*(d4*c3*c4 - d4*s3*s4 + d3*c3);

J24 = (s1*c2)*(-d4*c3*s4 - d4*s3*c4) - (s1*s2)*(-d4*s3*s4 + d4*c3*c4);

J31 = 0;

J32 = c2*(d4*c3*c4 - d4*s3*s4 + d3*c3 + d2) - s2*(d4*s3*c4 + d4*c3*s4 + d3*s3);

J33 = s2*(-d4*s3*c4 - d4*c3*s4 - d3*s3) + c2*(d4*c3*c4 - d4*s3*s4 + d3*c3);

J34 = s2*(-d4*c3*s4 - d4*s3*c4) + c2*(-d4*s3*s4 + d4*c3*c4);


  J = [J11, J12, J13, J14; J21, J22, J23, J24; J31, J32, J33, J34; 0, -c1, -c1, - c1];% 0, s1, s1, s1]; %0, -c1, -c1, - c1; 1, 0, 0, 0 ];

end
