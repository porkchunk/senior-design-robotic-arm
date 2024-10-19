function [Geometric_Jacobian,T07] = FindingJacobianMatrixandTransformationMatrix(theta,distances)

%syms theta1 theta2 theta3 theta4 theta5 theta6
DISTANCE_LINK_1 = distances(1);
DISTANCE_LINK_2 = distances(2);
DISTANCE_LINK_3 = distances(3);
DISTANCE_LINK_4 = distances(4);
DISTANCE_LINK_5 = distances(5);
DISTANCE_LINK_6 = distances(6);
DISTANCE_LINK_7 = distances(7);
DISTANCE_LINK_8 = distances(8);

theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);
theta4 = theta(4);
theta5 = theta(5);
theta6 = theta(6);

T01 = Transform(0, 0, DISTANCE_LINK_1, theta1);
T12 = Transform(pi/2, 0, 0, theta2);
T23 = Transform(0, DISTANCE_LINK_2, 0, theta3);
T34 = Transform(0, DISTANCE_LINK_3, 0, theta4);
T45 = Transform(0, DISTANCE_LINK_4, 0, theta5);
T56 = Transform(-pi/2, DISTANCE_LINK_6, DISTANCE_LINK_5, theta6);
T67 = Transform(0, DISTANCE_LINK_7, -DISTANCE_LINK_8, 0);

T02 = double(T01*T12);
T24 = double(T23*T34);
T46 = double(T45*T56);
T04 = double(T02*T24);
T06 = double(T04*T46);
T07 = T06*T67;
T07 = double(T07);

T03 = double(T01*T12*T23);
T05 = double(T04*T45);

t1 = -pi/2;
t2 = pi/2;
t3 = -pi/2;
t4 = -pi/2;
t5 = pi/2;
t6 = 0;    


    c1 = cos(theta1);
    c2 = cos(theta2);
    c3 = cos(theta3);
    c4 = cos(theta4);
    c5 = cos(theta5);
    c6 = cos(theta6);
    s1 = sin(theta1);
    s2 = sin(theta2);
    s3 = sin(theta3);
    s4 = sin(theta4);
    s5 = sin(theta5);
    s6 = sin(theta6);

%variables = [DISTANCE_LINK_1,DISTANCE_LINK_2,DISTANCE_LINK_3,DISTANCE_LINK_4,DISTANCE_LINK_5,DISTANCE_LINK_6, DISTANCE_LINK_7, DISTANCE_LINK_8, theta1,theta2,theta3,theta4,theta5,theta6];
%values = [1.5,7.2,3,7,0.95,2.5,3.6,0,t1,t2,t3,t4,t5,t6];

%subs(T07,variables,values)

     J11 = (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)) - DISTANCE_LINK_7*(c1*s6 - c6*s5*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)) + c5*c6*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3))) - (c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) - c2*s1*(DISTANCE_LINK_2 + DISTANCE_LINK_3*c3) + DISTANCE_LINK_3*s1*s2*s3;
     J21 = (c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) - DISTANCE_LINK_7*(s1*s6 - c5*c6*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) + c6*s5*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4))) + c1*c2*(DISTANCE_LINK_2 + DISTANCE_LINK_3*c3) - DISTANCE_LINK_3*c1*s2*s3;
    J31 = 0;
    
     J12 = - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) - DISTANCE_LINK_7*(c5*c6*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) + c6*s5*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3))) - (c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) - c1*s2*(DISTANCE_LINK_2 + DISTANCE_LINK_3*c3) - DISTANCE_LINK_3*c1*c2*s3;
     J22 = - DISTANCE_LINK_7*(c6*s5*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) + c5*c6*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4))) - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) - (c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) - s1*s2*(DISTANCE_LINK_2 + DISTANCE_LINK_3*c3) - DISTANCE_LINK_3*c2*s1*s3;
     J32 = (c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) + c2*(DISTANCE_LINK_2 + DISTANCE_LINK_3*c3) + DISTANCE_LINK_7*(c5*c6*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3)) - c6*s5*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))) - (c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5) - DISTANCE_LINK_3*s2*s3;
     
     J13 = - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) - DISTANCE_LINK_7*(c5*c6*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) + c6*s5*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3))) - (c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) - DISTANCE_LINK_3*c1*c2*s3 - DISTANCE_LINK_3*c1*c3*s2;
     J23 = - DISTANCE_LINK_7*(c6*s5*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) + c5*c6*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4))) - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) - (c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) - DISTANCE_LINK_3*c2*s1*s3 - DISTANCE_LINK_3*c3*s1*s2;
     J33 = (c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) + DISTANCE_LINK_7*(c5*c6*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3)) - c6*s5*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))) - (c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5) + DISTANCE_LINK_3*c2*c3 - DISTANCE_LINK_3*s2*s3;
     
     J14 = - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) - DISTANCE_LINK_7*(c5*c6*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) + c6*s5*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3))) - (c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5);
     J24 = - DISTANCE_LINK_6*(c6*s5*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) + c5*c6*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4))) - (DISTANCE_LINK_4 - DISTANCE_LINK_5*s5)*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)) - DISTANCE_LINK_5*c5*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3));
     J34 = (c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) + DISTANCE_LINK_7*(c5*c6*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3)) - c6*s5*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))) - (c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5);
     
     J15 = - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) - (DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5)*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) - DISTANCE_LINK_7*(c5*c6*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) + c6*s5*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)));
     J25 = - DISTANCE_LINK_7*(c6*s5*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) + c5*c6*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4))) - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) - (DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5)*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4));
     J35 = DISTANCE_LINK_7*(c5*c6*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3)) - c6*s5*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))) - (c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5) + (c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5);
     
     J16 = -DISTANCE_LINK_7*(c6*s1 + c5*s6*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) - s5*s6*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)));
     J26 = DISTANCE_LINK_7*(c1*c6 - c5*s6*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) + s5*s6*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)));
     J36 = -DISTANCE_LINK_7*(c5*s6*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4)) + s5*s6*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3)));
     


J41 = T01(1,3);
J51 = T01(2,3);
J61 = T01(3,3);

J42 = T02(1,3); 
J52 = T02(2,3);
J62 = T02(3,3);

J43 = T03(1,3); 
J53 = T03(2,3);
J63 = T03(3,3);

J44 = T04(1,3); 
J54 = T04(2,3);
J64 = T04(3,3);

J45 = T05(1,3); 
J55 = T05(2,3);
J65 = T05(3,3);

J46 = T06(1,3); 
J56 = T06(2,3);
J66 = T06(3,3);
     
    %{ 
 Jacobian = [J11, J12, J13, J14, J15, J16
       ;J21, J22, J23, J24, J25, J26
       ;J31, J32, J33, J34, J35, J36
       ;J41, J42, J43, J44, J45, J46
       ;J51, J52, J53, J54, J55, J56];
    %}
pitch = -asin(T07(3,1));
roll = atan((T07(3,2)/cos(pitch))/(T07(3,3)/cos(pitch)));
yaw = atan((T07(2,1)/cos(pitch))/(T07(1,1)/cos(pitch)));

%subs(roll,variables,values)
%subs(pitch,variables,values)
%subs(yaw,variables,values)

Geometric_Jacobian = [J11, J12, J13, J14, J15, J16;
           J21, J22, J23, J24, J25, J26;
           J31, J32, J33, J34, J35, J36];

Geometric_Jacobian_Angles = [J41, J42, J43, J44, J45, J46;
                             J51, J52, J53, J54, J55, J56;
                             J61, J62, J63, J64, J65, J66];

Geometric_Jacobian_Angles_Calc = [
                             J51, J52, J53, J54, J55, J56;
                             J61, J62, J63, J64, J65, J66];

matrix_to_analytical = [1, sin(roll), sin(roll)*sin(pitch);
                        0, cos(roll),-cos(roll)*sin(pitch);
                        0, 0,cos(pitch)];
%{
matrix_to_analytical = [1, 0, sin(pitch);
                        0,cos(roll),-sin(roll)*cos(pitch);
                        0,sin(roll),cos(roll)*cos(pitch)];
%}
matrix_to_analytical = inv(matrix_to_analytical);


Analytical_Jacobian_Angles = matrix_to_analytical*Geometric_Jacobian_Angles;

Analytical_Jacobian = [Geometric_Jacobian;Analytical_Jacobian_Angles(2,:);Analytical_Jacobian_Angles(3,:)];
Geometric_Jacobian = [Geometric_Jacobian;Geometric_Jacobian_Angles];

end


