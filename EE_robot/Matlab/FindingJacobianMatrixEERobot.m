syms theta1 theta2 theta3 theta4
syms DISTANCE_LINK_1 DISTANCE_LINK_2 DISTANCE_LINK_3 DISTANCE_LINK_4

T01 = Transform(0, 0, DISTANCE_LINK_1, theta1);
T12 = Transform(sym(pi/2), 0, 0, theta2);
T23 = Transform(0, DISTANCE_LINK_2, 0, theta3);
T34 = Transform(0, DISTANCE_LINK_3, 0, theta4);
T45 = Transform(0, DISTANCE_LINK_4, 0, 0);

T02 = T01*T12;
T24 = T23*T34;
T46 = T45*T56;
T04 = T02*T24;
T03 = T01*T12*T23;
T05 = T04*T45;


variables = [DISTANCE_LINK_1,DISTANCE_LINK_2,DISTANCE_LINK_3,DISTANCE_LINK_4,theta1,theta2,theta3,theta4];
values = [1,1,1,1,0,0,0,0];

subs(T05,variables,values)

% J11 = diff(T05(1,4),theta1);
% J21 = diff(T05(2,4),theta1);
% J31 = diff(T05(3,4),theta1);
% 
% J12 = diff(T05(1,4),theta2);
% J22 = diff(T05(2,4),theta2);
% J32 = diff(T05(3,4),theta2);
% 
% J13 = diff(T07(1,4),theta3);
% J23 = diff(T07(2,4),theta3);
% J33 = diff(T07(3,4),theta3);
% 
% J14 = diff(T07(1,4),theta4);
% J24 = diff(T07(2,4),theta4);
% J34 = diff(T07(3,4),theta4);
% 
% J15 = diff(T07(1,4),theta5);
% J25 = diff(T07(2,4),theta5);
% J35 = diff(T07(3,4),theta5);
% 
% J16 = diff(T07(1,4),theta6);
% J26 = diff(T07(2,4),theta6);
% J36 = diff(T07(3,4),theta6);
% 
% J41 = T01(1,3);
% J51 = T01(2,3);
% J61 = T01(3,3);
% 
% J42 = T02(1,3); 
% J52 = T02(2,3);
% J62 = T02(3,3);
% 
% J43 = T03(1,3); 
% J53 = T03(2,3);
% J63 = T03(3,3);
% 
% J44 = T04(1,3); 
% J54 = T04(2,3);
% J64 = T04(3,3);
% 
% J45 = T05(1,3); 
% J55 = T05(2,3);
% J65 = T05(3,3);
% 
% J46 = T06(1,3); 
% J56 = T06(2,3);
% J66 = T06(3,3);
% 
% Jacobian = [J11, J12, J13, J14, J15, J16
%            ;J21, J22, J23, J24, J25, J26
%            ;J31, J32, J33, J34, J35, J36
%            ;J41, J42, J43, J44, J45, J46
%            ;J51, J52, J53, J54, J55, J56
%            ;J61, J62, J63, J64, J65, J66];
% 
% subs(Jacobian, variables, values)




