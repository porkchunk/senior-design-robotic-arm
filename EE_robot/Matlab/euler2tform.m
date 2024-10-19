
%roll = 0;
%pitch = -pi/4;
%yaw = pi/2;

roll = 0;
pitch = pi/3;
yaw = pi/4;

T1 = [cos(yaw), -sin(yaw), 0;
      sin(yaw), cos(yaw), 0;
      0, 0, 1];

T2 = [cos(pitch), 0 , sin(pitch);
      0, 1, 0;
      -sin(pitch), 0, cos(pitch)];

T3 = [1,0,0;
      0,cos(roll),-sin(roll);
      0,sin(roll),cos(roll)];


T = [cos(roll)*cos(yaw), -cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw), sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);
     cos(pitch)*sin(yaw), cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw);
     -sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch)];

Othertransform = T1*T2*T3