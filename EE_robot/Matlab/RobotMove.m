function [x_initial] = RobotMove(x,y,z,pitch) 

x_final = [x,y,z,pitch]';
initial_angle = [0,pi/2,-pi/2,0]';

finaltransform = FinalTransform(0,pi/2,-pi/2,0);
initial_pitch = finaltransform(1,2);

jacobian = JacobianRobotArm(0,pi/2,-pi/2,0);
x_initial = [RRR([0,pi/2,-pi/2,0],zeros(4,1));initial_pitch];
time_step = 0.01;
counter = 0;
x_difference = 1;

while(norm(x_difference) >= 0.01)
    x_difference = x_final - x_initial;
   
    velocity = (x_difference/norm(x_difference))*8;
   
    jacobian_inverse = pinv(jacobian);
    
    delta_angle = jacobian_inverse*velocity;
   
    new_angle = initial_angle + delta_angle*time_step;
    initial_angle = new_angle;

    pause(0.01);
    finaltransform = FinalTransform(new_angle(1,1),new_angle(2,1),new_angle(3,1),new_angle(4,1));
    x_initial = [RRR([new_angle]',zeros(4,1));finaltransform(1,2)];
    jacobian = JacobianRobotArm(new_angle(1,1),new_angle(2,1),new_angle(3,1),new_angle(4,1));
    counter = counter + 1;

end

angles = [new_angle(1,1),new_angle(2,1),new_angle(3,1),new_angle(4,1)];
end

