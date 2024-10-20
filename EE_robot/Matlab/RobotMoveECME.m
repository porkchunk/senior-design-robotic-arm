function [x_initial] = RobotMoveECME(x,y,z,pitch,yaw) 

distances = [1.5,7.2,3,7,0.95,2.5,3.6,0];
initial_angles = [-pi/4,pi/2,-pi/2,-pi/2,pi/2,0]';
roll=0;

T1 = [cos(yaw), -sin(yaw), 0;
      sin(yaw), cos(yaw), 0;
      0, 0, 1];

T2 = [cos(pitch), 0 , sin(pitch);
      0, 1, 0;
      -sin(pitch), 0, cos(pitch)];

T3 = [1,0,0;
      0,cos(roll),-sin(roll);
      0,sin(roll),cos(roll)];

[~,transform_matrix] = FindingJacobianMatrixandTransformationMatrix(initial_angles, distances);

final_rotation_matrix = T1*T2*T3;
initial_rotation_matrix = transform_matrix(1:3,1:3);
%disp(final_rotation_matrix);

q_initial = rotm2quat(initial_rotation_matrix);
q_final = rotm2quat(final_rotation_matrix);

x_final = [x,y,z]';
x_initial = [transform_matrix(1,4),transform_matrix(2,4),transform_matrix(3,4)]';

[jacobian,~] = FindingJacobianMatrixandTransformationMatrix(initial_angles, distances);
%disp(jacobian);
time_step = 0.005;
counter = 0;
lambda = 2.8;
x_difference = 1;

while(norm(x_difference) >= 0.05)
    x_difference = x_final - x_initial;

    r_error1 = final_rotation_matrix*transpose(initial_rotation_matrix);
    r_error2 = transpose(final_rotation_matrix)*initial_rotation_matrix;
    r_error = r_error1 - r_error2;

    w_difference = (2)*[r_error(3,2), r_error(1,3), r_error(2,1)]';

    total_difference = [x_difference; w_difference];

    velocity = (total_difference/norm(total_difference));
    
    jacobian_inverse =  jacobian' * inv((jacobian*jacobian' + lambda*eye(6)));

    delta_angle = jacobian_inverse*velocity;
    
    new_angle = initial_angles + delta_angle*time_step;
    initial_angles = new_angle;

    [~,transform_matrix] = FindingJacobianMatrixandTransformationMatrix(new_angle, distances);
    initial_rotation_matrix = transform_matrix(1:3,1:3);

    pitch = -asin(transform_matrix(3,1));
    yaw = atan(transform_matrix(2,1)/cos(pitch)/transform_matrix(1,1)/cos(pitch));

    x_initial = [transform_matrix(1,4),transform_matrix(2,4),transform_matrix(3,4)]';
    position = [x_initial;pitch;yaw];

    if counter < 3000
        %disp(position)

    end

    [jacobian,~] = FindingJacobianMatrixandTransformationMatrix(new_angle, distances);
    counter = counter + 1;
    if counter>11000
        break;
    end
end
pitch = -asin(transform_matrix(3,1));
yaw = atan(transform_matrix(2,1)/cos(pitch)/transform_matrix(1,1)/cos(pitch));
disp(transform_matrix);
disp(new_angle);
disp(counter);
x_initial = [transform_matrix(1,4),transform_matrix(2,4),transform_matrix(3,4),pitch,yaw]';
end
