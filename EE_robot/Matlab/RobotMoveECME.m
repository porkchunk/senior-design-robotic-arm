function [x_initial] = RobotMoveECME(x,y,z,pitch,yaw) 

distances = [1.5,7.2,3,7,0.95,2.5,3.6,0];
initial_angles = [0,pi/2,-pi/2,-pi/2,pi/2,0]';
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

x_final = [x,y,z]';
x_initial = [transform_matrix(1,4),transform_matrix(2,4),transform_matrix(3,4)]';

[jacobian,~] = FindingJacobianMatrixandTransformationMatrix(initial_angles, distances);

time_step = 0.0009;
counter = 0;
lambda = 1e-2;
x_difference = 1;

while(norm(x_difference) >= 0.1)
    x_difference = x_final - x_initial;

    r_error = final_rotation_matrix*transpose(initial_rotation_matrix);
    w_difference = (1/2)*[r_error(3,2) - r_error(2,3); r_error(1,3) - r_error(3,1); r_error(2,1) - r_error(1,2)];
   
    %rotation_angle = acos((trace(r_error) - 1)/2);

    %rotation_axis = (1/(2*sin(rotation_angle)))*[r_error(3,2) - r_error(2,3); r_error(1,3) - r_error(3,1); r_error(2,1) - r_error(1,2)];
    %w_difference = rotation_angle*rotation_axis;

    total_difference = [x_difference; w_difference];

    velocity = (total_difference/norm(total_difference))*3;
    
    jacobian_inverse =  jacobian' * inv((jacobian*jacobian' + lambda*eye(6)));

    if counter < 1
       % disp(w_difference)
    end

    delta_angle = jacobian_inverse*velocity;
    
    new_angle = initial_angles + delta_angle*time_step;
    initial_angles = new_angle;

    [~,transform_matrix] = FindingJacobianMatrixandTransformationMatrix(new_angle, distances);
    initial_rotation_matrix = transform_matrix(1:3,1:3);

    x_initial = [transform_matrix(1,4),transform_matrix(2,4),transform_matrix(3,4)]';

    if counter < 2000
        %disp(x_initial)
    end

    [jacobian,~] = FindingJacobianMatrixandTransformationMatrix(new_angle, distances);
    counter = counter + 1;
    if counter>200000
        break;
    end
end
pitch = -asin(transform_matrix(3,1));
yaw = atan((transform_matrix(2,1)/cos(pitch))/(transform_matrix(1,1)/cos(pitch)));
disp(new_angle)
x_initial = [transform_matrix(1,4),transform_matrix(2,4),transform_matrix(3,4),pitch,yaw]';
end

