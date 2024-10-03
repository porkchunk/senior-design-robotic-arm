function [angles] = RobotMoveV2(x, y, z, pitch)

% Desired end effector position and pitch
x_final = [x; y; z; pitch];
initial_angles = [0; pi/2; -pi/2; 0];

% Compute the initial orientation quaternion
final_transform = FinalTransform(initial_angles(1), initial_angles(2), initial_angles(3), initial_angles(4));
initial_orientation = rotm2quat(final_transform(1:3, 1:3));
initial_quaternion = [initial_orientation(2:4)'; initial_orientation(1)];

% Initialize variables for the loop
time_step = 0.01;
x_initial = RRR(initial_angles, zeros(4, 1));
x_initial(4) = initial_quaternion(4); % Use the scalar part of the quaternion as the pitch value

while true
    jacobian = JacobianRobotArm(initial_angles(1), initial_angles(2), initial_angles(3), initial_angles(4));
    
    % Compute the error in position and orientation
    x_difference = x_final - x_initial;
    
    % Include quaternion error
    final_quaternion = angle2quat(0, pitch, 0, 'YXZ');
    quaternion_difference = quatmultiply(final_quaternion, quatinv(initial_quaternion.'));
    
    % Normalize the error to get the direction for position and orientation
    position_error_normalized = x_difference(1:3) / norm(x_difference(1:3));
    orientation_error_normalized = quaternion_difference(1:3) / norm(quaternion_difference(1:3));
    
    % Compute the velocity including orientation error
    velocity = [position_error_normalized * 8; orientation_error_normalized * 8];
    
    % Compute the inverse Jacobian and the change in joint angles
    jacobian_inverse = pinv(jacobian);
    delta_angles = jacobian_inverse * velocity * time_step;
    
    % Update the joint angles
    initial_angles = initial_angles + delta_angles(1:4);
    
    % Pause for visualization or real-time control
    pause(time_step);
    
    % Update the forward kinematics and the Jacobian
    final_transform = FinalTransform(initial_angles(1), initial_angles(2), initial_angles(3), initial_angles(4));
    x_initial = RRR(initial_angles, zeros(4, 1));
    x_initial(4) = final_transform(1, 2); % Update the pitch value
    
    % Update the quaternion representation of orientation
    initial_orientation = rotm2quat(final_transform(1:3, 1:3));
    initial_quaternion = [initial_orientation(2:4)'; initial_orientation(1)];
    
    % Check for convergence
    if norm(x_difference) < 0.01 && norm(quaternion_difference) < 0.01
        break;
    end
end

angles = initial_angles;
end