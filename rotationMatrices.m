function R = rotationMatrices(theta)
    % Thiss function calculate the rotation matrices given an array of joint angles.
    % Each rotation matrix R_i defines a rotation from reference frame i to reference frame i-1
    %
    % Args:
    %       theta : Array (1xN) of joint angles
    % Return:
    %       R : Cell array (1xN) contaning rotation matrices (2x2) for each joint.
    
    for i = 1:length(theta)
        R{i} = [cos(theta(i)),    -sin(theta(i));
                sin(theta(i)),     cos(theta(i))];
    end
end