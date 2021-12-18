
function X = forwardKinematics(Link, theta, debug=false)
    % The function calculate the position of each link position in 2D based on the joint angles.
    % Link and joint angles need to be in the same dimension,
    % Args:
    %   Link : Array (1xN) of link lengths
    %   theta : Array (1xN) of joint angles
    % Return:
    %   X: Matrix (2xN) of joint position in 2D

    % --- Rotation of end-effectors on its reference frame ---
    % Calculate the rotation matrix for each joint
    R = rotationMatrices(theta);

    % Debugging
    if debug == true
        celldisp(R)
    end

    % --- Translation of of end-effectors on its reference frame ---
    % Create new link array to be in 2D space: [x y]'
    % By default, joint i will be placed on the x axis of joint i-1,
    % So any end-effector of the link i will have position [Li 0]'
    L = Link;
    L(2,1) = 0; % Add y value

    % Create translation matrix for each joint
    % Each joint i origin is placed L_i-1 away from the joint i-1
    D = [[0 0].' L(:,1:end-1)];

    % --- Tranformation Matrices ---
    % Calculate the tranformation matrix for the end-effector of each link
    T_i = transformationMatrices(R,D);
    
    % Debugging
    if debug == true
        celldisp(T_i)
    end

    % --- Tranformation Matrix for each joint to joint 0 reference frame ----
    % For link i, the transformation matrix for the end-effector from reference frame i to reference frame 0 is defined as:
    % T = T_1 * T_2 * ... * T_i
    for i = 1:length(T_i)
        % T{i} = T_i{1};
        % for j = 2:i
        %     T{i} = T{i} * T_i{j};
        % end
        T{i} = eye(3);
        for j = 1:i
            T{i} = T{i} * T_i{j};
        end
    end

    % Debugging
    if debug == true
        celldisp(T)
    end

    % --- Position of end-effectors ---
    % The position of the end-effector of link i is defined as
    %   X = T * X_0,    where X_0 is the initial position vector
    %
    % The initial position will be based on the link array in 2D space, with addition of value 1 to make it as a vector of (3x1)
    
    % Create Initial position vectors
    L(3,:) = ones(1,size(L,2));

    % Calculate the forward kinematics
    for i = 1:length(T)
        X{i} = T{i} * L(:,i); 
    end

end