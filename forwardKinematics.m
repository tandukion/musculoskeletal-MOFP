
function [X J] = forwardKinematics(Link, theta, debug=false)
    % This function calculates the forward kinematics given the link lengths and the joint angles.
    % Link and joint angles need to be in the same dimension.
    % This function calculates both the position and the Jacobian matrix of end-effectors from each link.
    % This function accepts both numeric and symbolic variables.
    % 
    % The main process of this function calculates the forward kinematics in symbolic variables.
    % The processes will be divided into:
    %   - converting numeric inputs to symbolic variables
    %   - calculation in symbolic variables
    %   - substitute symbolic to numeric outputs for numeric inputs
    %
    % Args:
    %   Link : Array (1xN) of link lengths
    %   theta : Array (1xN) of joint angles
    %   debug(optional): boolean flag to display the result in every process.
    %
    % Return:
    %   X: Cell arrays (1xN) of end-effector position in 2D (3x1)
    %   J: Cell arrays (1xN) of Jacobian Matrices
    
    % ====== PRE-PROCESS input ======
    % Pre-processing is needed to calculate the partial derivatives of end-effector position for Jacobian

    % Force the input to have the same size (not in transpose)
    % Make sure the matrix size same
    if not(size(theta) == size(Link))
        theta = theta.';
    end

    % Confirm that the inputs are symbolic variables.
    % Partial derivatives need to process symbolic functions.
    if (isa(Link, 'sym'))
        sym_link = Link;
    else
        % Need to create symbolic variables to process partial derivatives
        for i = 1:length(Link)
            sym_link(i) = sym(strcat('L', num2str(i)));
        end
    end
    if (isa(theta, 'sym'))
        sym_theta = theta;
    else
        % Need to create symbolic variables to process partial derivatives
        for i = 1:length(theta)
            sym_theta(i) = sym(strcat('theta', num2str(i)));
        end
    end
    
    % ====== MAIN PROCESS ======
    % --- Rotation of end-effectors on its reference frame ---
    % Calculate the rotation matrix for each joint
    R = rotationMatrices(sym_theta);

    % Debugging
    if (debug == true)
        celldisp(R)
    end

    % --- Translation of of end-effectors on its reference frame ---
    % Create new link array to be in 2D space: [x y]'
    % By default, joint i will be placed on the x axis of joint i-1,
    % So any end-effector of the link i will have position [Li 0]'
    L = sym_link;
    L(2,1) = 0; % Add y value

    % Create translation matrix for each joint
    % Each joint i origin is placed L_i-1 away from the joint i-1
    D = [[0 0].' L(:,1:end-1)];

    % --- Tranformation Matrices ---
    % Calculate the tranformation matrix for the end-effector of each link
    T_i = transformationMatrices(R,D);
    
    % Debugging
    if (debug == true)
        celldisp(T_i)
    end

    % --- Tranformation Matrix for each joint to joint 0 reference frame ----
    % For link i, the transformation matrix for the end-effector from reference frame i to reference frame 0 is defined as:
    % T = T_1 * T_2 * ... * T_i
    for i = 1:length(T_i)
        T{i} = eye(3);
        for j = 1:i
            T{i} = T{i} * T_i{j};
        end
    end

    % Debugging
    if (debug == true)
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

    % --- Linear part of the Jacobian ---
    % In general, jacobian matrix is defined as:
    %   J = ∂X/∂theta

    % Calculate partial derivatives for each end-effector
    for i = 1:length(sym_theta)
        % Create array of joint angles affecting the current end-effector
        for j = 1:i
            theta_i(i) = sym_theta(j);
        end
        % Calculate partial derivatives of the end-effector position with respect to the affecting joint angles
        % NOTE: gradient() can't handle matrix.
        for j = 1:length(X{i})
            for k = 1:length(theta_i)
                A(j,k) = gradient(X{i}(j),theta_i(k));
            end
        end
        J{i} = A;
    end
    
    % ====== POST-PROCESS output ======
    % Return numerical values for numerical input variables.
    if (~(isa(Link, 'sym')) && ~(isa(theta, 'sym')))
        % Substituting float value will trigger warning from sym.
        % Need to deactivate the warning for the rest of process by triggering the warning first.
        % NOTE: handling the warning inside a loop will not handle the other iteration.
        % Deactivate the warning by ID
        msg = 'passing floating-point values to sym is dangerous, see "help sym"';
        id = 'OctSymPy:sym:rationalapprox';
        S = warning ('off', id);
        disp(strcat('Deactivate warning: ',msg))

        for i = 1:length(X)
            % End-effector position
            X{i} = double(subs(X{i},[sym_link sym_theta],[Link theta]));

            % Jacobian
            J{i} = double(subs(J{i},[sym_link sym_theta],[Link theta]));
        end
    end
end