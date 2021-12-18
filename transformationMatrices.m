function T = transformationMatrices(R, D)
    % This function calculate the transformation matrices for the end-effector of link i
    % from its reference frame i to reference frame i-1.
    %
    % For link i, the transformation matrix for the end-effector from reference frame i to reference frame i-1 is defined as:
    %   T = ⎡ R  D ⎤
    %       ⎣ 0  1 ⎦
    %
    % Args:
    %       R : Cell array (1xN) for rotation matrices (2x2) R{i} (rotation from reference frame i to reference frame i-1)
    %       R : Cell array (1xN) for translation matrices (1x2) D{i} (translation from reference frame i to reference frame i-1)
    % Return:
    %       T : Cell array (1xN) contaning rotation matrices (2x2) for each end-effector.

    % --- Tranformation Matrix for each link on its reference frame ----
    for i = 1:length(R)
        % Create [R D] matrix
        T{i} = [R{i} D(:,i)];
        % Append [0 1] matrix
        T{i}(size(T{i})(1)+1,size(T{i})(2)) = 1;
    end

end