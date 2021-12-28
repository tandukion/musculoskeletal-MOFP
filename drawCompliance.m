function drawCompliance(ax,X,C)
    % This function draws the compliance ellipse on the given end-effector position with the given compliance matrix.
    %
    % Args:
    %   - ax (Axes): Target axes to plot
    %   - X (N Cells of Arrays 3x1): Matrix of end-effector's 2D position on N joints. Third row is ones.
    %   - C (N Cells of Arrays NxN): Joint Compliance matrix for N joints


    % Create the end-effector position matrix in (2xN)
    for i = 1:length(X)
        ee_pos(:,i) = X{i}(1:2,:);
    end

    % Create new variables for special position to make it easier to understand
    ankle_pos = ee_pos(:,end-1);
    toe_pos = ee_pos(:,end);

    C_ankle = C(:,:,end-1);
    C_toe = C(:,:,end);

    % ====== Plot settings ======
    set (ax, 'title', 'Compliance Profile');
    set (ax, 'xlim', [-0.5 0.5]);

    % ====== Plot input ======
    % --- Draw robot link for all plots ---
    drawLink(ax, ee_pos);

    % --- Draw compliance eclipse ---
    % Create settings for the output force on polar axis
    com_scale = 10;

    % Draw compliance eclipse
    drawComplianceEllipse(ax, position=ankle_pos, C=C_ankle, scale=com_scale)
    drawComplianceEllipse(ax, position=toe_pos, C=C_toe, scale=com_scale)
end