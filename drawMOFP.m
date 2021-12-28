function drawMOFP(ax,X,Q)
    % This function draws the Maximum Output Force Profile on the given end-effector position and end-effector forces
    %
    % Args:
    %   - ax (Axes): Target axes to plot
    %   - X (N Cells of Arrays 3x1): Matrix of end-effector's 2D position on N joints. Third row is ones.
    %   - Q (N Cells of Arrays 3xM): Matrix of M vectors on N joints. Third row is zeros.


    % Create the end-effector position matrix in (2xN)
    for i = 1:length(X)
        ee_pos(:,i) = X{i}(1:2,:);
    end

    % Create new variables for special position to make it easier to understand
    ankle_pos = ee_pos(:,end-1);
    toe_pos = ee_pos(:,end);

    Q_ankle = Q(:,:,end-1);
    Q_toe = Q(:,:,end);

    % ====== Plot settings ======
    set (ax, 'title', 'Maximum Output Force Profile');

    % ====== Plot input ======
    % --- Draw robot link for all plots ---
    drawLink(ax, ee_pos);
    hold on;

    % --- Draw output force ---
    % Create settings for the output force on polar axis
    tick_space = 10; % tick space for polar axis
    axis_scale = 0.1; % axis scale to be fit into the robot link plot
    axis_scale = axis_scale / tick_space;

    % Create auto scaling for force values
    for i = 1:size(Q_toe,1)
        for j = 1:size(Q_toe,2)
            power(i,j) = ceil(log10(abs(Q_toe(i,j)))-1);
        end
    end
    value_power = max(max(power));  % The max power for the current values
    value_scale = axis_scale * 10^(-max(max(value_power)));  % The value scale on the force plot

    % Draw polar scale (only need to draw one at toe)
    drawPolarAxis(ax, position=toe_pos, value_power=value_power, axis_scale=axis_scale, tick_space=tick_space);
    hold on;

    % Draw individual output force vectors on ankle
    drawVector(ax, position=ankle_pos, F=Q_ankle, scale=value_scale);
    hold on;

    % Draw individual output force vectors on toe
    drawVector(ax, position=toe_pos, F=Q_toe, scale=value_scale);
    hold on;

    % ====== Plot analysis output ======
    % Compute the output force distribution
    % ankle
    V_ankle = outputForceDistribution(V=Q_ankle);
    % toe
    V_toe = outputForceDistribution(V=Q_toe);

    % Draw output force distribution
    drawOutputForceDistribution(ax, position=ankle_pos, V=V_ankle, scale=value_scale);
    hold on;
    drawOutputForceDistribution(ax, position=toe_pos, V=V_toe, scale=value_scale);
    hold on;

end