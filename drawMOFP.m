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

    % Compute the output force distribution
    % ankle
    MOFP_ankle = outputForceDistribution(V=Q_ankle)
    % toe
    MOFP_toe = outputForceDistribution(V=Q_toe)

    % ====== Plot settings ======
    set (ax, 'title', 'Maximum Output Force Profile');

    % Set axis to be square with 1.0 length
    min_YLim = toe_pos(2,1) - 0.3;
    if min_YLim > -0.6
        min_YLim = -0.6;
    end
    YLim = [min_YLim min_YLim+1.0];
    set (ax, 'YLim', YLim);

    if (toe_pos(1,1) >= 0)
        max_XLim = toe_pos(1,1) + 0.3;
        if max_XLim < 0.5
            max_XLim = 0.5;
        end
        XLim = [max_XLim-1.0  max_XLim];
    else
        min_XLim = toe_pos(1,1) - 0.3;
        XLim = [min_XLim  min_XLim+1.0];
    end
    set (ax, 'XLim', XLim);

    % ====== Plot input ======
    % --- Draw robot link for all plots ---
    drawLink(ax, ee_pos);
    hold on;

    % --- Create auto scaling for force plot ---
    % Target maximum size of the polygon
    target_max_size = 0.2;

    % Get the maximum vector length of the polygon
    maxV = max(max(abs(MOFP_toe)));

    % Calculate the value scale for the target size
    value_scale = target_max_size / maxV;

    % Round the value scale
    power = floor(log10(value_scale));  % Get the powers of ten
    value_scale = floor(value_scale/10^power) * 10^power;  % Round to only one digit

    % ====== Plot analysis output ======
    % Create settings for the output force on polar axis
    tick_space = 1;     % tick space for polar axis
    axis_scale = 0.1;   % axis scale to be fit into the robot link plot

    % Draw polar scale (only need to draw one at toe)
    drawPolarAxis(ax, position=toe_pos, value_scale=value_scale, axis_scale=axis_scale, tick_space=tick_space);
    hold on;

    % Draw individual output force vectors on ankle
    drawVector(ax, position=ankle_pos, F=Q_ankle, scale=value_scale);
    hold on;

    % Draw individual output force vectors on toe
    drawVector(ax, position=toe_pos, F=Q_toe, scale=value_scale);
    hold on;

    % Draw output force distribution
    drawOutputForceDistribution(ax, position=ankle_pos, V=MOFP_ankle, scale=value_scale);
    hold on;
    drawOutputForceDistribution(ax, position=toe_pos, V=MOFP_toe, scale=value_scale);
    hold on;

end