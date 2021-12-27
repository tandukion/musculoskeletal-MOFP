function drawMOFP(X,Q)
    % This function 


    % Create the end-effector position matrix in (2xN)
    for i = 1:length(X)
        ee_pos(:,i) = X{i}(1:2,:);
    end

    % Create new variables for special position to make it easier to understand
    ankle_pos = ee_pos(:,end-1);
    toe_pos = ee_pos(:,end);

    Q_ankle = Q(:,:,end-1);
    Q_toe = Q(:,:,end);


    % ====== Main figure window settings ======
    set(gcf, 'Position', [2000, 100, 1400, 700]);  % set position and size of the figure window

    % ====== Subplot settings ======
    % Subplot axis parameter
    axis_par = [-0.5 0.5 -1.0 0.1];

    % #1 Output force subplot
    ax_force = subplot(1,2,1);
    title('Maximum Output Force Profile');
    axis equal;
    axis(axis_par);
    hold on;

    % #2 Compliance subplot
    ax_compliance = subplot(1,2,2);
    title('Compliance Profile');
    axis equal;
    axis(axis_par);
    hold on;

    % ====== Plot input ======
    % --- Draw robot link for all plots ---
    % Draw for both force analysis and compliance analysis
    drawLink(ax_force, ee_pos);
    drawLink(ax_compliance, ee_pos);

    % --- Draw output force ---
    % Create settings for the output force on polar axis
    tick_space = 10; % tick space for polar axis
    axis_scale = 0.1 % axis scale to be fit into the robot link plot
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
    drawPolarAxis(ax_force, position=toe_pos, value_power=value_power, axis_scale=axis_scale, tick_space=tick_space);

    % Draw individual output force vectors on ankle
    drawVector(ax_force, position=ankle_pos, F=Q_ankle, scale=value_scale);

    % Draw individual output force vectors on toe
    drawVector(ax_force, position=toe_pos, F=Q_toe, scale=value_scale);


    % ====== Plot analysis output ======
    % Compute the output force distribution
    % ankle
    V_ankle = outputForceDistribution(V=Q_ankle);
    % toe
    V_toe = outputForceDistribution(V=Q_toe);

    % Draw output force distribution
    drawOutputForceDistribution(ax_force, position=ankle_pos, V=V_ankle, scale=value_scale);
    drawOutputForceDistribution(ax_force, position=toe_pos, V=V_toe, scale=value_scale);

end