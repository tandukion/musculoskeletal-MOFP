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
    scale = 0.01;

    % Draw polar scale (only need to draw one at toe)
    drawPolarAxis(ax_force, position=toe_pos, scale=scale, tick_space=tick_space);

    % Draw individual output force vectors on ankle
    drawVector(ax_force, position=ankle_pos, F=Q_ankle, scale=scale);

    % Draw individual output force vectors on toe
    drawVector(ax_force, position=toe_pos, F=Q_toe, scale=scale);


    % ====== Plot analysis output ======
    % Compute the output force distribution
    % ankle
    V_ankle = outputForceDistribution(V=Q_ankle);
    % toe
    V_toe = outputForceDistribution(V=Q_toe);

    % Draw output force distribution
    drawOutputForceDistribution(ax_force, position=ankle_pos, V=V_ankle, scale=scale);
    drawOutputForceDistribution(ax_force, position=toe_pos, V=V_toe, scale=scale);

end