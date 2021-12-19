function drawLink(ax, joint_pos, line_color=[0 0 0], line_width=2)
    % This function draws robot links and joints based on the given joint positions
    %
    % Args:
    %   - ax (Axes): Target axes to plot
    %   - joint_pos (Arrays): Array (2xN) of joint (+ end effector) position in 2D. e.g: for 3 DoF, N=4

    % --- Draw links ---
    % Draw lines from two consecutive joints
    for i = 1:length(joint_pos)-1
        A = [joint_pos(1,i) joint_pos(1,i+1)];
        B = [joint_pos(2,i) joint_pos(2,i+1)];
        line(ax, A,B,'Color',line_color,'LineWidth',line_width)
    end

    % Draw extra line to show which joint is the first joint
    line(ax, [-0.05 0.05] + joint_pos(1,1), [0 0] + joint_pos(2,1) ,'Color',line_color,'LineWidth',line_width)

    % --- Draw joints ---
    % Create circle(s) for the joint.
    % Define the radius of the circle here. Put number of circle to draw.
    r = [0.014 0.024];
    circle_color = [0 0 0.8];

    angle = 0.0:pi/16:2*pi;
    for i = 1:length(r)
        X(:,:,i) = r(i) * cos(angle);
        Y(:,:,i) = r(i) * sin(angle);
    end
    
    for i = 1:length(joint_pos)
        for j = 1:length(r)
            plot(ax, X(:,:,j) + joint_pos(1,i), Y(:,:,j) + joint_pos(2,i), 'Color', circle_color, 'LineWidth',line_width/2)
        end
    end
end