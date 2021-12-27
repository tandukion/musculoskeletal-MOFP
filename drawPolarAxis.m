function drawPolarAxis(ax, position, value_power=1, axis_scale=1, tick_space=1, line_width=0.5, line_color=[0.1 0.1 0.1])
    % This function draws robot links and joints based on the given end-effector position
    % The first joint will be drawn on (0,0) position.
    %
    % Args:
    %   - ax (Axes): Target axes to plot
    %   - position (Array 2x1): position of the polar axis origin 
    %   - scale (scalar): scale of the polar axis
    %   - tick_space (scalar): range for the tick space
    %   - line_width (scalar): line width
    %   - line_color (Array 1x3): line color

    % Define the radius for the circles
    r = [tick_space 2*tick_space 3*tick_space]; % [N]

    % Draw the circles
    angle = 0.0:pi/16:2*pi;
    for i = 1:length(r)
        X(:,:,i) = r(i) * axis_scale * cos(angle);
        Y(:,:,i) = r(i) * axis_scale * sin(angle);
        % Draw the circle with origin on the selected position
        plot(ax, X(:,:,i) + position(1), Y(:,:,i) + position(2), '--', 'Color', line_color, 'LineWidth',line_width)

        % Display scale on plot
        if (abs(value_power) > 2)
            scale_label = strcat(sprintf('%d', i*tick_space), ' e', sprintf('%d',value_power));
        else
            scale_label = sprintf('%d', i * tick_space * 10^value_power);
        end
        if (i==length(r))
            scale_label = strcat(scale_label, ' N');
        end
        
        x_axis_offset = -0.025;
        text(position(1)+r(i)*axis_scale, position(2)+x_axis_offset, scale_label, 'FontSize',12, 'HorizontalAlignment', 'center')
    end

    % Draw extra line axes
    angle = 0.0:pi/6:2*pi;
    for i = 1:length(angle)
        % Create line from origin with angle(i)
        A = [0 r(end) * axis_scale * cos(angle(i))];
        B = [0 r(end) * axis_scale * sin(angle(i))];

        % Plot the line with origin on the selected position (need to be in (1x2) dimension)
        plot(ax, A + repmat(position(1), [1 2]), B + repmat(position(2), [1 2]), '--','Color',line_color,'LineWidth',line_width)
    end
end