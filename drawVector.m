function drawVector(ax, position, V, scale=1, line_width=1, line_color=[0.5 0.1 0.7])
    % This function draws robot links and joints based on the given end-effector position
    % The first joint will be drawn on (0,0) position.
    %
    % Args:
    %   - ax (Axes): Target axes to plot
    %   - position (Array 2x1): position of the vector origin 
    %   - V (Array 2xN): Vectors
    %   - scale (scalar): scale of the polar axis
    %   - tick_space (scalar): range for the tick space
    %   - line_width (scalar): line width
    %   - line_color (Array 1x3): line color

    % Get the number of vector to draw
    n = size(V,2);

    % Draw the vector
    q = quiver(ax, zeros(1,n) + position(1), zeros(1,n) + position(2), V(1,:), V(2,:), scale, 'Color', line_color, 'LineWidth', line_width);

    % Uncomment below lines for simple line, not quiver
    % % Create set of lines
    % A =[zeros(1,n); V(1,:)*scale];
    % B =[zeros(1,n); V(2,:)*scale];
    % plot(ax, A + position(1), B + position(2), 'Color', line_color, 'LineWidth', line_width);
end