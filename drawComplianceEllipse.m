
function drawComplianceEllipse (ax, position, C, scale=1, line_width=0.5, line_color=[0.8 0.8 0.8])
    % This function draws the compliance ellipse on the end-effector.
    %
    % Args:
    %   - ax (Axes): Target axes to plot
    %   - position (Array 2x1): position of the end-effector
    %   - C (Array 2xN): Compliance matrix
    %   - scale (scalar): scale of the polar axis
    %   - line_width (scalar): line width
    %   - line_color (Array 1x3): line color
    
    % applied force on all direction
    t = linspace(0,2*pi);
    P = 1 * scale * [cos(t);sin(t)];
    
    X = C(1:2,1:2) * P;
    plot(ax, position(1)+X(1,:), position(2)+X(2,:), 'Color', line_color, 'LineWidth', line_width);

    % -- drawing compliance ellipse angle --
    % getting othogonal basis of C
    B = orth(C);
    m = B(2,1)/B(1,1);
    x = -1:0.1:1;
    y = m*(x - position(1)) + position(2);
    plot(x,y,'Color', line_color, 'LineStyle','--', 'LineWidth', line_width);
        
end