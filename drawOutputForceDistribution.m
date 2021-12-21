function drawOutputForceDistribution(ax, position, V, scale=1, line_width=1, fill_color=[0.5 0.1 0.7])
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

    % NOTE: octave doesn't support patch with transparency
    % p = patch(ax, V(1,:)*scale + position(1), V(2,:)*scale + position(2), fill_color);
    % set( get(a, 'children'), 'facealpha', 0.25 );
    % p.FaceVertexAlphaData = 0.2;
    % p.FaceAlpha = 'flat' ; 

    % Use simple line instead
    V = V * scale;
    V(:,end+1) = V(:,1);
    a = area (ax, V(1,:) + position(1), V(2,:) + position(2));
    set( get(a, 'children'), 'facealpha', 0.25 );


end