function X = calculateWireLengthWithADMA(ADMA_par, theta)
    % This function calculates the wire length for a muscle from the guide circle to the insertion point.
    %
    % For definition of the ADMA parameter and the moment arm calculation,
    % see ADMA paper or https://tandukion.github.io/blog/2017/11/30/musculoskeletal-force-profile.html
    %
    % Args:
    %     - ADMA_par (Arrays 5x1): ADMA parameter consisting of: 
    %         - a: translation parallel to link i
    %         - b: translation perpendicular to link i
    %         - c: position of insertion point from the center of rotation, parallel to link i+1
    %         - d: position of insertion point from the center of rotation, perpendicular to link i+1
    %         - r: radius of the guide circle
    %     - theta: joint angles
    %
    % Return:
    %     - X: moment arm

    % Get the ADMA parameter
    [a b c d r] = deal(num2cell(ADMA_par){:});

    % Calculate supporting variables
    A = -a + c*cos(theta) + d*sin(theta);
    B =  b + c*sin(theta) - d*cos(theta);

    phi = acos(r/sqrt(A^2+B^2)) - atan2(B,A);

    % Calculate wire perpendicular to the guide circle
    if(phi==pi/2||phi==-pi/2)
        X = (A - r*cos(phi)) / sin(phi);
    else
        X = (B + r*sin(phi)) / cos(phi); 
    end

    % Add the wire along the guide circle
    X = X + (pi/2 - phi) * r;
end