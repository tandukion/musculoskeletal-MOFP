function R = calculateADMA (ADMA_par,theta)
    % This function calculates the Angle-Dependent Moment Arm from the given ADMA parameter and joint angle.
    %
    % For definition of the ADMA parameter and the moment arm calculation,
    % see ADMA paper or https://tandukion.github.io/blog/2017/11/30/musculoskeletal-force-profile.html
    %
    % Args:
    %     - ADMA_par (Arrays 1x5): ADMA parameter consisting of: 
    %         - a: translation parallel to link i
    %         - b: translation perpendicular to link i
    %         - c: position of insertion point from the center of rotation, parallel to link i+1
    %         - d: position of insertion point from the center of rotation, perpendicular to link i+1
    %         - r: radius of the guide circle
    %     - theta: joint angle
    %
    % Return:
    %     - R: moment arm

    [a b c d r] = deal(num2cell(ADMA_par){:});

    % Calculate supporting variables
    A = -a + c*cos(theta) + d*sin(theta);
    B =  b + c*sin(theta) - d*cos(theta);

    phi = acos(r/sqrt(A^2+B^2)) - atan2(B,A);

    R = r + a*cos(phi) + b*sin(phi);
end