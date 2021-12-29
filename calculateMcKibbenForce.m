function F = calculateMcKibbenForce(D0,theta0,L0,L,P)
    % This function calculates the output force of a McKibben muscle.
    % For the detail of the calculation, please see the note on
    % https://tandukion.github.io/blog/2016/10/20/pam-review.html
    %
    % Args:
    %     - D0: initial braid diameter (m)
    %     - theta0: initial braid angle (rad)
    %     - L0: initial muscle length (m)
    %     - L: final muscle length (m)
    %     - P: air pressure (Pa)
    % Return:
    %     - F: Output force (N)

    % Calculate supporting variables
    A = 3 / (tan(theta0))^2;
    B = 1 / (sin(theta0))^2;
    epsilon = (L0 - L) / L;

    % Calculate output force
    F = pi * D0^2 * P * (A * (1-epsilon)^2 - B) / 4;
end