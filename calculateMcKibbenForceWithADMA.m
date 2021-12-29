function F = calculateMcKibbenForceWithADMA (mus_param,ADMA_par,theta,P, flexor=[1 1 1])
    % This function calculates the muscle force with ADMA structure.
    %
    % Args:
    %     - mus_param (Array 4x1): McKibben muscle parameter:
    %         - D0: initial braid diameter (m)
    %         - theta0: initial braid angle (rad)
    %         - L0: initial muscle length (m)
    %         - N: number of muscle
    %     - ADMA_par (Array 5xN): ADMA parameter of a muscle on N joints:
    %         - a: translation parallel to link i
    %         - b: translation perpendicular to link i
    %         - c: position of insertion point from the center of rotation, parallel to link i+1
    %         - d: position of insertion point from the center of rotation, perpendicular to link i+1
    %         - r: radius of the guide circle
    %     - theta (Array Nx1): current joint angles
    %     - P: air pressure (Pa)
    %     - flexor (Array 1xN): flag to indicate the current muscle is a flexor muscle for the joint i
    % Returns:
    %     - F: Output force (N)

    % Since the ADMA parameter doesn't handle sign for an antagonist muscle,
    % handle the antagonist muscle
    for i = 1:length(theta)
        if ~flexor(i)
            theta(i) = -theta(i);
        end
    end

    % Extract the muscle parameter
    [D0 theta0 L0 mus_num] = deal(num2cell(mus_param){:});

    % Extract the joint angles for max muscle length from ADMA parameter
    theta_Lmax = ADMA_par(6,:);

    % Make ADMA parameter only consist of [a b c d r]
    ADMA_par = ADMA_par(1:5,:);

    % --- Calculate the initial muscle length (Lmax) ---
    % Calculate the wire length on ADMA structure for all joints for Lmax
    for i = 1:size(ADMA_par,2)
        if ~isequal(ADMA_par(1:5,i), zeros(5,1))
            X(i) = calculateWireLengthWithADMA(ADMA_par(1:5,i), theta_Lmax(i));
        else
            X(i) = 0;
        end
    end
    Lmax = L0 + sum(X);

    % --- Calculate the prresurized muscle length ---
    % Calculate the wire length on ADMA structure for all joints
    for i = 1:size(ADMA_par,2)
        if ~isequal(ADMA_par(1:5,i), zeros(5,1))
            X(i) = calculateWireLengthWithADMA(ADMA_par(1:5,i), theta(i));
        else
            X(i) = 0;
        end
    end
    L = L0 + sum(X);

    Ldiff = Lmax - L;

    % --- Calculate muscle force
    F = calculateMcKibbenForce(D0,theta0,L0,L0-Ldiff,P);
    F = F * mus_num;
end