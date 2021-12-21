%
% The output forces developed by the multiple muscle systems are derived by geometrical summation of the individual output force vectors.
% The output force may shape in a hexagonal shape.
% 
% Reference:
% @inproceedings{oshima2000robotic,
%   title={Robotic analyses of output force distribution developed by human limbs},
%   author={Oshima, Toru and Fujikawa, Tomohiko and Kameyama, Osamu and Kumamoto, Minayori},
%   booktitle={Proceedings 9th IEEE International Workshop on Robot and Human Interactive Communication. IEEE RO-MAN 2000 (Cat. No. 00TH8499)},
%   pages={229--234},
%   year={2000},
%   organization={IEEE}
% }

function Vres = outputForceDistribution(V)
    % This function calculates the corners of the geometry shape of the output force distribution.
    %
    % Args:
    %   - ax (Axes): Target axes to plot
    %
    % Return:
    %   - Vres (Array): Vectors denotes the corners of the geometry shape of the output force distribution.

    % Based on the paper, the hexagon of output force distribution can be created by computing the resultant vectors for every two consecutive
    % vectors (sorted in angles), done two times.

    % Based on the paper, the geometry of output force distribution can be computed by the following steps:
    %   - For each individual vectors, compute resultant vectors with the other vectors. 
    %   - Create convex hull from the resultant vectors

    % --- Nested functions ---
    function Vout = filterVector(V)
        % This function filter Vector with X=0 and Y=0
        j = 0;
        for i = 1:length(V)
            if ~((V(1,i)==0 && (V(2,i)==0)))
                j += 1;
                Vout(:,j) = V(:,i);
            end
        end
    end

    % Filter the vector array without any zero vector
    V = filterVector(V);

    % Calculate the angle of each vector
    % Handle using atan2 method
    % phi = atan(V(2,:)./V(1,:));
    for i = 1:length(V)
        % Handle [-pi/2 .. pi/2]
        if (V(1,i) > 0)
            phi(i) = atan(V(2,i)./V(1,i));
        else
            % Handle [pi/2 .. pi]
            if (V(2,i) >= 0)
                phi(i) = atan(V(2,i)./V(1,i)) + pi;
            
            % Handle [-pi .. -pi/2]
            elseif (V(2,i) < 0)
                phi(i) = atan(V(2,i)./V(1,i)) - pi;
            
            % pi/2
            elseif ((V(2,i) == 0) && (V(2,i) > 0))
                phi(i) = pi/2;
            
            % -pi/2
            elseif ((V(2,i) == 0) && (V(2,i) < 0))
                phi(i) = -pi/2;
            end
        end
    end

    % Make phi in range of [0..2pi]
    for i = 1:length(phi)
        if (phi(i) < 0)
            phi(i) += 2*pi;
        end
    end

    % Sort the vector by angle
    [phi,sortId] = sort(phi,'ascend');
    V = V(:,sortId);

    % Check for vectors with same angle
    [phi,~,id] = unique(phi);

    % Sum any parallel vector with same angle
    Vsum = zeros(size(V));
    for i = 1:length(id)
        Vsum(:,id(i)) += V(:,i);
    end
    % Filter any zeros vector
    Vsum = filterVector(Vsum);
    
    % Create resultant vectors
    k = 0;
    for i = 1:length(Vsum)
        Vres(:,i) = Vsum(:,i);  % current vector
        for j = 1:length(Vsum)
            if ~(i==j)
                % Calculate vectors which only in [0..pi] ranges from the current vector
                diff_phi = phi(j) - phi(i);
                if ((diff_phi > 0)&&(diff_phi < pi) || (diff_phi+2*pi < pi))
                    Vres(:,i) = Vres(:,i) + Vsum(:,j);
                end
            end
        end
    end
end