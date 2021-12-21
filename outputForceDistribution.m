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

    % --- Nested functions ---
    function Vout = filterVector(V)
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
    phi = atan(V(2,:)./V(1,:));
    for i = 1:length(V),
        % Handle [-pi .. 0] range
        if V(1,i) < 0,
            phi(i) = phi(i) + pi;
        % Handle pi/2 degree
        elseif V(1,i) == 0,
            phi(i) = pi/2;
        end
    end

    % Sort the vector by angle
    [phi,sortId] = sort(phi,'ascend');
    V = V(:,sortId);

    % Check for vectors with same angle
    [~,~,id] = unique(phi);

    % Sum any parallel vector with same angle
    Vsum = zeros(size(V));
    for i = 1:length(id)
        Vsum(:,id(i)) += V(:,i);
    end
    % Filter any zeros vector
    Vsum = filterVector(Vsum);

    % Get the resultant vectors of the individual output forces
    for i = 1:length(Vsum)
        if ~(i==length(Vsum))
            Vtemp(:,i) = Vsum(:,i) + Vsum(:,i+1);
        
        % Handle the last vector in the array
        else
            Vtemp(:,i) = Vsum(:,i) + Vsum(:,1);
        end
    end

    % Get the resultant vectors of the resultants (for hexagon)
    for i = 1:length(Vtemp)
        if ~(i==length(Vtemp))
            Vres(:,i) = Vtemp(:,i) + Vtemp(:,i+1);
        
        % Handle the last vector in the array
        else
            Vres(:,i) = Vtemp(:,i) + Vtemp(:,1);
        end
    end

end