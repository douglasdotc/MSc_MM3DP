function T = tformrotinv(T)
    %This inverses only the rotation matrices of the tforms, leave point in
    %place
    
    for ii = 1:size(T, 3)
        %         R = T(1:3,1:3,ii)';
        %             p = -R*T(1:3,4);
        %             Tinv = [R,    p; ...
        %                 [0 0 0 1]];
        T(1:3, 1:3, ii) = T(1:3, 1:3, ii)' ;
    end

end
