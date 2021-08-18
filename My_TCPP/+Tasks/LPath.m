classdef LPath < Tasks.Path
    methods
        function self = LPath(nn)
            % angle - elevation of the line with respect to x axis
            % nn    - number of points 
            line = [0 1 0; zeros(1,3); 1 0 0];
            line = interp1(linspace(0, 1, size(line, 1)), line, linspace(0, 1, nn), 'pchip');
            self = self@Tasks.Path(line);
        end
    end
end