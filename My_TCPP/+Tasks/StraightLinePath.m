classdef StraightLinePath < Tasks.Path
    methods
        function self = StraightLinePath(angle, nn)
            % angle - elevation of the line with respect to x axis, degree
            % nn    - number of points 
            P     = [zeros(1,3); 1 0 0];
            rotmz = eul2rotm([angle*pi/180,0,0], 'ZYX');
            line  = (rotmz*P')';
            line  = interp1(linspace(0, 1, size(line, 1)), line, linspace(0, 1, nn), 'pchip');
            self  = self@Tasks.Path(line);
        end
    end
end