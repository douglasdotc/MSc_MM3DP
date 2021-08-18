classdef TaskAbilityMap < handle

    properties
        ikobj

    end

    methods

        function self = TaskAbilityMap()
            self.check_ros_setup()%Check that ros network is up
        end

    end

end
