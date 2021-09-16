classdef CLS_Obstacles
    methods (Static)
        function Obstacles = Obstacle_Config_select(idx, IsDEBUG)
        %%
            if idx == 1
                Obstacles = CLS_Obstacles.Obstacle_config_1();
            elseif idx == 2
                Obstacles = CLS_Obstacles.Obstacle_config_2();
            elseif idx == 3
                Obstacles = CLS_Obstacles.Obstacle_config_3();
            elseif idx == 4
                Obstacles = CLS_Obstacles.Obstacle_config_4();
            elseif idx == 5
                Obstacles = CLS_Obstacles.Obstacle_config_5();
            elseif idx == 6
                Obstacles = CLS_Obstacles.Obstacle_config_6();
            end
            
            if IsDEBUG
                for jdx = 1:length(Obstacles)
                    Obstacles_Poly{jdx} = polyshape(Obstacles{jdx});
                    if IsDEBUG
                        plot(Obstacles_Poly{jdx}, 'FaceColor', 'y')
                    end
                    hold on
                end
            end
        end
        
        function Obstacles = Obstacle_config_1()
            %%
            % Hinge:
            Obstacles{1} = CLS_Obstacles.square_type([1.5, 0.5], 0.8, 1.5);
            % Short narrow gap:
            Obstacles{2} = CLS_Obstacles.square_type([2.5, 1.5], 0.8, 0.5);
            Obstacles{3} = CLS_Obstacles.square_type([3.5, 1.5], 0.8, 0.5);
            Obstacles{4} = [0.1, 2.0;
                            0.1, 2.8;
                            0.15, 2.88;
                            0.85, 2.88;
                            0.9, 2.8;
                            0.9, 2.0;];
            Obstacles{5} = CLS_Obstacles.circle([0.5, 3.5], 0.4);
        end
        
        function Obstacles = Obstacle_config_2()
            %%
            % Long narrow gap (not printable):
            Obstacles{1} = CLS_Obstacles.square_type([2.5, 2], 0.8, 1.7);
            Obstacles{2} = CLS_Obstacles.square_type([3.5, 2], 0.8, 1.7);
        end
        
        function Obstacles = Obstacle_config_3()
            %%
            % Long narrow gap (printable):
            Obstacles{1} = [2.725, 2.85;
                            2.725, 1.15;
                            2.15,  1.15;
                            2.15,  2.85;];
                        
            Obstacles{2} = [3.275, 2.85;
                            3.275, 1.15;
                            4,     1.15;
                            4,     2.85;];
        end
        
        function Obstacles = Obstacle_config_4()
            %%
            % Long narrow gap (not printable):
            Obstacles{1} = CLS_Obstacles.square_type([2.5, 2.5], 0.8, 0.5);
            Obstacles{2} = CLS_Obstacles.square_type([3.5, 2], 0.8, 0.5);
            Obstacles{3} = CLS_Obstacles.square_type([2.5, 1.5], 0.8, 0.5);
        end
        
        function Obstacles = Obstacle_config_5()
            %%
            % Non convex
            Obstacles{1} = [0.1, 2.0;
                            0.1, 2.8;
                            0.15, 2.88;
                            0.85, 2.88;
                            0.9, 2.8;
                            0.9, 2.0;];
                            
            Obstacles{2} = [-0.1, 2.5;
                            -0.1, 2.9;
                            0.1, 3.1;
                            0.5, 3.1;
                            0.5, 3.6;
                            -0.6, 3.6;
                            -0.6, 2.5;];
        end
        
        function Obstacles = Obstacle_config_6()
            %%
            % Long narrow gap (not printable):
            Obstacles{1} = [];
        end
        
        function obstacle = square_type(center, w, h)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Create square type obstacles
        %
        % Inputs:
        % center:   1x2 vector, center position [x, y]
        % w:        scalar, width
        % h:        scalar, height
        % Outputs:
        % obstacle: 5x2 matrix, each row is a coordinate of the vertex of the
        %           obstacle
        %
        % Example:
        %                   +-----------+-
        %                   |           |^
        %                   |           ||
        %                   |     x     || h
        %                   |  center   ||
        %                   |           |v
        %                   +-----------+-
        %                   |<--------->|
        %                         w
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            obstacle = [center(1) - w/2, center(2) - h/2;
                        center(1) + w/2, center(2) - h/2;
                        center(1) + w/2, center(2) + h/2;
                        center(1) - w/2, center(2) + h/2;];
        end

        function obstacle = circle(center, r)
            theta    = [-pi:pi/50:pi]';
            obstacle = [r*cos(theta) + center(1), r*sin(theta) + center(2)];
        end
    end
end