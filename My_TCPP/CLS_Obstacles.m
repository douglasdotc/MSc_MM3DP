classdef CLS_Obstacles
    methods (Static)
        function [Obstacles, Obstacle_break] = Obstacle_Config_select(idx, IsDEBUG)
        %%
            if idx == 1
                [Obstacles, Obstacle_break] = CLS_Obstacles.Obstacle_config_1();
            elseif idx == 2
                [Obstacles, Obstacle_break] = CLS_Obstacles.Obstacle_config_2();
            elseif idx == 3
                [Obstacles, Obstacle_break] = CLS_Obstacles.Obstacle_config_3();
            elseif idx == 4
                [Obstacles, Obstacle_break] = CLS_Obstacles.Obstacle_config_4();
            elseif idx == 5
                [Obstacles, Obstacle_break] = CLS_Obstacles.Obstacle_config_5();
            elseif idx == 6
                [Obstacles, Obstacle_break] = CLS_Obstacles.Obstacle_config_6();
            elseif idx == 7
                Obstacles = CLS_Obstacles.Obstacle_test_config();
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
        
        function [Obstacles, Obstacle_break] = Obstacle_config_1()
            %%
            % Hinge:
            [Obstacles{1}, Obstacle_break{1}] = CLS_Obstacles.square_type(1.1, -0.25, 0.8, 1.5); %[1.5, 0.5], 0.8, 1.5);% 
            % Short narrow gap:
            [Obstacles{2}, Obstacle_break{2}] = CLS_Obstacles.square_type(2.1,  1.25, 0.8, 0.5); % [2.5, 1.5], 0.8, 0.5);% 
            [Obstacles{3}, Obstacle_break{3}] = CLS_Obstacles.square_type(3.1,  1.25, 0.8, 0.5); % [3.5, 1.5], 0.8, 0.5);% 
            Obstacles{4} = [[repmat(0.1,8,1), linspace(2,2.8,8)'];
                           [linspace(0.15,0.85,7)', repmat(2.88,7,1)];
                           [repmat(0.9,8,1), linspace(2.8,2,8)'];
                           [linspace(0.9,0.1,8)', repmat(2,8,1)]];
            Obstacle_break{4} = [0.1, 2.0;
                                0.1, 2.8;
                                0.15, 2.88;
                                0.85, 2.88;
                                0.9, 2.8;
                                0.9, 2.0;];

            [Obstacles{5}, Obstacle_break{5}] = CLS_Obstacles.circle([0.5, 3.5], 0.4);
        end
        
        function [Obstacles, Obstacle_break] = Obstacle_config_2()
            %%
            % Long narrow gap (not printable):
            [Obstacles{1}, Obstacle_break{1}] = CLS_Obstacles.square_type(2.1, 1.15, 0.8, 1.7);
            [Obstacles{2}, Obstacle_break{2}] = CLS_Obstacles.square_type(3.1, 1.15, 0.8, 1.7);
        end
        
        function [Obstacles, Obstacle_break] = Obstacle_config_3()
            %%
            % Long narrow gap (printable):
            [Obstacles{1}, Obstacle_break{1}] = CLS_Obstacles.square_type(2.15, 1.15, 0.5, 1.7);
            [Obstacles{2}, Obstacle_break{2}] = CLS_Obstacles.square_type(3.35, 1.15, 0.65, 1.7);
        end
        
        function [Obstacles, Obstacle_break] = Obstacle_config_4()
            %%
            % Long narrow gap (not printable):
            [Obstacles{1}, Obstacle_break{1}] = CLS_Obstacles.square_type(2.1, 2.25, 0.8, 0.5);
            [Obstacles{2}, Obstacle_break{2}] = CLS_Obstacles.square_type(3.1, 1.75, 0.8, 0.5);
            [Obstacles{3}, Obstacle_break{3}] = CLS_Obstacles.square_type(2.1, 1.25, 0.8, 0.5);
        end
        
        function [Obstacles, Obstacle_break] = Obstacle_config_5()
            %%
            % Non convex
            Obstacles{1}      = [[repmat(0.1,8,1), linspace(2,2.8,8)'];
                                 [linspace(0.15,0.85,7)', repmat(2.88,7,1)];
                                 [repmat(0.9,8,1), linspace(2.8,2,8)'];
                                 [linspace(0.9,0.1,8)', repmat(2,8,1)]];
            Obstacle_break{1} = [0.1, 2.0;
                                 0.1, 2.8;
                                 0.15, 2.88;
                                 0.85, 2.88;
                                 0.9, 2.8;
                                 0.9, 2.0;];
                             
            Obstacles{2}      = [[repmat(-0.1,4,1), linspace(2.5,2.9,4)'];
                                 [linspace(0.1,0.5,4)', repmat(3.1,4,1)];
                                 [repmat(0.5,5,1), linspace(3.1,3.6,5)'];
                                 [linspace(0.5,-0.6,11)', repmat(3.6,11,1)];
                                 [repmat(-0.6,11,1), linspace(3.6,2.5,11)']];
                             
            Obstacle_break{2} = [-0.1, 2.5;
                                 -0.1, 2.9;
                                  0.1, 3.1;
                                  0.5, 3.1;
                                  0.5, 3.6;
                                 -0.6, 3.6;
                                 -0.6, 2.5;];
        end
        
        function [Obstacles, Obstacle_break] = Obstacle_config_6()
            %%
            % Long narrow gap 2 (printable):
            [Obstacles{1}, Obstacle_break{1}] = CLS_Obstacles.square_type(2.15, 1.15, 0.57, 1.35);
            [Obstacles{2}, Obstacle_break{2}] = CLS_Obstacles.square_type(3.28, 1.15, 0.72, 1.35);
        end
        
        function [Obstacles, Obstacle_break] = Obstacle_test_config()
            den=0.1;
            mycube=@(x,y,dx,dy) [ [x,y]; [repmat(x,ceil(dy/den),1), linspace(y,y+dy,ceil(dy/den))'] ; [linspace(x,x+dx,ceil(dx/den))', repmat(y+dy,ceil(dx/den),1)];    [repmat(x+dx,ceil(dy/den),1), linspace(y+dy,y,ceil(dy/den))']   ];
            Obstacles={};
            Obstacles{1}=mycube(-.5,-.5,2-(.25/2),.25);
            Obstacles{2}=mycube(1.5-(.25/2),-.5,0.25,2);
            Obstacles{3}=mycube(1.5-(.25/2),2.5,0.25,.75);
            Obstacles{4}=mycube(1.5-(.25/2),3.25,3.25-1.5+(.25/2),.25);
            Obstacles{5}=mycube(3.25-.2,3.25-3.75,.2,3.75);
        end
        
        function [obstacle, obstacle_break] = square_type(x,y,dx,dy)% center, w, h)%
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
            den = 0.1;
            L = [repmat(x,ceil(dy/den),1), linspace(y,y+dy,ceil(dy/den))'];
            U = [linspace(x,x+dx,ceil(dx/den))', repmat(y+dy,ceil(dx/den),1)];
            R = [repmat(x+dx,ceil(dy/den),1), linspace(y+dy,y,ceil(dy/den))'];
            D = [linspace(x+dx,x,ceil(dx/den))', repmat(y,ceil(dx/den),1)];
            obstacle = [L;U;R;D];
            obstacle_break = [L(1,:);
                              U(1,:);
                              R(1,:);
                              D(1,:);
                              L(1,:)];
        end

        function [obstacle, obstacle_break] = circle(center, r)
            theta    = [-pi:pi/50:pi]';
            obstacle = [r*cos(theta) + center(1), r*sin(theta) + center(2)];
            obstacle_break = obstacle;
        end
    end
end