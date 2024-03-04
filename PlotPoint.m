classdef PlotPoint
    methods(Static)
        function LagrangePoint(coords)
            axes(gca);
            plot3(coords(1),coords(2),coords(3),'rx',...
                'MarkerSize',12,'DisplayName','LagrangePoint')
        end
        function Sun(coords)
            axes(gca);
            plot3(coords(1),coords(2),coords(3),'ro',...
                'MarkerSize',20,'DisplayName','Sun')
        end 
        function Moon(coords)
            axes(gca);
            plot3(coords(1),coords(2),coords(3),'ko',...
                'MarkerSize',15,'DisplayName','Moon')
        end 
        function Earth(coords)
            axes(gca);
            plot3(coords(1),coords(2),coords(3),'bo',...
                'MarkerSize',18,'DisplayName','Earth')
        end 
        function Asteroid(coords)
            axes(gca);
            plot3(coords(1),coords(2),coords(3),'ko',...
                'MarkerSize',15,'DisplayName','Asteroid')
        end 
        function Chaser(coords)
            axes(gca);
            plot3(coords(1),coords(2),coords(3),'bx',...
                'MarkerSize',10,'DisplayName','Chaser')
        end 
        function Target(coords,size)
            if nargin <2 
                size = 100; 
            end

            axes(gca);
            plot3(coords(1),coords(2),coords(3),'r.','MarkerSize',size,'DisplayName','Target Spacecraft')
        end

    end
end 
    