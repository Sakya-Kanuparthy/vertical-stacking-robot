clear all
close all

seed = 1234
rng(seed)

%% Generate some points

nrows = 1300;
ncols = 1300;

obstacle = false(nrows, ncols);

[x, y] = meshgrid (1:ncols, 1:nrows);

%% Generate some obstacle

obstacle (y<50 | y>(1300-50) | x<50 | x>(1300-50)) = true;

figure;
imshow(~obstacle);

axis ([0 ncols 0 nrows]);
axis xy; 
axis on;

xlabel ('x');
ylabel ('y');


title ('PRM Created Map (graph)');
grid on

%% PRM Parametes

Max_Nodes_Connect = 20;
Max_Connect_Length = 75;
Max_Nodes_Grid = 2000;

%% PRM Algorithm

Nodes = 0;
map = zeros(size(obstacle));
map(obstacle) = 1;
Graph_Connections = Inf(Max_Nodes_Grid, Max_Nodes_Connect + 1);
while (Nodes < Max_Nodes_Grid)
    %% Generate a node at a random location in the map & check if valid node
    Node_X = randi(ncols);
    Node_Y = randi(nrows);
    
    if (map(Node_Y, Node_X) == 1 || map(Node_Y, Node_X) == 2)
        continue;
    end
   
    Nodes = Nodes + 1;
    
    map(Node_Y, Node_X) == 2;

    hold on 
    plot(Node_X, Node_Y,'r*')
    hold off

    %% Connect the new node to the closest Max_Nodes_Connect nodes
    nodes_to_connect = [];
    distances = [];
    for i = 1: numel(Graph_Connections(:,1))
        if(Graph_Connections(i,1) == Inf)
            break
        end
        [row, col] = ind2sub(size(map), Graph_Connections(i,1));

        % Check if within range
        if(norm([Node_Y, Node_X] - [row, col]) > Max_Connect_Length)
            continue
        end

        nodes_to_connect = [nodes_to_connect, Graph_Connections(i,1)];
        
        distances = [distances; [Graph_Connections(i,1), norm([Node_Y Node_X] - [row, col])]];

    end

    %% Choose the closer Max_Nodes_Connect to connect to

    Graph_Connections(Nodes, 1) = sub2ind(size(map), Node_Y, Node_X);
    
    if (size(distances > 0))
        distances_sorted = sortrows(distances, 2);
        for i = 1;  min(Max_Nodes_Connect, size(distances_sorted, 1))
            Graph_Connections(Nodes, i+1) = distances_sorted(i, 1);

            [row, col] = ind2sub(size(map), distances_sorted(i,1) );

            hold on
            line([Node_X, col], [Node_Y, row])
            hold off

        end

    end


end




