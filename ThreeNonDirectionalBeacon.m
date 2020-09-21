% Author: Mario Terres Diaz
% Coordinates of the NDBs
clear all;
NDB = [70 10; 50 60; 30 20; 90 90; 10 50; 80 50];
NDBx = NDB(:,1);
NDBy = NDB(:,2);
 
% Creates a matrix with all possible combinations of three NDBs
n = length(NDB);
v = 1:1:n;
K = nchoosek(v, 3);
N = length(K);
 
% Plots the map for selecting the route
% Image dimensions
dimX = 100; 
dimY = 100;
P = imread('Europe.PNG');
figure('Position', get(0, 'Screensize'));
image([0 dimX], [dimY 0],P);
set(gca, 'ydir', 'normal'); hold on;
scatter(NDBx, NDBy, 200,'r', 'MarkerFaceColor','m'); hold on;
t = 'Click on the beginning and end of your route\nThen press enter';
text(2, dimY -5, t, 'Color', 'red');
xlim([0 dimX]);
ylim([0 dimY]);
axis off;
[x,y] = getpts; hold on;

% Plots the map with the selected route
image([0 dimX],[dimY 0],P);
set(gca,'ydir','normal'); hold on;
scatter(NDBx,NDBy,200,'r','MarkerFaceColor','m');
xlim([0 dimX]);
ylim([0 dimY]);
axis off;
 
% Condition for selecting 2 points
if length(x) ~= 2
    error("Select 2 points for defining the route");
end

% Plots the route
plot(x, y, 'k', 'LineWidth', 2); hold on;
 
% Find unit vector of the route direction
D = sqrt((x(2)-x(1))^2+(y(2)-y(1))^2);
 
% Divides the route in several points with distance dist
dist = 20;
q = floor(D / dist);
for i = 1:q
    d = i*dist;
    Px(i) = x(1) + (d/D)* (x(2)-x(1));
    Py(i) = y(1) + (d/D)* (y(2)-y(1));
end
 
% Plots the route divided
scatter(Px, Py);
scatter(x, y); hold on;
 
% Error for the first point of the route
for i = 1: N  
    ERROR(i) = CalculateLocationError(NDB(K(i,1),:), NDB(K(i,2),:), NDB(K(i,3),:), x(1), y(1), dimX);
end
 
% Selects the minimum error value and index from all the combinations
Minimum = min(ERROR);
MinE = find(ERROR==Minimum,1);
 
% Takes the poligon of that calculated error
[~, polyout, Lx1, Ly1, Lx2,Ly2] = CalculateLocationError(NDB(K(MinE,1),:), NDB(K(MinE,2),:), NDB(K(MinE,3),:), x(1), y(1), dimX);
 
plot(polyout,'FaceColor','blue'); hold on;
plot(Lx1, Ly1); hold on;
plot(Lx2, Ly2); hold on;

% Loop for the intermediate points of the route 
for i= 1:length(Px)
    plotAircraftLocation(NDB, N, K, x, y, P, Px, Py, i, dimX, dimY);
end
 
% Error for the lasst point of the route
plotAircraftLocation(NDB, N, K, x, y, P, x, y, 2, dimX, dimY);

function [] =  plotAircraftLocation(NDB, N, K, x, y, P, Px, Py, i, dimX, dimY)
    NDBx = NDB(:,1);
    NDBy = NDB(:,2);
    for j = 1: N
        ERROR(j) = CalculateLocationError(NDB(K(j,1),:), NDB(K(j,2),:), NDB(K(j,3),:), Px(i), Py(i), dimX);
    end

    Minimum = min(ERROR);
    MinE = find(ERROR==Minimum,1);

    [~, polyout, Lx1, Ly1,Lx2, Ly2] = CalculateLocationError(NDB(K(MinE,1),:), NDB(K(MinE,2),:), NDB(K(MinE,3),:), Px(i), Py(i), dimX);

    figure('Position', get(0, 'Screensize'))
    image([0 dimX],[dimY 0],P);
    set(gca,'ydir','normal'); hold on;
    scatter(NDBx,NDBy,200,'r','MarkerFaceColor','m'); hold on;
    scatter(Px,Py); hold on;
    scatter(x,y); hold on;
    plot(x,y,'k','LineWidth',2); hold on;
    plot(polyout,'FaceColor','blue'); hold on;
    plot(Lx1,Ly1); hold on;
    plot(Lx2,Ly2);
    xlim([0 dimX]);
    ylim([0 dimY]);
    axis off;
end

% Function for calculating the Error and the lines from the NDBs
function [ERROR, polyout, Lx1, Ly1, Lx2, Ly2] = CalculateLocationError(NDB1, NDB2, NDB3, x, y, dimX)
    % Coordinates of the NDBs
    NDB = [NDB1; NDB2; NDB3];

    NDBx = NDB(:,1);
    NDBy = NDB(:,2);

    % Calculate the lines of the error angle
    % Considers a constant error of 5 degrees
    error = 5;

    for i = 1:3

        Px(:,i) = [x,NDBx(i)];
        Py(:,i) = [y,NDBy(i)];

        % Distance NDB-plane
        D(i) = sqrt((Px(2,i)-Px(1,i))^2+(Py(2,i)-Py(1,i))^2);

        % The slope of NDB-plane lines
        slope(i) = (Py(2,i) - Py(1,i)) ./ (Px(2,i) - Px(1,i));
        angle(i) = atand(slope(i));

        % Finds out the slope of the error lines
        angle2(i) = angle(i)-error/2;
        slope2(i) = tand(angle2(i));

        angle3(i) = angle(i)+error/2;
        slope3(i) = tand(angle3(i));

        % Makes the error lines
        v = linspace(-dimX, dimX+50);
        w(:,i) = slope2(i)*(v-Px(2,i))+Py(2,i);
        z(:,i) = slope3(i)*(v-Px(2,i))+Py(2,i);

        % Makes a perpendicular line to NBD-plane line fo find intersections with 
        % error lines for making the triangle
        g(:,i) = (-1/slope(i))*(v-x)+y;

        % Makes he lines of the triangles
        l1=[v(1) w(1,i) v(100) w(100,i)];
        l2=[v(1) g(1,i) v(100) g(100,i)];
        [Ex(1,i),Ey(1,i)] = lineIntersect(l1,l2);

        l1=[v(1) z(1,i) v(100) z(100,i)];
        l2=[v(1) g(1,i) v(100) g(100,i)];
        [Ex(2,i),Ey(2,i)] = lineIntersect(l1,l2);

        Lx1(:,i) = [Ex(1,i),NDBx(i)];
        Ly1(:,i) = [Ey(1,i),NDBy(i)];

        Lx2(:,i) = [Ex(2,i),NDBx(i)];
        Ly2(:,i) = [Ey(2,i),NDBy(i)];

        % Elongates the lines to have more range

        d1(i) = sqrt((Lx1(2,i)-Lx1(1,i))^2+(Ly1(2,i)-Ly1(1,i))^2);
        d2(i) = sqrt((Lx2(2,i)-Lx2(1,i))^2+(Ly2(2,i)-Ly2(1,i))^2);

        % Elongates for a certain range
        r = 1000;

        e1 = r/d1(i);
        e2 = r/d2(i);

        Lx1(:,i) = [NDBx(i)+ e1*(Ex(1,i)-NDBx(i)),NDBx(i)];
        Ly1(:,i) = [NDBy(i)+ e1*(Ey(1,i)-NDBy(i)),NDBy(i)];

        Lx2(:,i) = [NDBx(i)+ e2*(Ex(2,i)-NDBx(i)),NDBx(i)];
        Ly2(:,i) = [NDBy(i)+ e2*(Ey(2,i)-NDBy(i)),NDBy(i)];

    end

    % Find out one polygon
    Ix = [Lx1(2,1) Lx1(1,1) Lx2(1,1)];
    Iy = [Ly1(2,1) Ly1(1,1) Ly2(1,1)];

    pgon1 = polyshape(Ix,Iy);

    % Find out the second polygon
    Ix = [Lx1(2,2) Lx1(1,2) Lx2(1,2)];
    Iy = [Ly1(2,2) Ly1(1,2) Ly2(1,2)];

    pgon2 = polyshape(Ix,Iy);

    % Find out the third polygon
    Ix = [Lx1(2,3) Lx1(1,3) Lx2(1,3)];
    Iy = [Ly1(2,3) Ly1(1,3) Ly2(1,3)];

    pgon3 = polyshape(Ix,Iy);

    % Find out the intersection of the polygons
    polyout = intersect(pgon1,pgon2);
    polyout = intersect(pgon3,polyout);

    % Calculate the area of the final polygon
    ERROR = area(polyout);

end
 
% Function for calculating the intersection of two lines
% Function not implemented by Mario Terres Diaz
function [x,y]=lineIntersect(l1,l2)
 
    % Default values for x and y, in case of error these are the outputs
    x = nan;
    y = nan;

    % Test if the user provided the correct number of arguments
    if nargin~=2
        disp('You need to provide two arguments')
        return
    end

    % Get the information about each arguments
    l1info=whos('l1');
    l2info=whos('l2');

    % Test if the arguments are numbers
    if (~((strcmp(l1info.class,'double') & strcmp(l2info.class,'double'))))
        disp('You need to provide two vectors')
        return
    end

    % Test if the arguments have the correct size
    if (~all((size(l1)==[1 4]) & (size(l2)==[1 4])))
        disp('You need to provide vectors with one line and four columns')
        return
    end
    try
    ml1 = (l1(4)-l1(2))/(l1(3)-l1(1));
    ml2 = (l2(4)-l2(2))/(l2(3)-l2(1));
    bl1 = l1(2)-ml1*l1(1);
    bl2 = l2(2)-ml2*l2(1);
    b = [bl1 bl2]';
    a = [1 -ml1; 1 -ml2];
    Pint = a\b;

    % When the lines are paralel there's x or y will be Inf
    if (any(Pint==Inf))
        disp('No solution found, probably the lines are paralel')
        return
    end

    % Put the solution inside x and y
    x = Pint(2);
    y = Pint(1);

    % Find maximum and minimum values for the final test
    l1minX = min([l1(1) l1(3)]);
    l2minX = min([l2(1) l2(3)]);
    l1minY = min([l1(2) l1(4)]);
    l2minY = min([l2(2) l2(4)]);
    l1maxX = max([l1(1) l1(3)]);
    l2maxX = max([l2(1) l2(3)]);
    l1maxY = max([l1(2) l1(4)]);
    l2maxY = max([l2(2) l2(4)]);

    % Test if the intersection is a point from the two lines because 
    %all the performed calculations where for infinite lines 
    if ((x<l1minX) | (x>l1maxX) | (y<l1minY) | (y>l1maxY) |...
           (x<l2minX) | (x>l2maxX) | (y<l2minY) | (y>l2maxY) )
        x = nan;
        y = nan;
        return
    end
    catch err

    end
end