f=@(x,y) x.^3*exp(-x.^2-y.^4);

% Create a grid of x and y values
[x, y] = meshgrid(-3:0.1:3, -3:0.1:3);

% Evaluate the function on the grid
z = f(x, y);

% Plot the surface
figure(1);
%figure;
fsurf(f, [-3 3 -3 3]);
colorbar
title("3D Plot of f(x,y)");
xlabel('x');
ylabel('y');
zlabel('f(x, y)');

% contour plot
figure(2);
fcontour(f);
colorbar
title("Function f(x,y)");
xlabel("x");
ylabel("y");