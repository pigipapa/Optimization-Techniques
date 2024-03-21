f = @(x, y) (1/3).*x.^2+3.*y.^2;

% Create a grid of x and y values
[x, y] = meshgrid(-5:0.1:5, -5:0.1:5);

% Evaluate the function on the grid
z = f(x, y);

% Plot the surface
figure(1);
%figure;
fsurf(f, [-5 5 -5 5]);
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