clear all;

e=10^(-3);
x(1)=2;
y(1)=3;

% Define the function
f = @(x, y) (1/3).*x.^2+3.*y.^2;

% Derivatives
df_dx= @(x, y) (2.*x)./3;
df_dy= @(x, y) 6.*y;

% Create a meshgrid for contour plot
[x_mesh, y_mesh] = meshgrid(-3:0.1:3, -3:0.1:3);
z = f(x_mesh, y_mesh);

figure(1);
% Plot contour lines
contour(x_mesh, y_mesh, z, 50, 'LineWidth', 1.5);
hold on;

% gradient of f
grad=[df_dx(x(1),y(1)); df_dy(x(1),y(1))];

k=1;

% CONSTANT GAMMA MINIMIZATION METHOD

gamma=0.25;

while norm(grad)>e
    %fprintf('norm = %d \n',norm(grad));

    store_k(k) = k;
    store_f(k) = f(x(k),y(k));
    
    d(k, :) = -grad;
    
    % Update x and y based on the optimal gamma
    x(k+1) = x(k) + gamma * d(k, 1);
    y(k+1) = y(k) + gamma * d(k, 2);

     % Plot the current point
    plot(x(k:k+1), y(k:k+1), '-o');  % Connect points with lines
    hold on;
    
    % Compute the gradient at the new point
    grad = [df_dx(x(k+1), y(k+1)); df_dy(x(k+1), y(k+1))];
    
    k = k + 1;
    
end


xlabel('x');
ylabel('y');
title('Optimization Path');
grid on;

% Create a grid of x and y values
[p, g] = meshgrid(-3:0.1:3, -3:0.1:3);

% Evaluate the function on the grid
z = f(p, g);

% Plot the surface
figure(2);
%figure;
h = fsurf(f, [-3 3 -3 3]);
%view(45, 0);
colorbar

hold on;
x_highlight = x(end);
y_highlight = y(end);
z_highlight = f(x(end),y(end));
scatter3(x_highlight, y_highlight, z_highlight, 100, 'r', 'filled'); % Red marker, replace 100 with your desired marker size
hold off;

title("3D Plot of f(x,y)");
xlabel('x');
ylabel('y');
zlabel('f(x, y)');


store_k(k) = k;
store_f(k) = f(x(k),y(k));
hold on;
figure(3);
plot(store_k,store_f,'-s')
xlabel('k')
ylabel('f(x,y)')
title("Iterations");
hold off;
