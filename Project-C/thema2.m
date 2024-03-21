clear all;

e=10^(-2);
gamma=0.5;
sk=0.5;
x(1)=-5;
y(1)=10;

xbar = 0;
ybar = 0;

% Define the function
f = @(x, y) (1/3)*x.^2 + 3*y.^2;

% Derivatives
df_dx = @(x, y) (2.*x)/3;
df_dy= @(x, y) 6*y;

% Create a meshgrid for contour plot
[x_mesh, y_mesh] = meshgrid(-10:0.1:10, -10:0.1:10);
z = f(x_mesh, y_mesh);

figure(1);
% Plot contour lines
contour(x_mesh, y_mesh, z, 50, 'LineWidth', 1.5);
hold on;

% Gradient calculation
grad=[df_dx(x(1),y(1)); df_dy(x(1),y(1))];

k=1;

maxIter = 1500;

while norm(grad) > e && k < maxIter  
    
    %fprintf('norm = %d \n',norm(xk-xb));
    
    store_k(k) = k;
    store_f(k) = f(x(k),y(k));
   
    xbar = x(k)-sk*grad(1);
    ybar = y(k)-sk*grad(2); 
    
    % projection
    if xbar <= -10
        xbar = -10;
    elseif xbar >= 5 
        xbar = 5;
    end
        
    if ybar <= -8
        ybar = -8;
    elseif ybar >= 12 
        ybar = 12;
    end
        
    x(k+1) = x(k) + gamma * (xbar - x(k));
    y(k+1) = y(k) + gamma * (ybar - y(k));
     
    plot(x(k:k+1), y(k:k+1), '-o');  % Connect points with lines
    hold on;
        
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

colorbar

hold on;
plot3(x, y, f(x,y), '-o', 'LineWidth', 2);

title("3D Plot of f(x,y)");
xlabel('x');
ylabel('y');
zlabel('f(x, y)');

hold off;


store_k(k) = k;
store_f(k) = f(x(k),y(k));
hold on;
figure(3);
plot(store_k,store_f,'-s')
xlabel('k')
ylabel('f(x,y)')
title("Iterations");
hold off;
