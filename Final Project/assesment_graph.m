f = @(u1,u2) sin(u1 + u2)*sin(u2^2);

[u1, u2] = meshgrid(-1:0.1:2, -2:0.1:1);

fsurf(f, [-1 2 -2 1]);
colorbar
title("3D Plot of f(u1,u2)");
xlabel('u1');
ylabel('u2');
zlabel('f(u1, u2)');

clear all;