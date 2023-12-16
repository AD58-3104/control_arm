clear;

x = [0 0 0 4.32987e-14 4.32987e-14 4.32987e-14 ];
y = [0 0 0 0 190 256.5 ];
z = [0 24 24 219 219 219 ];

grid on;

plot3(x,y,z,'-o');
title('My Plot');
xlabel('x');
ylabel('y');
zlabel('z');
xlim([-100 100]);
% ylim([-5 10]);

inp = input('Press any key to finish...','s');