% syms theta
import Robotics

clear;

% オイラー角 Z Y X 
eul = [0 pi/2 pi/2];
R = eul2rotm(eul)


origin_para_theta = pi/2;
%  Ro = [cos(origin_para_theta) -sin(origin_para_theta) 0;sin(origin_para_theta) cos(origin_para_theta) 0;0 0 1;];
Ro = rotm2tform(eul2rotm([origin_para_theta 0 0]));
To = trvec2tform([0 0 0]);

link0_para_a = 2;
link0_para_theta = pi/2;
 % R0 = [cos(link0_para_theta) -sin(link0_para_theta) 0;sin(link0_para_theta) cos(link0_para_theta) 0;0 0 1;];
R0 = rotm2tform(eul2rotm([link0_para_theta 0 0]));
T0 = trvec2tform([link0_para_a 0 0]);

link1_para_a = 2;
link1_para_theta = 0;
 % R1 = [cos(link1_para_theta) -sin(link1_para_theta) 0;sin(link1_para_theta) cos(link1_para_theta) 0;0 0 1;];
% T1 = [R1 [link1_para_a; 0; 0]; 0 0 0 1]
R1 = rotm2tform(eul2rotm([link1_para_theta 0 0]));
T1 = trvec2tform([link1_para_a 0 0]);

res = To * T0 * T1

% 前がx後ろがy,最後がz
origin = [0 0 0];
link0 = [2 0 0];
link1 = [2 0 0];

result = [origin;link0;link1]

result_x = result(: ,1);
result_y = result(: ,2);

% プロットするとこ
plot(result_x,result_y, '-o');
title('My Plot');
xlabel('x');
ylabel('y');
xlim([0 10]);
ylim([0 10]);