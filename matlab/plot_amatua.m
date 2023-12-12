% syms theta
% import Robotics

clear;

% オイラー角 Z Y X 
eul = [0 pi/2 pi/2];
R = eul2rotm(eul);


origin_para_theta = pi/3;
Ro = rotm2tform(eul2rotm([origin_para_theta 0 0]));
To = trvec2tform([0 0 0]);

link0_para_a = 3;
link0_para_theta = -pi/2;
R0 = rotm2tform(eul2rotm([link0_para_theta 0 0]));
T0 = trvec2tform([link0_para_a 0 0.3]);

link1_para_a = 2;
link1_para_theta = pi/5;
R1 = rotm2tform(eul2rotm([link1_para_theta 0 0]));
T1 = trvec2tform([link1_para_a 0 0]);

link2_para_a = 1;
link2_para_theta = pi;
R2 = rotm2tform(eul2rotm([link2_para_theta 0 0]));
T2 = trvec2tform([link2_para_a 0 0]);

link0  = To * Ro * T0 * R0
link1  = link0 * T1 * R1
link2  = link1 * T2 * R2

% 前がx後ろがy,最後がz
result_0 = tform2trvec(link0)
result_1 = tform2trvec(link1)
result_2 = tform2trvec(link2)

result_x = [0 result_0(1) result_1(1) result_2(1)]
result_y = [0 result_0(2) result_1(2) result_2(2)]
result_z = [0 result_0(3) result_1(3) result_2(3)]

% プロットするとこ
grid on;
plot3(result_x,result_y,result_z, '-o');
title('My Plot');
xlabel('x');
ylabel('y');
zlabel('z');
xlim([0 10]);
ylim([-5 10]);
% syms theta
% import Robotics

clear;

% オイラー角 Z Y X 
eul = [0 pi/2 pi/2];
R = eul2rotm(eul);


origin_para_theta = pi/3;
Ro = rotm2tform(eul2rotm([origin_para_theta 0 0]));
To = trvec2tform([0 0 0]);

link0_para_a = 3;
link0_para_theta = -pi/2;
R0 = rotm2tform(eul2rotm([link0_para_theta 0 0]));
T0 = trvec2tform([link0_para_a 0 0]);

link1_para_a = 2;
link1_para_theta = pi/5;
R1 = rotm2tform(eul2rotm([link1_para_theta 0 0]));
T1 = trvec2tform([link1_para_a 0 0]);

link2_para_a = 1;
link2_para_theta = pi;
R2 = rotm2tform(eul2rotm([link2_para_theta 0 0]));
T2 = trvec2tform([link2_para_a 0 0]);

link0  = To * Ro * T0 * R0
link1  = link0 * T1 * R1
link2  = link1 * T2 * R2

% 前がx後ろがy,最後がz
result_0 = tform2trvec(link0)
result_1 = tform2trvec(link1)
result_2 = tform2trvec(link2)

result_x = [0 result_0(1) result_1(1) result_2(1)]
result_y = [0 result_0(2) result_1(2) result_2(2)]
result_z = [0 result_0(3) result_1(3) result_2(3)]

% プロットするとこ
grid on;
plot3(result_x,result_y,result_z, '-o');
title('My Plot');
xlabel('x');
ylabel('y');
zlabel('z');
xlim([0 10]);
zlim([-3 3]);