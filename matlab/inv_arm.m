clear;
ez = [0;0;1]

target_location = [3.2,2.8]

for i = 1:100
    rot_e = [0; pi/5; pi/6]

    Ro = eul2rotm([rot_e(1) 0 0]);
    po = [0; 0; 0];

    link0_para_a = 2;
    R0 = eul2rotm([rot_e(2) 0 0]);
    p0 = [link0_para_a; 0; 0];
    Ro0 = Ro * R0 

    link1_para_a = 2;
    R1 = eul2rotm([rot_e(3) 0 0]);
    p1 = [link1_para_a; 0; 0];
    Ro1 = Ro * R0 * R1


    base_jaco = [
            cross(Ro*ez,po) cross(Ro0*ez,p0) cross(Ro1*ez,p1);
            Ro*ez    Ro0*ez    Ro1*ez];
    pinv(base_jaco)
    rot_e = (rot_e - pinv(base_jaco) * rot_e)
end
% link2_para_a = 1;
% link2_para_theta = pi;
% R2 = eul2rotm([link2_para_theta 0 0]);
% T2 = [link2_para_a 0 0];
% 
% link0  = po * Ro * p0 * R0
% link1  = link0 * p1 * R1
% link2  = link1 * T2 * R2

% 前がx後ろがy,最後がz
% result_0 = tform2trvec(link0)
% result_1 = tform2trvec(link1)
% % result_2 = tform2trvec(link2)
% 
% result_x = [0 result_0(1) result_1(1)]
% result_y = [0 result_0(2) result_1(2)]
% result_z = [0 result_0(3) result_1(3)]
% 
% 
% grid on;
% plot(result_x,result_y, '-o');
% title('My Plot');
% xlabel('x');
% ylabel('y');
% xlim([0 10]);
% ylim([-5 10]);