#ifndef HELPER_FUNC_HPP
#define HELPER_FUNC_HPP
#include "robot_arm.hpp"
#include <sciplot/sciplot.hpp>
#include <fstream>


void plotArm(const ResultPosition data){
    std::ofstream ofs("pltarm.m");
    ofs << "x = [";
    for (auto &x : data.x)
    {
        ofs << x << " ";
    }
    ofs << "];" << std::endl;
    ofs << "y = [";
    for (auto &y : data.y)
    {
        ofs << y << " ";
    }
    ofs << "];" << std::endl;
    ofs << "z = [";
    for (auto &z : data.z)
    {
        ofs << z << " ";
    }
    ofs << "];" << std::endl;
    ofs
    << std::endl << "grid on;"
    << std::endl << "plot3(x,y,z,'-o');"
    << std::endl << "title('My Plot');"
    << std::endl << "xlabel('x');"
    << std::endl << "ylabel('y');"
    << std::endl << "zlabel('z');"
    << std::endl << "xlim([-100 100]);"
    << std::endl << "% ylim([-5 10]);"
    << std::endl << "plot(y,z,'-o');"
    ;
    ofs.close();
    std::system("octave --persist pltarm.m > /dev/null 2>&1");

    return;
}

#endif // !HELPER_FUNC_HPP