#ifndef HELPER_FUNC_HPP
#define HELPER_FUNC_HPP
#include "robot_arm.hpp"
#include <sciplot/sciplot.hpp>
#include <fstream>
#include <random>
#include <vector>
#include <ctime>

void plotArm(const ResultPosition data)
{
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
        << std::endl
        << "grid on;"
        << std::endl
        << "plot3(x,y,z,'-o');"
        << std::endl
        << "title('My Plot');"
        << std::endl
        << "xlabel('x');"
        << std::endl
        << "ylabel('y');"
        << std::endl
        << "zlabel('z');"
        << std::endl
        << "xlim([-100 100]);"
        << std::endl
        << "% ylim([-5 10]);"
        // << std::endl << "plot(y,z,'-o');"
        // << std::endl << "ylabel('y');zlabel('z');"
        ;
    ofs.close();
    std::system("octave --persist pltarm.m > /dev/null 2>&1");

    return;
}

/**
 * @brief Get the Random Joint Degree object
 * @param num 生成する個数
 * @return std::vector<double>　角度[rad]
 */
std::vector<double> getRandomJointDegree(const size_t num)
{
    std::vector<double> result{};
    static uint32_t count = 0;
    if (count == 0)
    {
        for (size_t i = 0; i < num; i++)
        {
            //全部0をやる
            result.push_back(0);
        }
        count++;
        return result;
    }
    std::random_device seed_gen("/dev/random");
    std::mt19937 engine(seed_gen());
    std::uniform_real_distribution<> dist(0, 360);
    std::cout << "random joint degree:";
    for (size_t i = 0; i < num; i++)
    {
        result.push_back(dist(engine) * M_PI / 360.0f);
        std::cout << "," << result[i];
    }
    std::cout << std::endl;
    count++;
    return result;
}

#endif // !HELPER_FUNC_HPP