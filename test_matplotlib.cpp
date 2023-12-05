#include <iostream>
#include <sciplot/sciplot.hpp>
#include <eigen3/Eigen/Dense>
using namespace sciplot;
using namespace Eigen;
const float arm1_len = 2.0f;
const float arm2_len = 3.0f;

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec)
{
    for (auto& el : vec)
    {
        os << el << ' ';
    }
    return os;
}

int main(int argc, char** argv)
{
    // Create a vector with x-axis values
    // x x1
    // y y1
    Matrix2d arm1;
    arm1 << 0, arm1_len,
            0, 0;

    Matrix2d arm2;
    arm2 << arm1(0,1), arm1(0,1) + arm2_len,
            arm1(1,1), 0;
    
    std::cout << arm1 << std::endl;
    std::cout << arm2 << std::endl;
    std::cout << "-----------------" << std::endl;
    // Create a Plot object
    Plot2D plot;

    // Set the legend
    plot.legend().hide();

    // Set the x and y labels
    plot.xlabel("x");
    plot.ylabel("y");

    // Set the y range
    plot.yrange(0.0f, 5);

    std::vector<float> x1{arm1(0,0),arm1(0,1)},y1{arm1(1,0),arm1(1,1)};
    std::vector<float> x2{arm2(0,0),arm2(0,1)},y2{arm2(1,0),arm2(1,1)};

    std::cout << x1 << std::endl;
    std::cout << y1 << std::endl;

    // Add values to plot
    plot.drawWithVecs("lines",x1,y1).lineColor("red");
    plot.drawWithVecs("lines",x2,y2).lineColor("blue");

    // Create figure to hold plot
    Figure fig = {{plot}};

    // Create canvas to hold figure
    Canvas canvas = {{fig}};

    // Show the plot in a pop-up window
    canvas.show();

}