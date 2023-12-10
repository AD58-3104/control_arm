#include <iostream>
#include <cmath>

double target_function(double x)
{
    return x * x * x - 16 + x * x + 4 * x; // √2を求める
}

int main(int argc, char const *argv[])
{
    const size_t count = 100;
    double x = 10000; // 初期値
    for (int i = 0; i < count; i++)
    {
        x = x - target_function(x) / (3 * x * x + 2 * x + 4);
        std::cout << "x = " << x << ", f(x) = " << target_function(x) << std::endl;
        if (target_function(x) < 0.0000000000000001f)
        {
            std::cout << "\ntotal loop count = " << i << std::endl;
            break;
        }
    }
    return 0;
}
