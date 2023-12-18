#ifndef ROBOT_ARM_HPP
#define ROBOT_ARM_HPP
#include "dxl_motor.hpp"
#include "robot_model.hpp"
#include <list>
#include <barrier>
#include <memory>
#include <future>
#include <optional>
#include <ctime>
#include <random>

#include <sciplot/sciplot.hpp>
#include <fstream>
#include <vector>
#include <ctime>

/**
 * @brief Get the Random Joint Degree object
 * @param num 生成する個数
 * @return std::vector<double>　角度[rad]
 */
std::vector<double> getRandomJointDegree(const size_t num)
{
    std::vector<double> result{};
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
    return result;
}

using arm_shared_ptr_t = std::shared_ptr<std::vector<StraightChainRobotModel>>;

static arm_shared_ptr_t robot_arm = nullptr;

arm_shared_ptr_t createRobotArm()
{
    robot_arm = std::make_shared<std::vector<StraightChainRobotModel>>();
    // ロボットモデルはここで定義する
    // 根元の回転軸をzとした左手系で定義。xは右、yは上

    // origin
    robot_arm->push_back(StraightChainRobotModel(1, 0, Eigen::Vector3d::Zero(), 24, Eigen::Vector3d::Zero()));
    // 根元回転
    robot_arm->push_back(StraightChainRobotModel(2, 0, Eigen::Vector3d(0, -M_PI / 2.0, 0), 0, Eigen::Vector3d::Zero()));
    // id2のクローン.
    robot_arm->back().clone_motors.push_back(StraightChainRobotModel(2 + clone_id_offset, true));
    // 次のリンク
    // ここyへの22.5も足さなくてはいけない...。どうしよう
    robot_arm->push_back(StraightChainRobotModel(3, 195, Eigen::Vector3d(0, 0, 0), 0, Eigen::Vector3d(0, 0, M_PI / 2)));
    robot_arm->back().setYoffset(22.5); // だいぶ小手先だが仕方ない...
    // 次のリンク
    robot_arm->push_back(StraightChainRobotModel(4, 190, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    // 手先のリンク
    robot_arm->push_back(StraightChainRobotModel(5, 66.5, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    // robot_arm->push_back(StraightChainRobotModel(5, 60, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    for(auto& motor : *robot_arm){
        motor.setJointAngle(0);    //JointAngleを0にしておく
    }
    return robot_arm;
}

arm_shared_ptr_t getRobotArm()
{
    assert(robot_arm != nullptr); // robot_armが初期化されていない
    return robot_arm;
}

/**
 * @brief アームを動かす
 * @param robot_arm
 * @param motors
 */
void moveArm(dxl_motor &motors)
{
    if (robot_arm == nullptr)
    {
        throw std::runtime_error("robot_arm is not initialized!");
    }
    for (auto &motor : *robot_arm)
    {
        int16_t position = motor.joint_angle_ * 180 / M_PI; // ラジアンから戻す
        // if (motor.does_reverse_)
        // {
        //     position *= -1.0f;    //setJointAngleの方で逆にした
        // }
        uint16_t raw_dxl_position = std::max(DXL_CENTOR_POSITION + position, 0);
        std::cout << "[move data] raw dxl = " << raw_dxl_position << std::endl;
        // クローン動作のモータがある場合
        if (!motor.clone_motors.empty())
        {
            // 基のモータのデータを追加
            std::vector<dxl_motor_sync_moves> move_data;
            move_data.push_back({motor.motor_id_, raw_dxl_position});
            for (auto &clone_motor : motor.clone_motors)
            {
                uint16_t raw_dxl_position_clone = DXL_CENTOR_POSITION + -1.0f * position;
                // クローン動作のモータを追加
                move_data.push_back({clone_motor.motor_id_, raw_dxl_position_clone});
                std::cout << "[move data] raw dxl clone = " << raw_dxl_position_clone << std::endl;
            }
            motors.setSyncGoalPosition(move_data);
        }
        else
        {
            motors.setGoalPosition(motor.motor_id_, raw_dxl_position);
        }
    }
    return;
}

/**
 * @brief アームのリンクそれぞれの位置の結果
 */
struct ResultPosition
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    auto getEndPositionVec()
    {
        return Eigen::Vector3d(x.back(), y.back(), z.back());
    }
    auto getPositionVec(const size_t index)
    {
        return Eigen::Vector3d(x[index], y[index], z[index]);
    }
    void print()
    {
        std::cout << "^^^ ResultPosition  ^^^\n";
        std::cout << "x: ";
        for (auto &data : x)
        {
            std::cout << data << ", ";
        }
        std::cout << "\n";
        std::cout << "y: ";
        for (auto &data : y)
        {
            std::cout << data << ", ";
        }
        std::cout << "\n";
        std::cout << "z: ";
        for (auto &data : z)
        {
            std::cout << data << ", ";
        }
        std::cout << "\n";
    }
};

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

ResultPosition calcForwardKinematics(bool print = false);

void plot2darm(bool reset = false){
    using namespace sciplot;
    auto result = calcForwardKinematics();
    static Plot2D plot2d;
    if(reset){
        plot2d.clear();
    }
    plot2d.legend().hide();
    plot2d.xlabel("x");
    plot2d.ylabel("y");
    plot2d.xrange(0.f, 500.f);
    plot2d.yrange(0.f, 500.f);
    plot2d.drawWithVecs("linespoints", result.y, result.z).lineColor("red");
    Figure fig2d = {{plot2d}};
    Canvas canvas2d = {{fig2d}};
    canvas2d.show();
    return;
}

ResultPosition calcForwardKinematics(bool print)
{
    auto arm_ptr = getRobotArm();
    ResultPosition result;
    result.x.emplace_back(0); // 原点
    result.y.emplace_back(0); // 原点
    result.z.emplace_back(0); // 原点
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
    for (auto &motor : *arm_ptr)
    {
        auto tf = motor.getTransformMatrix();
        res *= tf;
        result.x.emplace_back(res(0, 3));
        result.y.emplace_back(res(1, 3));
        result.z.emplace_back(res(2, 3));
        if (print)
        {
            std::cout << "*** motor id " << (uint16_t)(motor.motor_id_) << " ***" << std::endl;
            std::cout << tf << std::endl;
            std::cout << "--------------------------" << std::endl;
            std::cout << res << std::endl;
        }
    }
    if (print)
    {
        std::cout << "^^^ result ^^^" << std::endl;
        std::cout << res << std::endl;
    }
    return result;
}

/**
 * @brief Set the All Joint Angle object
 * @param joint_angles 各関節角度　[rad]
 */
void setAllJointAngle(const std::vector<double> joint_angles)
{
    assert(robot_arm != nullptr); // robot_armが初期化されていない
    if (joint_angles.size() != robot_arm->size())
    {
        throw std::runtime_error("joint_angles size is not equal to robot_arm size!");
    }
    auto itr = joint_angles.begin();
    for (auto &motor : *robot_arm)
    {
        motor.setJointAngle(*itr);
        itr++;
    }
    return;
}

/**
 * @brief 与えられた目標位置に対するアームの逆運動学を解く
 * @param target_position 目標位置(x,y)
 * @return std::vector<double> 各関節角度[rad]
 * @warning これを実行すると内部で関節の状態を変更してしまうので注意
 * 多分本番と回転の向きが逆だ！！！！！
 */
constexpr double error_threshold = 5.0f;
std::optional<std::vector<double>> calcInverseKinematics(const Eigen::Vector3d target_position)
{
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(0);
    if (robot_arm == nullptr)
    {
        throw std::runtime_error("robot_arm is not initialized!");
    }
    const size_t arm_size = getRobotArm()->size();
    auto convertToDegreeContainer = [](const std::vector<double> rad_data)
    {
        std::vector<double> result;
        for (auto &data : rad_data)
        {
            result.push_back(data * 180 / M_PI);
        }
        return result;
    };
    double minimum_error_norm = 765876346315283;
    bool solved = false;
    std::vector<double> result(arm_size, 0.0f);
    ResultPosition pos_finally;
    size_t iterate = 0;
    for (iterate = 0; iterate < 100; iterate++)
    {
        for (size_t index = arm_size; 1 < index; index--) // 根元は計算しないので、1までしか対象に入れない.ここは1 < である必要あり。何故かというと、Fkineが原点の0,0,0を返してしまうからそれを計算しないようにするため。
        {
            auto res = calcForwardKinematics();
            auto link_end = res.getEndPositionVec();
            auto target_link = res.getPositionVec(index - 1);
            auto target_link_vec = link_end - target_link;
            auto origin_link_vec = target_position - target_link;
            double target_angle = acos(target_link_vec.dot(origin_link_vec) / (target_link_vec.norm() * origin_link_vec.norm()));
            auto muki = target_link_vec.cross(origin_link_vec).normalized();
            // std::cout << "muki: " << muki << std::endl;
            // 回転の軸を決める
            if (muki(0) >= 0)
            {
                target_angle *= -1.0f; // zの回転軸が自分が最初設定したつもりのものと逆だったのでx方向へのプラスのベクトルで逆回転になる。
            }
            getRobotArm()->at(index - 1).setRelativeJointAngle(target_angle);
            result[index - 1] = target_angle * -1.0f;
            // std::cout << "index: " << index - 1 << std::endl;
            // std::cout << "target_angle: " << target_angle * 180 /M_PI << std::endl;
            // setAllJointAngle(convertToDegree(result));
        }
        auto res = calcForwardKinematics();
        auto link_end = res.getEndPositionVec();
        auto error = target_position - link_end;
        if (std::abs(minimum_error_norm - error.norm()) < 0.1f)
        {
            // 更新が無い場合
            solved = false;
            break;
        }
        minimum_error_norm = std::min(minimum_error_norm, error.norm());
        if (minimum_error_norm < error_threshold)
        {
            std::cout << "Loop count: " << iterate << "\n";
            std::cout << "[Target vector] " << target_position.transpose() << "\n";
            std::cout << "[End link vector] " << link_end.transpose() << "\n";
            std::cout << "OK error is small!!  "
                      << "\n";
            solved = true;
            if (iterate > 200)
            {
                throw std::runtime_error("めちゃ時間かかってるね！！！");
            }
            break;
        }
    }
    std::cout << "----------- [Inverse kinematics Result] -----------"
              << "\n";
    std::cout << "[Target position] :: " << target_position.transpose() << "\n";
    std::cout << "[Loop count] :: " << iterate << "\n";
    std::cout << "[Minimum error norm] :: " << minimum_error_norm << "\n";
    pos_finally.print();
    std::cout << "[Joint degrees] ";
    for (const auto deg : result)
    {
        std::cout << deg << "\n";
    }

    if (solved)
    {
        return result;
    }
    else
    {
        return std::nullopt;
    }
}

bool checkYZisnotMinus(const ResultPosition &pos)
{
    for (size_t ind = 0; ind < pos.z.size(); ind++)
    {
        if (pos.z[ind] < -0.1) // 誤差考慮
        {
            return false;
        }
        if (pos.y[ind] < -0.1)
        { // 誤差考慮
            return false;
        }
        ind++;
    }
    return true;
}

/**
 * @brief 順運動学の結果からベクトル同士で各関節の角度を計算する
 * @return std::vector<double> 
 */
std::vector<double> getCalcJointAngle()
{
    std::vector<double> result;
    std::cout << "[getJointAngle]\nraw joint angle ";
    for (auto &motor : *getRobotArm())
    {
        std::cout << motor.joint_angle_ * 180.f / M_PI<< ", ";
    }
    std::cout << std::endl;
    auto links = calcForwardKinematics();
    std::cout << "Local joint angle ";
    for (size_t index = 1; index < links.x.size() - 2; index++)
    {
        Eigen::Vector3d first_link = links.getPositionVec(index + 1)  - links.getPositionVec(index);
        Eigen::Vector3d second_link = links.getPositionVec(index + 2) - links.getPositionVec(index + 1);
        double target_angle = acos(second_link.dot(first_link) / (second_link.norm() * first_link.norm()));
        result.push_back(target_angle);
        std::cout << target_angle * 180.f / M_PI << ", ";
    }
    std::cout << std::endl;
    links.print();
    return result;
}

/**
 * @brief 今設定されている関節角度を取得する
 * @return std::vector<double> 角度[rad]
 */
std::vector<double> getJointAngleParam(){
    std::vector<double> result;
    for (auto &motor : *getRobotArm())
    {
        result.push_back(motor.joint_angle_);
    }
    return result;
}



/**
 * @brief CCDで逆運動学を解くが、初期姿勢をランダムにして繰り返す処理もやる
 * 
 * @param target_pos 目標位置
 * @return std::optional<std::vector<double>> 解けた場合は各関節角度 [rad]
 */
std::optional<std::vector<double>> solveCCDINV(const Eigen::Vector3d target_pos)
{
    ResultPosition result;
    std::vector<double> degs;
    size_t yarinaoshi_count = 0;
    constexpr size_t yarinaoshi_threshould = 10000;
    std::optional<std::vector<double>> solution;
    while (true)
    {
        auto joi = getRandomJointDegree(getRobotArm()->size());
        joi[0] = 0;            // 根元の回転はしてほしくないので0にする
        setAllJointAngle(joi); // ランダムな初期姿勢を設定
        solution = calcInverseKinematics(target_pos);
        if (solution.has_value())
        {
            result = calcForwardKinematics();
            if (checkYZisnotMinus(result))
            {
                break;
            }
        }
        yarinaoshi_count++;
        if (yarinaoshi_count > yarinaoshi_threshould)
        {
            std::cerr << "<<<<< Cannot solve problem!!! >>>>> " << std::endl;
            return std::nullopt;
        }
    }
    std::cout << "------------- [Calculation is finished] -------------" << std::endl;
    std::cout << "--- End position ---" << std::endl;
    std::cout << result.getEndPositionVec().transpose() << std::endl;
    result.print();
    std::cout << "[yarinaoshi_count] :" << yarinaoshi_count << std::endl;
    if (solution.has_value())
    {
        std::cout << "[CCD solution degrees] :";
        for (auto &deg : solution.value())
        {
            std::cout << deg * 180.0f / M_PI << ", ";
        }
        std::cout << std::endl;
    }
    return solution;
}

#endif // !ROBOT_ARM_HPP