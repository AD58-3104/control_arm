#ifndef HELPER_FUNC_HPP
#define HELPER_FUNC_HPP
#include <Eigen/Dense>
#include <boost/asio.hpp>

/**
 * @brief 対象のx,yの位置への距離を求める
 * @param target_pos 対象の位置
 * @return double 距離
 */
double calcDistanceXY(const Eigen::Vector3d &target_pos)
{
    return std::sqrt(std::pow(target_pos(0), 2) + std::pow(target_pos(1), 2));
}

/**
 * @brief 対象のx,yの位置への角度を求める
 * @param target_pos 対象の位置
 * @return double 角度[rad]
 */
double calcAngleXY(const Eigen::Vector3d &target_pos)
{
    return std::atan2(target_pos(1), target_pos(0));
}

struct MagnetController
{
    boost::asio::io_context ioc_;
    boost::asio::serial_port port_;
    MagnetController() : ioc_(), port_(ioc_)
    {
        port_.open("/dev/ttyUSB0");
        port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        port_.set_option(boost::asio::serial_port_base::baud_rate(100000));
        port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        sleep(1);
    }
    ~MagnetController()
    {
        port_.close();
    }
    bool write_some(const std::vector<uint8_t> &buffers)
    {
        int write_len =  port_.write_some(boost::asio::buffer(buffers, buffers.size()));
        return (write_len == buffers.size());
    }
    enum magnet_state : uint8_t
    {
        ENABLE_UP = 0b00000001,
        DISABLE_UP = 0b00000010,
        ENABLE_DOWN = 0b00000100,
        DISABLE_DOWN = 0b00001000
    };
    bool control(uint8_t desired_state){
        uint8_t data = 0;
        if(desired_state & ENABLE_UP){
            data |= ENABLE_UP;
        }
        if(desired_state & DISABLE_UP){
            data |= DISABLE_UP;
        }
        if(desired_state & ENABLE_DOWN){
            data |= ENABLE_DOWN;
        }
        if(desired_state & DISABLE_DOWN){
            data |= DISABLE_DOWN;
        }
        std::vector<uint8_t> sendbuf;
        sendbuf.push_back(data);
        return write_some(sendbuf);
    }
};

#endif // !HELPER_FUNC_HPP