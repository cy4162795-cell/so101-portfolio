#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <chrono>
#include "so101_cpp_node/srv/move_to_target.hpp"

// 读一个电机的当前位置（格数），两次尝试都失败返回 -1
int read_position(int fd, int id) {
    for (int attempt = 0; attempt < 2; attempt++) {
        tcflush(fd, TCIFLUSH);   // 清掉缓冲区里残留的旧字节
        unsigned char tx[8] = {0xFF, 0xFF, (unsigned char)id, 0x04, 0x02, 0x38, 0x02, 0};
        tx[7] = 255 - (id + 64);
        write(fd, tx, 8);

        usleep(3000);   // 等完整响应到齐
        unsigned char rx[16] = {0};
        int n = read(fd, rx, sizeof(rx));
        if (n >= 8 && rx[0] == 0xFF && rx[1] == 0xFF && rx[2] == id) {
            return rx[5] + (rx[6] << 8);
        }
    }
    return -1;
}

// 读 6 个电机当前格数，存进 current[6]；返回 0 成功、-1 失败
int read_all(int fd, int current[6]) {
    for (int id = 1; id <= 6; id++) {
        int v = read_position(fd, id);
        if (v < 0 || v > 4095) {
            std::cerr << "电机 " << id << " 读当前位置失败\n";
            return -1;
        }
        current[id - 1] = v;
    }
    return 0;
}



void write_torque(int fd,int id ,bool on){
  unsigned char val = on ? 1:0;
  unsigned char tx[8] = {0xFF,0xFF,(unsigned char)id, 0x04, 0x03, 0x28, val, 0};
  tx[7] = 255 - (id + 4 + 3 + 40 + val);
  write(fd,tx,8);
}
void write_goal(int fd,int id,int goal){
  unsigned char low = goal & 0xFF;
  unsigned char high = (goal>>8) &0xFF;
  unsigned char tx[9] = {0xFF, 0xFF, (unsigned char)id, 0x05, 0x03, 0x2A, low, high, 0};
  tx[8] = 255 - (id + 5 + 3 + 42 + low + high);
  write(fd, tx, 9);
}

// 6 电机同步点到点：外层「第几拍」、内层「6 个电机」，同一拍各走一小步
void move_all(int fd, int target[6], int steps) {
    int current[6] = {0};
    if (read_all(fd, current) < 0) {
        std::cerr << "读当前 6 电机失败，中止\n";
        return;
    }

    // 安全四步 1：写 goal = 当前值
    for (int id = 1; id <= 6; id++) write_goal(fd, id, current[id - 1]);
    usleep(200000); tcflush(fd, TCIFLUSH);

    // 安全四步 2：全上电
    for (int id = 1; id <= 6; id++) write_torque(fd, id, true);
    usleep(200000); tcflush(fd, TCIFLUSH);

    // 安全四步 3：同步插值——外层拍、内层电机
    for (int j = 1; j <= steps; j++) {
        for (int id = 1; id <= 6; id++) {
            int g = current[id - 1] + (target[id - 1] - current[id - 1]) * j / steps;
            write_goal(fd, id, g);
        }
        usleep(20000);
    }
    usleep(200000); tcflush(fd, TCIFLUSH);

    // 安全四步 4：全断电
    for (int id = 1; id <= 6; id++) write_torque(fd, id, false);
    usleep(200000); tcflush(fd, TCIFLUSH);
}

class JointStatePublisher : public rclcpp::Node {
public:
    JointStatePublisher() : Node("joint_state_publisher") {
        pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&JointStatePublisher::publish_cb, this));
        
        move_srv_ = this->create_service<so101_cpp_node::srv::MoveToTarget>(
                    "/move_to",
                    std::bind(&JointStatePublisher::move_cb, this,
                    std::placeholders::_1, std::placeholders::_2));
        fd_ = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "打不开串口 /dev/ttyACM1");
            return;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        tcgetattr(fd_, &tty);
        cfsetospeed(&tty, B1000000);
        cfsetispeed(&tty, B1000000);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 0;
        tcsetattr(fd_, TCSANOW, &tty);
    }

    ~JointStatePublisher() {
        if (fd_ >= 0) close(fd_);
    }

private:
    void move_cb(const std::shared_ptr<so101_cpp_node::srv::MoveToTarget::Request> req,
                std::shared_ptr<so101_cpp_node::srv::MoveToTarget::Response> res) {
        // 1. 弧度目标 → 格数，并校验是否在 0~4095
        int goal[6];
        for (int i = 0; i < 6; i++) {
            goal[i] = (int)std::lround(2048.0 + req->target[i] * 4096.0 / (2.0 * 3.14159265358979323846));
            if (goal[i] < 0 || goal[i] > 4095) {
                res->success = false;
                res->message = "目标超出关节范围";
                return;
            }
        }
        // 2. 校验步数
        if (req->steps <= 0 || req->steps > 200) {
            res->success = false;
            res->message = "步数超出范围（1~200）";
            return;
        }
        // 3. 串口没开就拒绝
        if (fd_ < 0) {
            res->success = false;
            res->message = "串口未打开";
            return;
        }
        // 4. 真的让臂动（会阻塞约一秒，期间真机在动）
        move_all(fd_, goal, req->steps);
        res->success = true;
        res->message = "移动完成";
    }


    void publish_cb() {
        if (fd_ < 0) return;

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = {"shoulder_pan", "shoulder_lift", "elbow_flex",
                    "wrist_flex", "wrist_roll", "gripper"};
        msg.position.clear();
        for (int id = 1; id <= 6; id++) {
            int count = read_position(fd_, id);
            if (count < 0 || count > 4095) continue;   // 读失败就跳过，不塞垃圾值
            double rad = (count - 2048) * 2.0 * 3.14159265358979323846 / 4096.0;
            msg.position.push_back(rad);
        }
        pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int fd_ = -1;

    rclcpp::Service<so101_cpp_node::srv::MoveToTarget>::SharedPtr move_srv_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
