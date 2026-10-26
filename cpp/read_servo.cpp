
#include <iostream>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int read_position(int fd, int id) {
    for (int attempt = 0; attempt < 2; attempt++) {
        tcflush(fd, TCIFLUSH);   // 清掉缓冲区里残留的旧字节
        unsigned char tx[8] = {0xFF, 0xFF, (unsigned char)id, 0x04, 0x02, 0x38, 0x02, 0};
        tx[7] = 255 - (id + 64);
        write(fd, tx, 8);

        usleep(3000);   // 等完整响应到齐，避免 read 只读一半

        unsigned char rx[16] = {0};
        int n = read(fd, rx, sizeof(rx));
        if (n >= 8 && rx[0] == 0xFF && rx[1] == 0xFF && rx[2] == id) {
            return rx[5] + (rx[6] << 8);
        }
        // 第一次读坏了，循环再来一次
    }
    return -1;   // 两次都失败，返回无效值
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
void move_to(int fd, int id, int target, int steps) {
    int current = read_position(fd, id);
    if (current < 0 || current > 4095) {
        std::cerr << "读当前位置失败，中止移动\n";
        return;
    }
    write_goal(fd, id, current);
    write_torque(fd, id, true);
    for (int i = 1; i <= steps; i++) {
        int g = current + (target - current) * i / steps;
        write_goal(fd, id, g);
        usleep(20000);
    }
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



int main() {
    int fd = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) { std::cerr << "打不开串口\n"; return 1; }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(fd, &tty);

    cfsetospeed(&tty, B1000000);   // 发送波特率 1M
    cfsetispeed(&tty, B1000000);   // 接收波特率 1M

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8 数据位
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);  // 无校验、1 停止位、无硬件流控
    tty.c_cflag |= (CLOCAL | CREAD);              // 忽略 modem、启用接收
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw：关行缓冲/回显/信号
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);       // 关软件流控
    tty.c_oflag &= ~OPOST;                        // 关输出处理
    tty.c_cc[VMIN] = 1;   // 至少收 1 字节 read 才返回
    tty.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &tty);


    int current[6] = {0};
    if (read_all(fd, current) < 0) {
        std::cerr << "读取初始位置失败，不操作\n";
        close(fd);
        return 1;
    }

    int target[6];
    for (int i = 0; i < 6; i++) target[i] = current[i] + 100;

    std::cout << "开始 6 电机同步点到点（各 +10 格，20 步）\n";
    move_all(fd, target, 20);

    for (int id = 1; id <= 6; id++) {
        int count = read_position(fd, id);
        if (count < 0 || count > 4095) {
            std::cout << "电机 " << id << ": 读失败，跳过\n";
            continue;
        }
        double deg = (count - 2048) * 360.0 / 4096.0;
        std::cout << "电机 " << id << ": 格数=" << count << " 角度=" << deg << " 度\n";
    }
    close(fd);
    return 0;

}
