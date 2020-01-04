#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <stdio.h>
#include <unistd.h>    // UART需要
#include <sys/fcntl.h> // UART需要
#include <termios.h>   // UART需要
#include <string>
#include <bits/stdc++.h>

using namespace boost::python;

#define NSERIAL_CHAR 256
#define VMINX 1
#define BAUDRATE B9600
#define SERVO_FRAME_HEADER 0x55
#define CMD_SERVO_MOVE 3
#define GET_LOW_BYTE(A) (uint8_t)((A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//宏函数 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//宏函数 以A为高八位 B为低八位 合并为16位整形
const char *uart_target = "/dev/ttyTHS1";

// Fucntion to convert a string to integer array
void convertStrtoArr(std::string str, uint16_t *arr)
{
    int j = 0, i, sum = 0;

    for (i = 0; str[i] != '\0'; i++)
    {
        if (str[i] == ' ')
        {
            j++;
        }
        else
        {
            arr[j] = arr[j] * 10 + (str[i] - 48);
        }
    }
}

int moveServo(std::string positionStr, int16_t time)
{
    int fid = -1;
    uint8_t id[6] = {1, 2, 3, 4, 5, 6};
    uint16_t position[6] = {0};
    convertStrtoArr(positionStr, position);
    struct termios port_options;
    tcgetattr(fid, &port_options);
    fid = open(uart_target, O_RDWR | O_NOCTTY);

    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);
    usleep(1000000);

    if (fid == -1)
    {
        printf("Error! Cannot open UART.\n");
    }

    port_options.c_cflag &= ~PARENB;
    port_options.c_cflag &= ~CSTOPB;
    port_options.c_cflag &= ~CSIZE;
    port_options.c_cflag |= CS8;
    port_options.c_cflag &= ~CRTSCTS;
    port_options.c_cflag |= CREAD | CLOCAL;
    port_options.c_iflag &= ~(IXON | IXOFF | IXANY);
    port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    port_options.c_oflag &= ~OPOST;
    port_options.c_lflag = 0;
    port_options.c_cc[VMIN] = VMINX;
    port_options.c_cc[VTIME] = 0;

    cfsetispeed(&port_options, BAUDRATE);
    cfsetospeed(&port_options, BAUDRATE);
    int att = tcsetattr(fid, TCSANOW, &port_options);

    if (att != 0)
    {
        printf("\nERROR in Setting port attributes");
    }
    else
    {
        printf("\nSERIAL Port Good to Go.\n");
    }

    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);
    usleep(500000);
    unsigned char buf[25];
    // position = (position < 0) ? 0 : position;
    // position = (position > 1000) ? 1000 : position;
    buf[0] = SERVO_FRAME_HEADER;
    buf[1] = SERVO_FRAME_HEADER;
    buf[2] = 23;             // length
    buf[3] = CMD_SERVO_MOVE; // cmd
    buf[4] = 6;              // number of servo to be controlled
    buf[5] = GET_LOW_BYTE(time);
    buf[6] = GET_HIGH_BYTE(time);
    for (int ind = 1; ind < 7; ind++)
    {
        buf[ind * 3 + 4] = id[ind - 1];
        buf[ind * 3 + 5] = GET_LOW_BYTE(position[ind - 1]);
        buf[ind * 3 + 6] = GET_HIGH_BYTE(position[ind - 1]);
    }

    if (fid != -1)
    {
        int count = write(fid, &buf[0], 25);
        usleep(1000);

        if (count < 0)
            printf("UART TX error\n");
    }
    close(fid);
    return 0;
}

BOOST_PYTHON_MODULE(moveServo)
{
    def("moveServo", moveServo);
}