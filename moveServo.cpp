#include <stdio.h>
#include <unistd.h>    // UART需要
#include <sys/fcntl.h> // UART需要
#include <termios.h>   // UART需要
#include <string>
#include <Python.h>

using namespace std;

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

int moveServo(uint16_t position, int16_t time)
{
    int fid = -1;
    uint8_t id[] = {1, 2, 3, 4, 5, 6};
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
    unsigned char buf[10];
    position = (position < 0) ? 0 : position;
    position = (position > 1000) ? 1000 : position;
    buf[0] = SERVO_FRAME_HEADER;
    buf[1] = SERVO_FRAME_HEADER;
    buf[2] = 8;              // length
    buf[3] = CMD_SERVO_MOVE; // cmd
    buf[4] = 1;              // number of servo to be controlled
    buf[5] = GET_LOW_BYTE(time);
    buf[6] = GET_HIGH_BYTE(time);
    buf[7] = id;
    buf[8] = GET_LOW_BYTE(position);
    buf[9] = GET_HIGH_BYTE(position);

    int count;
    if (fid != -1)
    {
        count = write(fid, &buf[0], 10);
        usleep(1000);

        if (count < 0)
            printf("UART TX error\n");
    }
    close(fid);
    return count;
}

PyObject *WrappmoveServo(PyObject *self, PyObject *args)
{
    uint8_t id;
    uint16_t position;
    int16_t time;
    if (!PyArg_ParseTuple(args, "iii", &id, &position, &time))
    {
        return NULL;
    }
    return Py_BuildValue("i", moveServo(id, position, time));
}

static PyMethodDef moveServo_methods[] = {
    {"moveServo", WrappmoveServo, METH_VARARGS, "something"},
    {NULL}};

extern "C"

    void
    initmoveServo()
{
    Py_InitModule("moveServo", moveServo_methods);
}