#include <stdio.h>
#include <stdlib.h>
#include "SerialPort.hpp"

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}

int main()
{
    SerialPort SP;
    if (!SP.openPort("/dev/ttyS0"))
    {
        printf("can't open port\n");
        return 0;
    }
    else
        printf("serial port open suceed\n");

    int keyIn = 0;
    while( keyIn != 27 )
    {
        keyIn = getch();

        /* read data */
        printf("\nstart read port\n");
        SP.readPort();
        printf("dip = %d\n", (unsigned int)SP.rx.data.dip);
        printf("motion = %d\n", (unsigned int)SP.rx.data.bodyDoingCode);

        /* write data */
        printf("\nstart write port\n");
        //SP.tx.data.bodyMotionCode = 72;
        SP.tx.data.bodyMotionCode = SP.rx.data.dip;
        for (int i = 2; i<8; i++)
            SP.tx.arr[i] = i;
        SP.writePort();
        printf("send startCode %d\n", SP.tx.data.startCode);
        printf("send bodyMotionCode = %d\n", SP.tx.arr[1]);
        printf("send checksum %d\n", SP.tx.data.checksum);
    }

    return 0;
}