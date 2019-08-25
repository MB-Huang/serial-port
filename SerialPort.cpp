#include "SerialPort.hpp"

SerialPort::SerialPort(void)
{
    /* memset() initialize the rx tx data memory with 0 */
    memset(this->tx.arr ,0 ,SERIALPORT_TX_LEN);
    memset(this->rx.arr ,0 ,SERIALPORT_RX_LEN);
    /* initial tx data*/
    this->tx.data.startCode = 255;
    this->tx.data.tiltPos   = 2048;   // MX origin pos
    this->tx.data.panPos    = 2048;    // MX origin pos
    /* initial rx data*/
    this->rx.data.dip = 2;
    /* initial other variable*/
    this->isOpen = false;
}

bool SerialPort::openPort(const char *deviceName, int baudrate)
{
    this->devName = deviceName;
    this->BAUDRATE = baudrate;
    
    /* open port */
    printf("Opening serial %s\r\n", deviceName);
    this->fd = open(deviceName, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1)
    {   /* open is unsucessful */
        printf("Error opening serial port %s\n", deviceName);
        return false;
    }
    else
    {
        printf("serial open succeed\n");
        this->isOpen = true;
    }
    fcntl(fd, F_SETFL, 0); /* Reads will be blocking */

    /* setup port */
    memset( &tty, 0, sizeof(tty) );
    if (tcgetattr(fd, &tty) < 0)
    {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return false;
    }

    /* setup input & output baudrate */
    /* (void) is to stop warning in cygwin */
    (void)cfsetospeed(&tty, (speed_t)this->BAUDRATE);
    (void)cfsetispeed(&tty, (speed_t)this->BAUDRATE);

    /* c_cflag成員用來控制串列埠的鮑率、同位元、停止位元等 */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~PARODD;
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */
    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    
    /* setup for non-canonical mode */
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;      /* RAW output */

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = SERIALPORT_RX_LEN; // Minimum number of characters for non-canonical read.
    tty.c_cc[VTIME] = 10;               // Timeout in deciseconds for non-canonical read.

    /* change the port parameter */
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return false;
    }
    return true;
}

bool SerialPort::readPort()
{
    usleep(100);
    tcflush(fd, TCIFLUSH);          // clean the rx buffer
    tcsetattr(fd, TCSANOW, &tty);   // change the port parameter
    int readSize = 0;
    unsigned char checkSum = 0;
    unsigned char arr[SERIALPORT_RX_LEN * 2];

    if (this->isOpen)
    {
        //arr = (unsigned char *)malloc(SERIALPORT_RX_LEN * 2);

        // check the whole data be received
		for (int i = 0; i < SERIALPORT_RX_LEN*2; i += readSize) 
		{
			readSize = read( this->fd, arr + i, SERIALPORT_RX_LEN*2 - i );
			if ( readSize <= 0) 
			{
				printf("[Error using 'SerialPort::readPort()'] can't read data!\n");
				this->isOpen = false;
				return false;
			}
            // else
            //     printf("read %d byte\n", readSize);
   		}

        /* show recv data */
        // for (int i=0; i<SERIALPORT_RX_LEN*2; i++)
        //     printf("arr[%02d] = %d\n", i, (unsigned int)arr[i]);


        /* check the receive data */
        for (int i=0; i<SERIALPORT_RX_LEN*2; i++)
        {
            /* find start code location */
            if ( arr[i]==255 && arr[i+1]==254 )
            {
                /* initial checkSum value */
                checkSum = 0;

                /* sum up the whole data array except for the start code */
                for ( int j = i+2; j < i+SERIALPORT_RX_LEN-1; j++ )
                    checkSum += arr[j];

                /* get the complment of the data sum as checkSum value*/
                checkSum = ~(checkSum);

                /* check if checkSum == rx.data.checksum */
                if ( checkSum == arr[i+SERIALPORT_RX_LEN-1] )
                {                    
                    /* check head motor value */
                    if ( (arr[i+3]+arr[i+4]*256)>4096 || (arr[i+5]+arr[i+6]*256)>4096 )
                    {
                        printf("--- BOARD ERROR ---\n");										
                        for (int j = i; j < i+SERIALPORT_RX_LEN; j++)
                            printf( "arr[%d] = %d\n", j, arr[j] );
                        printf("--- END OF BOARD ERROR ---\n");
                    }

                    /* ensure the data is great, store it! */
                    for (int j = i; j < i+SERIALPORT_RX_LEN; j++)
                        this->rx.arr[j-i] = arr[j];
                }
                else
                {
                    printf("--- CHECKSUM ERROR ---\n");
                    for (int j = i; j < i+SERIALPORT_RX_LEN; j++)
                        printf( "arr[%d] = %d\n", j, arr[j] );
                    printf("--- END OF CHECKSUM ERROR ---\n");
                } // END OF check sum
                break;	// break the check data loop
            } // END OF find start code
        } // END OF for loop

        //free(arr);
        return true;
    }
    else
    {
        printf("[Error using 'SerialPort::readPort()'] please open the port!\n");
        return false;
    }   // END OF check Port open
}

bool SerialPort::writePort()
{
    usleep(100);
    tcflush(fd, TCOFLUSH);          // clean the tx buffer
    tcsetattr(fd, TCSANOW, &tty);   // change the port parameter
    static int count = 0;

    /* setup file start code */
    this->tx.data.startCode = 255;
    this->setCheckSum(this->tx.arr, SERIALPORT_TX_LEN);

    if (this->isOpen)
    {
        int res;

        /* check if strategy need delay*/
        // if ( info->strategy.adult.globalpara.sendDelayCount != 0  )
        // {
        //     if ( count % info->strategy.adult.globalpara.sendDelayCount == 0 ){
        //         res = write(fd, tx.arr, SERIALPORT_TX_LEN);
        //         printf("delay count &5d\n", count);
        //     }
        // }
        // else
        // {
            res = write(this->fd, this->tx.arr, SERIALPORT_TX_LEN);
        // }

        count++;

        if (res)
            return true;
        else
        {
            printf("write port error\n");
            return false;
        }
    }
    else
    {
        printf("[Error using 'SerialPort::writePort()'] please open the port!\n");
        return false;
    }
}

void SerialPort::setCheckSum( unsigned char* arr ,int len )
{
    /*
     * 'checkSum' locate at the last byte of the rx/tx data.
     * This function compute the checkSum value of the data array.
     */ 

    /* inital checkSum (last byte) */
    arr[len-1] = 0;

    /* sum up the whole data array except for the start code */
    for ( int i = 1 ;i < len-1; i++ )
        arr[len-1] += arr[i];

    /* get the complment of the data sum as checkSum value*/
    arr[len-1] = ~(arr[len-1]);
}
