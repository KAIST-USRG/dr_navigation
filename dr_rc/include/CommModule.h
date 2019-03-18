
#include "DefineList.h"

using namespace ros;

int OpenSerial(const char *device_name)
{
    int fd;
    struct termios newtio;

    fd = open(device_name, O_RDWR | O_NOCTTY);

    if(fd < 0)
    {
        printf("Serial Port Open Fail.\n");
        return -1;
    }

    memset(&newtio, 0, sizeof(newtio));
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_cflag = CS8|CLOCAL|CREAD;

    switch(BAUDRATE)
    {
        case 921600 : newtio.c_cflag |= B921600;
        break;
        case 115200 : newtio.c_cflag |= B115200;
        break;
        case 57600  : newtio.c_cflag |= B57600;
        break;
    }

    newtio.c_lflag 		= 0;
    newtio.c_cc[VTIME] 	= 0;
    newtio.c_cc[VMIN] 	= (sizeof(StrRXttyO)/2 + 40);

    tcflush(fd,TCIFLUSH);
    tcsetattr(fd,TCSANOW, &newtio);

    return fd;
}



void SerialSend(int fd)
{

    //===== initial header =====//
    tx.header[0] = header1;
    tx.header[1] = header2;

    tx.IDs[0] = IDs1;
    tx.IDs[1] = IDs2;

    tx.checksum[0] = 0;
    tx.checksum[1] = 0;

    unsigned char *data = (unsigned char *)&tx;

    for(int ind=0; ind<sizeof(struct_Senddata)-2;ind++)
    {
        tx.checksum[0] += data[ind];
        tx.checksum[1] += tx.checksum[0];
    }

    write(fd,&tx,sizeof(struct_Senddata));
}



void SerialReceive(int FdPort1)
{

    // pthread create
    pthread_t p_thread;
    int thread_rx;

    thread_rx = pthread_create(&p_thread, NULL, receive_p_thread, (void *)FdPort1);

    if(thread_rx < 0)
    {
        perror("thread create error : ");
        exit(0);
    }

}



void* receive_p_thread(void *fdt)
{

    int fd = *((int*)&fdt);
    unsigned char RXRawData[sizeof(StrRXttyO)];

    printf("pthread RX process start!\n\n");
    while( ok() )
    {

        int ParsingMode   = 1;
        int ContinueWhile = 1;
        while(ContinueWhile)
        {
            switch(ParsingMode)
            {
            case 1:
                if(read((int)fd, &RXRawData[0], 1) == 1)
                {
                    if(RXRawData[0] == 0x12)
                    {
                        ParsingMode = 2;
                    }
                    else
                    {
                        ParsingMode = 1;
                    }
                }
                break;

            case 2:
                if(read((int)fd, &RXRawData[1], 1) == 1)
                {
                    if(RXRawData[1] == 0x34)
                    {
                        ParsingMode = 3;
                    }
                    else
                    {
                        ParsingMode = 1;
                    }
                }
                break;

            case 3:
                if(read((int)fd, &RXRawData[2], 1) == 1)
                {
                    if(RXRawData[2] == 0x56)
                    {
                        ParsingMode = 4;
                    }
                    else
                    {
                        ParsingMode = 1;
                    }
                }
                break;

            case 4:
                if(read((int)fd, &RXRawData[3], 1) == 1)
                {
                    if(RXRawData[3] == 0x78)
                    {
                        ParsingMode = 5;
                    }
                    else
                    {
                        ParsingMode = 1;
                    }
                }
                break;

            case 5:
                if(read((int)fd,&RXRawData[4],(sizeof(StrRXttyO)-4)/2)==(sizeof(StrRXttyO)-4)/2)
                {
                    if(read((int)fd,&RXRawData[4]+(sizeof(StrRXttyO)-4)/2,(sizeof(StrRXttyO)-4)/2)==(sizeof(StrRXttyO)-4)/2)
                    {
                        // Calculate Checksum
                        unsigned char CalChecksumA = 0;
                        unsigned char CalChecksumB = 0;

                        int Ind;

                        for(Ind = 0; Ind<(sizeof(StrRXttyO)-2); Ind++)
                        {
                            CalChecksumA += RXRawData[Ind];
                            CalChecksumB += CalChecksumA;
                        }

                        if((CalChecksumA == RXRawData[sizeof(StrRXttyO)-2])&&(CalChecksumB == RXRawData[sizeof(StrRXttyO)-1]))
                        {
                            memcpy((void *)(&StrRXttyO), (void *)(RXRawData), sizeof(StrRXttyO));
                            ContinueWhile = 0;
                        }
                        else
                        {
                            ParsingMode = 1;
                        }
                    }
                    else
                    {
                        ParsingMode = 1;
                    }
                }
                else
                {
                    ParsingMode = 1;
                }
                break;

            default:
                ParsingMode = 1;
                break;
            }
        }

        if(StrMainFuncArgs.Flag_Args[0] == 1) // Case of Activating Onboard Logging
        {
            //printf("Debug OnboadLog\n");
            OnboardLog();
        }

    }

}



void OnboardLog(void)
{
    static int    Flag_Initialized = 0;
    static FILE * FD_ONBOARD_LOG;

    if(Flag_Initialized==0) // Not Initialized
    {
        FD_ONBOARD_LOG = fopen(filename,"wb"); // File Opening
        if(FD_ONBOARD_LOG == NULL) // Open Error

        {
            printf("[ERROR] 'DS_OnboardLog()'\n");
            exit(-1); // Terminate Program
        }
        else // Opening Success
        {
            fclose(FD_ONBOARD_LOG);
            printf("[DONE] Creating onboard log file\n");
            Flag_Initialized = 1; // Initialized
        }
    }

    if(Flag_Initialized==1) // After Initializing
    {
        // Copy Data to Log File
        FD_ONBOARD_LOG = fopen(filename,"ab"); // File Opening with Update Mode
        fwrite(&StrRXttyO, sizeof(StrRXttyO), 1, FD_ONBOARD_LOG);
        fclose(FD_ONBOARD_LOG);
    }
}

