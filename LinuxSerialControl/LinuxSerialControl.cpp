#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/signal.h>
#include <termios.h>
#include <fcntl.h>

#include <SBGC.h>
#include <SBGC_Linux.h>

#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <math.h>


#include <atomic>
#include <thread>
#include <pthread.h>  
#include <mutex>

  
#define PORT  10002

#define BUF_MAX 512

//#define DEVICE_NAME "/dev/ttyACM0"
#define DEVICE_NAME "/dev/ttyUSB0"

int fd;
struct termios oldtio, newtio;



volatile float yaw = 0;
volatile float pitch = 0;

std::mutex read_coords;

void receive_messages() {
    int sockfd;
    
    struct sockaddr_in servaddr, cliaddr;
    char data[16];
      
    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
      
    memset(&servaddr, 0, sizeof(servaddr));
    // memset(&cliaddr, 0, sizeof(cliaddr));
      
    // Filling server information
    servaddr.sin_family    = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);
      
    // Bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *)&servaddr, 
            sizeof(servaddr)) < 0 )
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    socklen_t len = sizeof(cliaddr);

    uint32_t old_seq_num = 0;

    uint32_t new_seq_num = 0;

    float pitch_, yaw_;

    while(true)
    {
        recvfrom(sockfd, (char *)data, 16, 
                MSG_WAITALL, ( struct sockaddr *) &cliaddr,
                &len);

        memcpy(&new_seq_num, data, 4);

        if (new_seq_num > std::numeric_limits<uint32_t>::max() - 1000)
            break;

        if (new_seq_num < old_seq_num)
            continue;

        old_seq_num = new_seq_num;

        memcpy(&yaw_, data + 4, 4);
        memcpy(&pitch_, data + 8, 4);



        pitch_ = pitch_ / M_PI * 180;
        yaw_ = yaw_ / M_PI * 180;

        if (pitch_ > 45)
            pitch_ = 45;

        if (pitch_ < -45)
            pitch_ = -45;


        if (yaw_ > 110)
            yaw_ = 110;

        if (yaw_ < -110)
            yaw_ = -110;


        read_coords.lock();

        pitch = pitch_;
        yaw = yaw_;

        read_coords.unlock();
    }
}


void init_serial(int &fd, struct termios &oldtio, struct termios &newtio)
{
    printf("trying to open " DEVICE_NAME "...\n");
    fd = open( DEVICE_NAME, O_RDWR | O_NOCTTY );
    if (fd<0) {
        fprintf(stderr, "failed to open " DEVICE_NAME "\n");
        exit(-1);
    }

    if (tcgetattr(fd,&oldtio)) {
        fprintf(stderr, "Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    printf("open " DEVICE_NAME " by FD(%d)\n", fd);
    memset(&newtio, 0, sizeof(newtio) );

    /*
     from SimpleBGC_2_5_Serial_Protocol_Specification.pdf

     32bit boards with firmware version 2.40, works only with parity=EVEN COM-port setting. Starting from 2.41,
both EVEN and NONE parity are supported (NONE is default, and EVEN is detected automatically). So
beside baud rates, host should vary parity setting when connecting to boards ver.>3.0
      
        reference: https://www.cmrr.umn.edu/~strupp/serial.html
    */
    newtio.c_cflag = B115200; // BAUD RATE
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;

    newtio.c_iflag = IGNPAR;
 //   newtio.c_iflag = ICRNL;
    newtio.c_oflag = 0;
    newtio.c_lflag &= ~ICANON; // MUST
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    if (tcflush(fd, TCIFLUSH)) {
        fprintf(stderr, "Error %i from tcflush: %s\n", errno, strerror(errno));
    }
    if (tcsetattr(fd, TCSANOW, &newtio)) {
        fprintf(stderr, "Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    sleep(3);
}

void deinit_serial(int &fd, struct termios &oldtio)
{
    if (tcsetattr(fd,TCSANOW,&oldtio)) {
        fprintf(stderr, "Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
    close( fd);
}

//
// reference: https://stackoverflow.com/questions/6990093/how-to-convert-signal-name-string-to-signal-code
//
void signal_handler(int signalno)
{
    fprintf(stderr, "[SIG] %s(%d)\n", strsignal(signalno), signalno);

    deinit_serial(fd, oldtio);
    exit(1);
}

void init_sig(void)
{
    struct sigaction saio;
    saio.sa_handler = signal_handler;
    sigemptyset(&saio.sa_mask);
    saio.sa_flags = 0;
    saio.sa_restorer = NULL;
    sigaction(SIGINT, &saio,NULL);
}

//
// reference:
//   * https://stackoverflow.com/questions/63785360/c-alignas1-does-not-affect-the-size-of-a-struct
//   * https://dojang.io/mod/page/view.php?id=432
//
#pragma pack(push, 1)
struct Version_Info {
    unsigned char board_ver;
    unsigned short firmware_ver;
    unsigned char debug_mode;
    unsigned short board_features;
    unsigned char connection_flags;
    unsigned int  frw_extra_id;
    unsigned char reserved0;
    unsigned char reserved1;
    unsigned char reserved2;
    unsigned char reserved3;
    unsigned char reserved4;
    unsigned char reserved5;
    unsigned char reserved6;
};
#pragma pack(pop)

struct Angle_Info {
    short imu_angle;
    short rc_target_angle;
    short rc_speed;
};

int main( void)
{
    init_serial(fd, oldtio, newtio);

    init_sig();

	SBGC_Demo_setup(fd);
#if 1
    printf("version info:\n");
    {
        SerialCommand cmd;
        cmd.init(SBGC_CMD_BOARD_INFO);
        sbgc_parser.send_cmd(cmd, 0);

        sleep(1); // MUST

        int size=0;
        char buf[BUF_MAX];
        printf("[header]---------------------\n");
        size = read(fd,buf,4);
        if (size != 4) {
            fprintf(stderr, "failed to read header \n");
            exit(-1);
        }

        for (int i = 0;i < 4; i++) {
            printf("0x%x\t(%u)\n", (unsigned char)buf[i], (unsigned char)buf[i]);
        }

        printf("[body]---------------------\n");
#if 1
        struct Version_Info version = {0, };
        size = read(fd, (unsigned char*)&version, sizeof(Version_Info));

        if (size != sizeof(Version_Info))
        {
            fprintf(stderr, "size = %d, sizeof(Version_Info) = %lu\n", size, sizeof(Version_Info));
            fprintf(stderr, "failed to read body \n");
            exit(-1);
        }

        // Only for check
//        unsigned char *cur = (unsigned char*)(&version);
//        for (int i = 0; i < sizeof(Version_Info); i++)
//        {
//            printf("[%d] %u\n", i, cur[i]);
//        }

        printf("board_ver: %u(0x%x)\n", version.board_ver, version.board_ver);
        printf("firmware_ver: %u(0x%x)\n", version.firmware_ver, version.firmware_ver);

        // checksum
        size = read(fd,buf,BUF_MAX);
        buf[size] = '\0';

        for (int i = 0;i < size; i++) {
            printf("0x%x\t(%d)\n", buf[i], buf[i]);
        }

#else

        size = read(fd,buf,BUF_MAX);
        buf[size] = '\0';

        for (int i = 0;i < size; i++) {
            printf("0x%x\t(%d)\n", buf[i], buf[i]);
        }
#endif
    }
#endif

    std::thread udp_thread (receive_messages);


    printf("angle info:\n");

    //for (int k = 0; k < 100; k++)
    {
        SerialCommand cmd;
        cmd.init(SBGC_CMD_GET_ANGLES);
        sbgc_parser.send_cmd(cmd, 0);

        sleep(1); // MUST

        int size=0;
        char buf[BUF_MAX];
        printf("[header]---------------------\n");
        size = read(fd,buf,4);
        if (size != 4) {
            fprintf(stderr, "failed to read header \n");
        }

        for (int i = 0;i < 4; i++) {
            printf("0x%x\t(%u)\n", (unsigned char)buf[i], (unsigned char)buf[i]);
        }

        printf("[body]---------------------\n");

        struct Angle_Info angles[3] = {{0, }, {0, }, {0, }};
        size = read(fd, (unsigned char*)angles, sizeof(Angle_Info)*3);
        buf[size] = '\0';

        for (int i = 0;i < 3; i++) {
            printf("[%03d] imu_angle:       %03.03f\t(0x%x)\n", i, SBGC_ANGLE_TO_DEGREE(angles[i].imu_angle), angles[i].imu_angle);
            printf("[%03d] rc_target_angle: %03.03f\t(0x%x)\n", i, SBGC_ANGLE_TO_DEGREE(angles[i].rc_target_angle), angles[i].rc_target_angle);
            printf("[%03d] rc_speed:        %d\t(0x%x)\n", i, angles[i].rc_speed, angles[i].rc_speed);
        }

        // checksum
        size = read(fd,buf,BUF_MAX);
        buf[size] = '\0';

        for (int i = 0;i < size; i++) {
            printf("0x%x\t(%d)\n", buf[i], buf[i]);
        }
    }

    printf("moving...\n");

    printf("Set RPY=(0, 0, 0)...\n");



      

#if 1
    static SBGC_cmd_control_ext_t cmd_control = { 
        { SBGC_CONTROL_MODE_NO, SBGC_CONTROL_MODE_ANGLE, SBGC_CONTROL_MODE_ANGLE },  // control mode for ROLL, PITCH, YAW
        { { 0, 0 }, { 0, 0 }, { 0, 0 } }  // angle and speed
    }; 

    SBGC_cmd_control_ext_send(cmd_control, sbgc_parser);


    //loop where coordinates are read and put into the command
    int i = 0;

    while (i < 1000)
    {
        read_coords.lock();

        cmd_control.data[PITCH].angle = SBGC_DEGREE_TO_ANGLE(-pitch);
        cmd_control.data[YAW].angle = SBGC_DEGREE_TO_ANGLE(-yaw);

        read_coords.unlock();


        SBGC_cmd_control_ext_send(cmd_control, sbgc_parser);

        usleep(30 * 1000);
        i++;
    }



    



 //    SBGC_cmd_control_ext_send(cmd_control, sbgc_parser);
	// sleep(4);

 //    printf("Set RPY=(0, 10, 0)...\n");
 //    cmd_control.data[PITCH].angle = SBGC_DEGREE_TO_ANGLE(20); 
 //    SBGC_cmd_control_ext_send(cmd_control, sbgc_parser);
	// sleep(4);


 //    printf("Set RPY=(10, 0, 0)...\n");
 //    cmd_control.data[ROLL].angle = SBGC_DEGREE_TO_ANGLE(10); 
 //    SBGC_cmd_control_ext_send(cmd_control, sbgc_parser);
	// sleep(4);

 //    printf("Set RPY=(0, 0, 0)...\n");
 //    cmd_control.data[PITCH].angle = SBGC_DEGREE_TO_ANGLE(0); 
 //    SBGC_cmd_control_ext_send(cmd_control, sbgc_parser);
	// sleep(4);

 //    printf("Set RPY=(0, 0, 10)...\n");
 //    cmd_control.data[YAW].angle = SBGC_DEGREE_TO_ANGLE(20);
 //    SBGC_cmd_control_ext_send(cmd_control, sbgc_parser);
	// sleep(4);


 //    printf("Set RPY=(0, 0, -10)...\n");
 //    cmd_control.data[YAW].angle = SBGC_DEGREE_TO_ANGLE(-20);
 //    SBGC_cmd_control_ext_send(cmd_control, sbgc_parser);
	// sleep(4);

 //    printf("Set RPY=(0, 0, 0)...\n");
 //    cmd_control.data[YAW].angle = SBGC_DEGREE_TO_ANGLE(0);
 //    SBGC_cmd_control_ext_send(cmd_control, sbgc_parser);
	// sleep(4);

    printf("Turn off Control...\n");
    cmd_control = { 
        { SBGC_CONTROL_MODE_NO, SBGC_CONTROL_MODE_NO, SBGC_CONTROL_MODE_NO},  // control mode for ROLL, PITCH, YAW
        { { 0, 0 }, { 0, 0 }, { 0, 0 } }  // angle and speed
    }; 
    SBGC_cmd_control_ext_send(cmd_control, sbgc_parser);
	sleep(4);



    //sending a packet that will cause the receiving thread to terminate

    int sigsockfd;
    if ( (sigsockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in sigaddr;
    memset(&sigaddr, 0, sizeof(sigaddr));

    sigaddr.sin_family = AF_INET; // IPv4
    sigaddr.sin_port = htons(PORT);
    sigaddr.sin_addr.s_addr = inet_addr("127.0.0.1");

    char zeros[12];
    memset(zeros, 0, 12);

    uint32_t stop_seq_num = std::numeric_limits<uint32_t>::max();
    memcpy(zeros, &stop_seq_num, 4);


    sendto(sigsockfd, (const char *)zeros, 12,
                   MSG_DONTWAIT,
                   (const struct sockaddr *) &sigaddr,
                   sizeof(sigaddr));


#else
	SBGC_cmd_control_t c = { 0, 0, 0, 0, 0, 0, 0 };
	c.mode = SBGC_CONTROL_MODE_ANGLE;
	c.anglePITCH = SBGC_DEGREE_TO_ANGLE(20);
    //c.angleYAW = SBGC_DEGREE_TO_ANGLE(20);
	if (SBGC_cmd_control_send(c, sbgc_parser))
    {
        fprintf(stderr, "Something wrong\n");
        exit(-1);
    }
	sleep(4);

	c.mode = SBGC_CONTROL_MODE_ANGLE;
	c.anglePITCH = SBGC_DEGREE_TO_ANGLE(-20);
    //c.angleYAW = SBGC_DEGREE_TO_ANGLE(10);
	if (SBGC_cmd_control_send(c, sbgc_parser))
    {
        fprintf(stderr, "Something wrong\n");
        exit(-1);
    }
	sleep(4);
	c.anglePITCH = SBGC_DEGREE_TO_ANGLE(-10);
	c.angleYAW = SBGC_DEGREE_TO_ANGLE(-10);
	SBGC_cmd_control_send(c, sbgc_parser);
	sleep(4);

	c.anglePITCH = 0;
	c.angleYAW = 0;
	SBGC_cmd_control_send(c, sbgc_parser);
	sleep(4);

#endif

    deinit_serial(fd, oldtio);

    udp_thread.join();
    return 0;
}





