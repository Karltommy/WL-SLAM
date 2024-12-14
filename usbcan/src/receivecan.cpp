#   include <ros/ros.h>
#   include <stdio.h>
#   include "controlcan.h"
#   include <iostream>
#   include <thread>
#   include <stdlib.h>
#   include <string.h>
#   include <strings.h>
#   include <unistd.h>
#   include <sys/types.h>
#   include <sys/stat.h>
#   include <fcntl.h>
#   include <pthread.h>
#   include "usbcan/jointangle.h"

#define MAX_CHANNELS  2
#define CHECK_POINT  200
#define RX_WAIT_TIME  100
#define RX_BUFF_SIZE  10
#define msleep(ms)  usleep((ms)*1000)

unsigned gDevType = 4;  //设备型号
unsigned gDevIdx = 0;  //设备索引号
unsigned gChMask = 3;  //通道数
// unsigned gBaud = 500; //波特率定时器0
// unsigned gBaud = 0x1400; //波特率定时器0
unsigned gTxType = 2;  //报文类型
unsigned gTxSleep = 3;  //休眠时间
unsigned gTxFrames = 10;  //每次需要发送帧数
unsigned gTxCount = 1000;  //发送次数

usbcan::jointangle jangle;
ros::Publisher jointangle_pub;
float angle_hfl,angle_kfl,angle_hfr,angle_kfr,angle_hrl,angle_krl,angle_hrr,angle_krr,angle;

int caninit()
{
    // ----- init & start -------------------------------------------------
    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xffffffff;
    config.Filter = 1;  //滤波方式，1表示单滤波，0表示双滤波
    config.Mode = 0;  //模式，0表示正常模式，1表示只听模式
    config.Timing0 = 0x00;  //波特率定时器0
    config.Timing1 = 0x1C;   //波特率定时器1
    // config.Timing0 = gBaud & 0xff;  //波特率定时器0
    // config.Timing1 = gBaud >> 8;   //波特率定时器1

    int i, j;
    for (i = 0; i < MAX_CHANNELS; i++)
    {
        if ((gChMask & (1 << i)) == 0) continue;
        
        if (!VCI_InitCAN(gDevType, gDevIdx, i, &config))
        {
            printf("VCI_InitCAN(%d) failed\n", i);
            return 0;
        }
        printf("VCI_InitCAN(%d) succeeded\n", i);

        if (!VCI_StartCAN(gDevType, gDevIdx, i))
        {
            printf("VCI_StartCAN(%d) failed\n", i);
            return 0;
        }
        printf("VCI_StartCAN(%d) succeeded\n", i);
    }
    return 1;
}

int verify_frame(VCI_CAN_OBJ *can)
{
    if (can->DataLen > 8) return 0; // error: data length
    unsigned bcc = 0;
    unsigned i;
    for (i = 0; i < can->DataLen; i++)
        bcc ^= can->Data[i];
    if ((can->ID & 0xff) != bcc) return 0; // error: data checksum
    if (((can->ID >> 8) & 7) != (can->DataLen - 1)) return 0; // error: data length
    if (!can->ExternFlag) return 1; // std-frame ok
    if (((can->ID >> 11) & 0x7ff) != (can->ID & 0x7ff)) return 0; // error: frame id
    if (((can->ID >> 22) & 0x7f) != (can->ID & 0x7f)) return 0; // error: frame id
    return 1; // ext-frame ok
}

float Hextofloat(unsigned char& hex1,unsigned char& hex2,unsigned char& hex3,unsigned char& hex4)
{
    unsigned char hex[] = {hex1,hex2,hex3,hex4};
    float* p = (float*)hex;
    return *p;
}

void interpret(VCI_CAN_OBJ &canonce)
{
    
    switch(canonce.ID)
	{
		case 0x102:
		        printf("chn%d receive front left hip angle, ID is %x ,value: ",0,canonce.ID);
                angle_hfl=Hextofloat(canonce.Data[4],canonce.Data[5],canonce.Data[6],canonce.Data[7]);
                // angle=Hextofloat(canonce.Data[0],canonce.Data[1],canonce.Data[2],canonce.Data[3]);
                // printf("%f,%f\n", angle,angle_hfl);
                jangle.phi1_lf=180-angle_hfl;
                // printf("%f\n", angle_hfl);
                printf("%f,%f\n", jangle.phi1_lf,angle_hfl);
		break;
		case 0x112:
				printf("chn%d receive front left knee angle, ID is %x ,value: ",0,canonce.ID);
                angle_kfl=Hextofloat(canonce.Data[4],canonce.Data[5],canonce.Data[6],canonce.Data[7]);
                // angle=Hextofloat(canonce.Data[0],canonce.Data[1],canonce.Data[2],canonce.Data[3]);
                // printf("%f,%f\n", angle,angle_kfl);
                jangle.phi2_lf=angle_kfl+180-angle_hfl;
                // printf("%f\n", angle_kfl);
                printf("%f,%f\n", jangle.phi2_lf,angle_kfl);
		break;
		case 0x132:
				printf("chn%d receive front right hip angle, ID is %x ,value: ",0,canonce.ID);
                angle_hfr=Hextofloat(canonce.Data[4],canonce.Data[5],canonce.Data[6],canonce.Data[7]);
                // angle=Hextofloat(canonce.Data[0],canonce.Data[1],canonce.Data[2],canonce.Data[3]);
                // printf("%f,%f\n", angle,angle_hfr);
                jangle.phi1_rf=angle_hfr;
                // printf("%f\n", angle_hfr);
                printf("%f,%f\n", jangle.phi1_rf,angle_hfr);
		break;
		case 0x142:
				printf("chn%d receive front right knee angle, ID is %x ,value: ",0,canonce.ID);
                angle_kfr=Hextofloat(canonce.Data[4],canonce.Data[5],canonce.Data[6],canonce.Data[7]);
                // angle=Hextofloat(canonce.Data[0],canonce.Data[1],canonce.Data[2],canonce.Data[3]);
                // printf("%f,%f\n", angle,angle_kfr);
                jangle.phi2_rf=angle_hfr-angle_kfr;
                // printf("%f\n", angle_kfr);
                printf("%f,%f\n", jangle.phi2_rf,angle_kfr);
		break;
		case 0x162:
		        printf("chn%d receive rear left hip angle, ID is %x ,value: ",0,canonce.ID);
                angle_hrl=Hextofloat(canonce.Data[4],canonce.Data[5],canonce.Data[6],canonce.Data[7]);
                // angle=Hextofloat(canonce.Data[0],canonce.Data[1],canonce.Data[2],canonce.Data[3]);
                // printf("%f,%f\n", angle,angle_hrl);
                jangle.phi1_lr=angle_hrl;
                // printf("%f\n", angle_hrl);
                printf("%f,%f\n", jangle.phi1_lr,angle_hrl);
		break;
		case 0x172:
				printf("chn%d receive rear left knee angle, ID is %x ,value: ",0,canonce.ID);
                angle_krl=Hextofloat(canonce.Data[4],canonce.Data[5],canonce.Data[6],canonce.Data[7]);
                // angle=Hextofloat(canonce.Data[0],canonce.Data[1],canonce.Data[2],canonce.Data[3]);
                // printf("%f,%f\n", angle,angle_krl);
                jangle.phi2_lr=angle_hrl-angle_krl;
                // printf("%f\n", angle_krl);
                printf("%f,%f\n", jangle.phi2_lr,angle_krl);
		break;
		case 0x192:
				printf("chn%d receive rear right hip angle, ID is %x ,value: ",0,canonce.ID);
                angle_hrr=Hextofloat(canonce.Data[4],canonce.Data[5],canonce.Data[6],canonce.Data[7]);
                // angle=Hextofloat(canonce.Data[0],canonce.Data[1],canonce.Data[2],canonce.Data[3]);
                // printf("%f,%f\n", angle,angle_hrr);
                jangle.phi1_rr=180-angle_hrr;
                // printf("%f\n", angle_hrr);
                printf("%f,%f\n", jangle.phi1_rr,angle_hrr);
		break;
		case 0x1A2:
				printf("chn%d receive rear right knee angle, ID is %x ,value: ",0,canonce.ID);
                angle_krr=Hextofloat(canonce.Data[4],canonce.Data[5],canonce.Data[6],canonce.Data[7]);
                // angle=Hextofloat(canonce.Data[0],canonce.Data[1],canonce.Data[2],canonce.Data[3]);
                // printf("%f,%f\n", angle,angle_krr);
                jangle.phi2_rr=angle_krr+180-angle_hrr;
                // printf("%f\n", angle_krr);
                printf("%f,%f\n", jangle.phi2_rr,angle_krr);
		break;

        ++jangle.header.seq;
        
        
        
        // case 0x112:
		// 		printf("chn%d receive front left knee angle, ID is %x ,data: ",0,canonce.ID);
		// 		for(UINT j =0;j<canonce.DataLen;j++)
		// 		{
		// 		printf(" %x",canonce.Data[j]);
		// 		}
		// 		printf("\n");
		// break;
		// default:
		// printf("no messages");
        
	}
}

typedef struct {
    unsigned channel; // channel index, 0~3
    unsigned stop; // stop RX-thread
    unsigned total; // total received
    unsigned error; // error(s) detected
} RX_CTX;//接收函数传递的参数类型；

void rx_thread(RX_CTX ctx)
{
    ctx.total = 0; // reset counter
    VCI_CAN_OBJ can[RX_BUFF_SIZE]; // buffer
    int cnt; // current received
    int i;

    while (!ctx.stop && !ctx.error)
    {

        //   int num;

        //   num = VCI_GetReceiveNum(gDevType, gDevIdx,0);
        //   std::cout<< ctx.channel << "num:" << num <<std::endl;
          
          cnt = VCI_Receive(gDevType, gDevIdx, ctx.channel, can, RX_BUFF_SIZE, RX_WAIT_TIME);
        //   std::cout<<  "cnt:" <<cnt <<std::endl;
          if (!cnt)
          continue;

        //   for (i = 0; i < cnt; i++) {
            // if (verify_frame(&can[i]))
            //     {
                //  printf( "received number:%d, CANDATA:%s\n",cnt,can[i].Data);
                //  int a;
                //  a = can[i].Data[0];
                //  std::cout<< " CANdata:" << a << std::endl;
                //  std::cout<<static_cast<int>(can[i].Data[0])<<std::endl;
                for (UINT a = 0; a < cnt; a++)
			        {
				        interpret(can[a]);
		        	}
                 continue;   
            //     }
                
            // printf("CAN%d: verify_frame() failed\n", ctx.channel);
            
            // ctx.error = 1;
            // break;
        // }
        if (ctx.error) break;

        ctx.total += cnt;
    }

}

void rx_receive(RX_CTX ctx)
{
    ctx.total = 0; // reset counter
    VCI_CAN_OBJ can[RX_BUFF_SIZE]; // buffer
    int cnt; // current received
    int i;
    int flag=0;
    flag = VCI_ClearBuffer(gDevType, gDevIdx, ctx.channel);
    if(flag)
    {
        cnt = VCI_Receive(gDevType, gDevIdx, ctx.channel, can, RX_BUFF_SIZE, RX_WAIT_TIME);
        //   std::cout<<  "cnt:" <<cnt <<std::endl;

        //   for (i = 0; i < cnt; i++) 
        //   {

                for (UINT a = 0; a < cnt; a++)
			        {
				        interpret(can[a]);
		        	}
                //  continue;   
        //  }

    }   
}

void generate_frame(VCI_CAN_OBJ *can)
{
    memset(can, 0, sizeof(VCI_CAN_OBJ));
    can->SendType = gTxType;
    can->DataLen = 1 + (rand() % 8); // random data length: 1~8
    unsigned i;
    for (i = 0; i < can->DataLen; i++)
    {
        can->Data[i] = rand() & 0xff; // random data
        can->ID ^= can->Data[i]; // id: bit0~7, checksum of data0~N
    }
    can->ID |= ((unsigned)can->DataLen - 1) << 8; // id: bit8~bit10 = data_length-1
    can->ExternFlag = rand() % 2; // random frame format
    if (!can->ExternFlag)
        return;
    can->ID |= can->ID << 11; // id: bit11~bit21 == bit0~bit10
    can->ID |= can->ID << 11; // id: bit22~bit28 == bit0~bit7
}


void text()
{
    VCI_CAN_OBJ can;
    VCI_CAN_OBJ *buff = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ) * gTxFrames);
    int i,j;
    int err = 0;
    unsigned tx;
    for (tx = 0; !err && tx < gTxCount; tx++)
    {
        for (i = 0; i < MAX_CHANNELS; i++)
        {
            if ((gChMask & (1 << i)) == 0) continue;

            for (j = 0; j < gTxFrames; j++)
            	generate_frame(&buff[j]);
            if (gTxFrames != VCI_Transmit(gDevType, gDevIdx, i, &buff[0], gTxFrames))
            {
                printf("CAN%d TX failed: ID=%08x\n", i, can.ID);
                err = 1;
                break;
            }
        }
        if (gTxSleep) msleep(gTxSleep);//如果有发文休眠时间，则休眠
    }

    free(buff);


}

void canCallback()
{
   // ----- create RX-threads --------------------------------------------

    RX_CTX rx_ctx0;
    RX_CTX rx_ctx1;

    rx_ctx0.channel = 0;
    rx_ctx0.stop = 0;
    rx_ctx0.total = 0;
    rx_ctx0.error = 0;

    rx_ctx1.channel = 1;
    rx_ctx1.stop = 0;
    rx_ctx1.total = 0;
    rx_ctx1.error = 0;
   

    std::thread rx_thread0(rx_receive,rx_ctx0);
    std::thread rx_thread1(rx_receive,rx_ctx1);
    // std::thread tx_thread(text);
    
    rx_thread0.join();
    rx_thread1.join();
    // tx_thread.join();

}

int main(int argc, char **argv)
{
    
    ros::init(argc,argv,"jointangle_publisher");
    ros::NodeHandle nh;
    jointangle_pub = nh.advertise<usbcan::jointangle>("/joint_angle",10);
    ros::Rate loop_rate(50);


    if (!VCI_OpenDevice(gDevType, gDevIdx, 0)) 
    {
    printf("VCI_OpenDevice failed\n");
    return 0;
    }
    printf("VCI_OpenDevice succeeded\n");


    if(!caninit())
    {
        ROS_ERROR("CAN initialization failed");
        VCI_CloseDevice(gDevType, gDevIdx);
        return 0;
    }

   
    while(ros::ok())
    {
    //  std::cout<< "1"<<std::endl;
     canCallback();
    //  std::cout<< "2"<<std::endl;
     jangle.header.frame_id = "joint_angle";
     jangle.header.stamp.sec = ros::Time::now().toSec();
     jangle.header.stamp.nsec = ros::Time::now().toNSec();
     jointangle_pub.publish(jangle);
     ros::spinOnce();
     loop_rate.sleep();
    }


    VCI_CloseDevice(gDevType, gDevIdx);
    ROS_INFO("VCI_CloseDevice");
    return 0;
    
}