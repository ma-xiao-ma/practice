#include <pthread.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>

/***********************************************/
#define TTC_ROUTE_ADDR  		0       /*星内节点*/
#define OBC_ROUTE_ADDR  		1
#define ADCS_ROUTE_ADDR 		2
#define EPS_ROUTE_ADDR  		3
#define CAM_ROUTE_ADDR  		5
/***********************************************/
#define GND_ROUTE_ADDR  		66      /*地面节点*/
/***********************************************/

/*下行消息类型*/
#define OBC_TELEMETRY         	0xE1
#define ADCS_TELEMETRY         	0xE2
#define CAM_IMAGE_INFO         	0xE3
#define CAM_IMAGE              	0xE4
#define FILE_INFO			    0xE5
#define FILE_DATA			    0xE6

/*各种帧长度*/
#define IMAGE_PACK_MAX_SIZE    	220

#define ROUTE_HEAD_SIZE			3

#define BEACON_SIZE			    21
#define OBC_TELEMETRY_SIZE		181
#define	ADCS_TELEMETRY_SIZE		223

typedef struct __attribute__((__packed__))
{
	uint8_t ax25[16];
	uint8_t dst;
	uint8_t src;
	uint8_t typ;
} protocol_header_t;

typedef struct __attribute__((__packed__))
{
	uint16_t PacketID;
    uint8_t PacketSize;
} image_header_t;

typedef struct __attribute__((__packed__))
{
    uint32_t ImageID;       //图像ID
    uint32_t ImageSize;     //图像大小
    uint16_t TotalPacket;   //图像包总数
    uint8_t PacketSize;     //数据包大小
    uint8_t LastPacketSize; //尾包大小
    uint32_t ImageTime;     //拍照时间
    float ImageLocation[3]; //拍照位置
} image_info_t;

typedef struct __attribute__((__packed__))
{
	uint32_t file_size;		//文件总字节数
    uint16_t total_packet;   //文件包总数
    uint8_t packet_size;     //数据包大小
    uint32_t time;
	uint8_t filename[20];
} file_info_down_t;

extern int errno;

uint8_t image_data[IMAGE_PACK_MAX_SIZE];
uint8_t file_data[IMAGE_PACK_MAX_SIZE];

void *thread_function(void *arg);


int main(void)
{
    int res;
    pthread_t a_thread;

    res = pthread_create(&a_thread, NULL, thread_function, NULL);
    if (res != 0)
    {
        printf("ERROR: Thread creation failed!\n");
        exit(EXIT_FAILURE);
    }
  

    while(1)
    {
        char pc_get[20];

        printf("Enter 'q' to qiut, and kill the thread!\n");

        memset(pc_get, 0, 20);
        scanf("%s", pc_get);

        if (strcmp("q", pc_get) == 0)
        {
            pthread_cancel (a_thread);
            break;
        }
    }

    return 0;
}

void *thread_function(void *arg)
{
    int src_file;
    int received_file = 0, received_image = 0;

    char Image_file_path[50];
    char file_path[50];

    image_info_t info = {0};
    file_info_down_t file_info = {0};
    uint16_t next_packet_id = 0 , next_file_packet_id = 0;

    uint32_t n_packet_loss = 0, obc_tm_packet_num = 0, adcs_tm_packet_num = 0, n_file_packet_loss = 0;
    uint32_t image_info_packet_num = 0, image_data_packet_num = 0, beacon_num = 0;
    uint32_t sat_reponse_num = 0, file_info_packet_num = 0, file_data_packet_num = 0, error_packet_num = 0;

    ssize_t  rd_return;

    __off64_t current_src_file_pointer = 0;

    src_file = open( "./NJUST/1_bpsk_test", O_RDONLY );

    if( src_file == -1 )
    {
        printf("Open 1_bpsk_test file fail!\n");
        return 0;
    }

    while(1)
    {
        protocol_header_t header = {0};
        image_header_t data_header = {0};
        image_header_t file_data_header = {0};

        /* 将源文件指针偏移至 保存的源文件指针处 */
        lseek( src_file, current_src_file_pointer, SEEK_SET );

        while( read(src_file, &header, sizeof(protocol_header_t)) == sizeof(protocol_header_t) )
        {
            /* 默认信标的前三个字节为0x48 0x69 0x20 */
            if( header.dst == 0x48 && header.src == 0x69 && header.typ == 0x20 )
            {
                beacon_num ++;
                lseek( src_file, BEACON_SIZE - ROUTE_HEAD_SIZE, SEEK_CUR );
                current_src_file_pointer = lseek( src_file, 0, SEEK_CUR );
            }
            else if( header.dst == GND_ROUTE_ADDR && header.typ < OBC_TELEMETRY )
            {
                /* 上行遥控指令回复内容 */
                sat_reponse_num ++;
                lseek( src_file, 1, SEEK_CUR );
                current_src_file_pointer = lseek( src_file, 0, SEEK_CUR );
            }
            else if( header.dst == GND_ROUTE_ADDR && header.typ == 0xE7 )
            {
                
                lseek( src_file, 32, SEEK_CUR );
                current_src_file_pointer = lseek( src_file, 0, SEEK_CUR );
            }	
            else if( header.dst == GND_ROUTE_ADDR && header.typ == OBC_TELEMETRY )
            {	/* 星务遥测主帧 */
                obc_tm_packet_num ++;
                lseek( src_file, OBC_TELEMETRY_SIZE, SEEK_CUR );
                current_src_file_pointer = lseek( src_file, 0, SEEK_CUR );
            }
            else if( header.dst == GND_ROUTE_ADDR && header.typ == ADCS_TELEMETRY )
            {	/* 姿控遥测辅帧 */
                adcs_tm_packet_num ++;
                lseek( src_file, ADCS_TELEMETRY_SIZE, SEEK_CUR );
                current_src_file_pointer = lseek( src_file, 0, SEEK_CUR );
            }
            else if( header.dst == GND_ROUTE_ADDR && header.typ == FILE_INFO )
            {	/* 文件信息包 */
                
                if( read( src_file, &file_info, sizeof(file_info_down_t) ) != 
                sizeof(file_info_down_t) )
                {  
                    printf("ERROR: File info read fial!!\n");
                    goto error;	
                }

                printf("\r\n");
                printf( "file name: %s \n\n", file_info.filename );
                printf( "file size: %u.\n", file_info.file_size );
                printf( "total packet: %u.\n", file_info.total_packet );
                printf( "packet size: %u.\n", file_info.packet_size );
                printf( "TimeStamp: %s.\n", ctime( (time_t *)&file_info.time ) );
                printf("\r\n");
                memset( file_path, 0, 50 );

                sprintf( file_path, "./Documents/%s", file_info.filename );

                received_file = open( file_path, O_CREAT | O_RDWR, S_IRWXU );

                if( received_file == -1 )
                {
                    printf("ERROR: Receiving file create fail!\n");
                    goto error;	
                }

                fsync(received_file);

                next_file_packet_id = 0;

                current_src_file_pointer = lseek( src_file, 0, SEEK_CUR );
                file_info_packet_num ++;
            }
            else if( header.dst == GND_ROUTE_ADDR && header.typ == FILE_DATA )
            {
                /* 如果没有收到文件信息包， 没有创建文件 */
                if( received_file == 0 || received_file == -1 )
                {
                    printf("WARNING: File info has not received!!!\n");

                    received_file = open( "./Documents/UndefinedFile.dat", O_CREAT | O_RDWR, S_IRWXU );
                    if( received_file == -1 )
                    {
                        printf("ERROR: Receiving file create fail!\n");
                        goto error;	
                    }
                }

                /* 读取文件数据包包头，得到文件 包ID 和 包数据长度 */
                if( read( src_file, &file_data_header, sizeof(image_header_t) ) != 
                sizeof(image_header_t) )
                {
                    printf("ERROR: Image data header read fial!!\n");
                    goto error;	
                }

                /* 如果文件报数据长度不符合协议规定 */
                if( file_data_header.PacketSize > IMAGE_PACK_MAX_SIZE )
                {
                    printf("ERROR: Image packet size error!!\n");
                    goto error;	
                }

                /* 读取文件数据包的 数据部分 */
                if( rd_return = read( src_file, file_data, file_data_header.PacketSize ) != 
                file_data_header.PacketSize ) 
                {
                    printf("ERROR: Image data packet id %u read %u byte!!\n", file_data_header.PacketID, (uint32_t)rd_return);
                    goto error;	
                }

                /* 计算丢包数 */
                if( next_file_packet_id < file_data_header.PacketID )
                {
                    printf("LOSS: file packet%u -- file packet%u.\n", next_file_packet_id, file_data_header.PacketID - 1);
                    n_file_packet_loss += file_data_header.PacketID - next_file_packet_id;
                    next_file_packet_id = file_data_header.PacketID;
                }

                if( next_file_packet_id == file_data_header.PacketID )
                    next_file_packet_id++; // next_file_packet_id must be the next packet ID

                /* 根据 文件数据包 包头设置目的文件指针偏移 */
                if( lseek( received_file, file_data_header.PacketID * IMAGE_PACK_MAX_SIZE, SEEK_SET ) != 
                file_data_header.PacketID * IMAGE_PACK_MAX_SIZE )
                {
                    printf("ERROR: Image File lseek fial!\n");
                    goto error;	
                }

                /* 将文件数据包写入指定位置 */
                if( write( received_file, file_data, file_data_header.PacketSize ) != file_data_header.PacketSize )
                {
                    printf("ERROR: Image File Write fail!\n");
                    goto error;	
                }

                fsync(received_file);

                /* 成功读出数据后，保存当前源文件指针值 */
                current_src_file_pointer = lseek( src_file, 0, SEEK_CUR );
                file_data_packet_num ++;

                // if( file_data_packet_num == 10 )
                //     system(strcat("eog ", file_path));
            }
            else if( header.dst == GND_ROUTE_ADDR && header.typ == CAM_IMAGE_INFO )
            {	/* 图像信息包 */

                if( read( src_file, &info, sizeof(image_info_t) ) != 
                sizeof(image_info_t) )
                {  
                    printf("ERROR: Image info read fial!!\n");
                    goto error;	
                }

                printf("\r\n");
                printf( "ImageID: %u.\n", info.ImageID );
                printf( "ImageSize: %u.\n", info.ImageSize );
                printf( "TotalPacket: %u.\n", info.TotalPacket );
                printf( "PacketSize: %u.\n", info.PacketSize );
                printf( "LastPacketSize: %u.\n", info.LastPacketSize );
                printf( "TimeStamp: %s.\n", ctime( (time_t *)&info.ImageTime ) );
                printf("\r\n");
                
                memset(Image_file_path, 0, 50);

                sprintf(Image_file_path, "./Documents/Sat_Image-%u.dat", info.ImageID);

                received_image = open( Image_file_path, O_CREAT | O_RDWR, S_IRWXU );

                if( received_image == -1 )
                {
                    printf("ERROR: Receiving image create fail!\n");
                    goto error;	
                }

                fsync(received_image);

                next_packet_id = 0;

                current_src_file_pointer = lseek( src_file, 0, SEEK_CUR );
                image_info_packet_num ++;

            }
            else if( header.dst == GND_ROUTE_ADDR && header.typ == CAM_IMAGE )
            {	/* 图像数据包 */

                if( received_image == 0 || received_image == -1 )
                {
                    printf("WARNING: Image info has not received!!!\n");

                    received_image = open( "./Documents/UndefinedImage.dat", O_CREAT | O_RDWR, S_IRWXU );
                    if( received_image == -1 )
                    {
                        printf("ERROR: Receiving image create fail!\n");
                        goto error;	
                    }
                }

                if( read( src_file, &data_header, sizeof(image_header_t) ) != 
                sizeof(image_header_t) )
                {
                    printf("ERROR: Image data header read fial!!\n");
                    goto error;	
                }

                if( data_header.PacketSize > IMAGE_PACK_MAX_SIZE )
                {
                    printf("ERROR: Image packet size error!!\n");
                    goto error;	
                }

                if( rd_return = read( src_file, image_data, data_header.PacketSize ) != 
                data_header.PacketSize ) 
                {
                    printf("ERROR: Image data packet id %u read %u byte!!\n", data_header.PacketID, (uint32_t)rd_return);
                    goto error;	
                }

                if( next_packet_id < data_header.PacketID )
                {
                    printf("LOSS: packet%u -- packet%u.\n", next_packet_id, data_header.PacketID - 1);
                    n_packet_loss += data_header.PacketID - next_packet_id;
                    next_packet_id = data_header.PacketID;
                }

                if( next_packet_id == data_header.PacketID )
                    next_packet_id++; // next_packet_id must be the next packet ID

                if( lseek( received_image, data_header.PacketID * IMAGE_PACK_MAX_SIZE, SEEK_SET ) != 
                data_header.PacketID * IMAGE_PACK_MAX_SIZE )
                {
                    printf("ERROR: Image File lseek fial!\n");
                    goto error;	
                }

                if( write( received_image, image_data, data_header.PacketSize ) != data_header.PacketSize )
                {
                    printf("ERROR: Image File Write fail!\n");
                    goto error;	
                }
                fsync(received_image);

                /* 成功读出数据后，保存当前源文件指针值 */
                current_src_file_pointer = lseek( src_file, 0, SEEK_CUR );
                image_data_packet_num ++;

                // if( image_data_packet_num == 10 )
                //     system(strcat("eog ", Image_file_path));
            }
            else
            {   
                /* 如果收到一个错误帧，则寻找下一个头 */
                printf( "ERROR: Invalid packet detacted!!\n" );

                uint8_t is_header_byte = 0, byte_read = 0;
                
                lseek( src_file, -sizeof(protocol_header_t), SEEK_CUR );

                while( read(src_file, &byte_read, sizeof(uint8_t) ) == sizeof(uint8_t) )
                {
                    printf("0x%02X ", byte_read);

                    if( is_header_byte == 0 && byte_read == 0x84 )
                         is_header_byte++;
                    else if( is_header_byte == 1 && byte_read == 0x92 )
                         is_header_byte++;
                    else if( is_header_byte == 2 && byte_read == 0x68 )
                         is_header_byte++;
                    else if( is_header_byte == 3 && byte_read == 0xA6 )
                         is_header_byte++;
                    else if( is_header_byte == 4 && byte_read == 0xA8 )
                         is_header_byte++;
                    else if( is_header_byte == 5 && byte_read == 0x40 )
                        is_header_byte++;
                    else if( is_header_byte == 6 && byte_read == 0xE2 )
                        is_header_byte++;
                    else if( is_header_byte == 7 && byte_read == 0x84 )
                        is_header_byte++;
                    else if( is_header_byte == 8 && byte_read == 0x92 )
                        is_header_byte++;
                    else if( is_header_byte == 9 && byte_read == 0x68 )
                        is_header_byte++;
                    else if( is_header_byte == 10 && byte_read == 0xA6 )
                        is_header_byte++;
                    else if( is_header_byte == 11 && byte_read == 0xA8 )
                        is_header_byte++;
                    else if( is_header_byte == 12 && byte_read == 0x40 )
                        is_header_byte++;
                    else if( is_header_byte == 13 && byte_read == 0x61 )
                        is_header_byte++;
                    else if( is_header_byte == 14 && byte_read == 0x03 )
                        is_header_byte++;
                    else if( is_header_byte == 15 && byte_read == 0xF0 )
                    {
                        error_packet_num++;

                        /* 将文件指针放置至下一个包头处 */
                        current_src_file_pointer = lseek( src_file, -16, SEEK_CUR );
                        is_header_byte = 0;
                        break;
                    }
                    else
                        is_header_byte = 0;
                }

                printf("\n\n");

                /* 将源文件指针偏移至 保存的源文件指针处 */
                lseek( src_file, current_src_file_pointer, SEEK_SET );

                continue;
            }
        }


error:
        printf("\r\n");
        printf("COUNT: beacon packet %u.\n", beacon_num);
        printf("COUNT: sat rsp packet %u.\n", sat_reponse_num);
        printf("COUNT: obc tm packet %u.\n", obc_tm_packet_num);
        printf("COUNT: adcs tm packet %u.\n", adcs_tm_packet_num);

        printf("COUNT: img info packet %u.\n", image_info_packet_num);
        printf("COUNT: img data packet %u.\n", image_data_packet_num);
        printf("COUNT: img data packet loss %u.\n", n_packet_loss);

        printf("COUNT: file info packet %u.\n", file_info_packet_num);
        printf("COUNT: file data packet %u.\n", file_data_packet_num);
        printf("COUNT: file packet loss %u.\n", n_file_packet_loss);

        printf("COUNT: error packet %u.\n", error_packet_num);
        printf("\r\n");

        printf("Enter 'q' to qiut, and kill the thread!\n");

        /* 如果没有收到文件信息包， 没有创建文件 */
        if( received_file != 0 && received_file != -1 )
            fsync(received_file);

        if( received_image != 0 && received_image != -1 )
            fsync(received_image);

        sleep(3);
    }
		
}
