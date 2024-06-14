#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include <pthread.h>
#include <stdlib.h>
#include <stdint.h>
#include "simple_udp.h"
#include <cmath>
#include <cstring>
// ����ʱ��Щ�����ú�д������������޸�Ϊ�����ò���
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <string>
#include <iostream>
#define MY_RATE 10 // 10 Hz

// Ŀ�� UDP ���������ã���������
#define MY_DEST_ADDR "192.168.1.105" // ���ó�Ŀ�� IP
#define MY_DEST_PORT 4096        // ����Э���޸�

// ���� UDP ���������ã���������
#define MY_SERV_ADDR "192.168.1.107" // �����޸ģ�"0.0.0.0" ��ʾ������������
#define MY_SERV_PORT 4096      // ����Э���޸�
void *cli;

//����һ���µ����ݽṹ�����ڱ�ʾ���� x��y �� yaw ����Ϣ
struct UplinkData {
    int32_t xCoordinate;
    int32_t yCoordinate;
    int32_t yawAngle;
    int8_t checksum;
};

// �����л�����
UplinkData deserializeUplinkData(const std::string& data) {
    UplinkData result;
    memcpy(&result, data.data(), sizeof(UplinkData));
    return result;
}

void my_recv_handler(const std::string& data, void *arg_ptr) {
    UplinkData uplinkData = deserializeUplinkData(data);
    //std::cout << data << std::endl;
    std::cout << "Received x: " << uplinkData.xCoordinate
              << ", y: " << uplinkData.yCoordinate
              << ", yaw: " << uplinkData.yawAngle << std::endl;
    fflush(stdout);
    //��������
}

// UDP ����������һ���߳�
void *my_udp_srv_thread(void *arg_ptr)
{
	struct my_arg_struct {
		int* running_flag;
		//.pub = &pub
	};
    struct my_arg_struct *arg = (struct my_arg_struct*)arg_ptr;
	my_udp_srv(MY_SERV_ADDR, MY_SERV_PORT,
		my_recv_handler, arg_ptr, SIMPLE_UDP_RECV_BUFSIZ,arg->running_flag);
	printf("simple_udp server exited");
	return NULL;
}

//�ֽ����������˺ͺ��̵�״̬������λ��ʾ�������ˣ�������λ��ʾ���̵�״̬��00��ʾΪ�ӽ����̵ƣ�01��ʾ�̵ƣ�10��ʾ��ƣ�11��ʾ�Ƶƣ��ڰ�λΪУ��λ
// BCCУ���㷨������У��λ
int8_t calculateBCC(int8_t data) {
    int8_t bcc = 0;
    for (int i = 0; i < 7; ++i) { // ֻ����ǰ7λ
        bcc ^= (data >> i) & 1;
    }
    return bcc;
}

// �������������
void sendData(bool pedestrianPresent, int8_t trafficLightStatus, void* cli) {
    int8_t data = 0x00; // ��ʼ�������ֽ�
    // ��������״̬λ������λ��
    if (pedestrianPresent) {
        data |= 0x10; // ��������ˣ����õ���λΪ1
    }
    // ���ú��̵�״̬λ��������λ��
    data |= (trafficLightStatus & 0x03) << 5; // ȷ��ֻ����λ��ʹ�ã�����������ȷ��λ��
    // ����У��λ�����ã��ڰ�λ��
    int8_t bcc = calculateBCC(data);
    data |= (bcc << 7); // ��У��λ���õ��ڰ�λ
    // ��������
    std::string message(1, data); // �������ֽ�ת��Ϊ�ַ���
    //printf("trasmit result:%d\n",my_udp_send(cli, message));
    if (my_udp_send(cli, message) < 0) {
        std::cerr << "Failed to send data" << std::endl;
    }
}


int main(int argc, char* argv[])
{
	cli = my_udp_cli(MY_DEST_ADDR, MY_DEST_PORT);
	
	int running_flag = ~0; // running_flag ��0���÷�������������
	struct my_arg_struct {
		int* running_flag;
		//.pub = &pub
	};
    my_arg_struct srv_arg;
    srv_arg.running_flag=&running_flag;
	pthread_t srv_pid;
	//printf("start");
	//pthread_create(&srv_pid, NULL, my_udp_srv_thread, &srv_arg);
	
	int thread_create_result = pthread_create(&srv_pid, NULL, my_udp_srv_thread, &srv_arg);
    if (thread_create_result != 0) {
      printf("Failed to create thread: %d\n", thread_create_result);
    } else {
      printf("Thread created successfully\n");
    }
	fflush(stdout);
	
	pthread_detach(srv_pid);
	bool pedestrianPresent = 0;
    int8_t trafficLightStatus = 0x01;
	while (1) {
        sendData(pedestrianPresent, trafficLightStatus, cli);
        //std::cerr << "send over!" << std::endl;
		sleep(1/MY_RATE);
	};


    sleep(5);
	my_udp_cli_close(cli);
	return 0;
}
