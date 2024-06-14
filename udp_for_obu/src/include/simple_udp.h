#include<string>
#ifndef _SIMPLE_UDP_H_
#define _SIMPLE_UDP_H_

#define SIMPLE_UDP_RECV_BUFSIZ 1048576

typedef void (*udp_recv_handler)(const std::string&, void *arg);

int my_udp_srv(const char *addr, uint16_t port,
	udp_recv_handler handler, void *arg, size_t bufsiz, int *running);

void *my_udp_cli(const char *addr, uint16_t port);
void my_udp_cli_close(void *cli);

int my_udp_send(void *cliptr, const std::string& s);

#endif
