#include <string>
#include <csignal>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdint.h>
#include "simple_udp.h"

int my_udp_srv(const char *addr, uint16_t port,
	udp_recv_handler handler, void *arg, size_t bufsiz, int *running)
{
	static int run_forever = 1;
	int sfd;
	struct sockaddr_in server_sockaddr;
	struct sockaddr_in client_sockaddr;
	sfd = socket(AF_INET, SOCK_DGRAM, 0);
	socklen_t client_len = sizeof(struct sockaddr);
	server_sockaddr.sin_family = AF_INET;
	server_sockaddr.sin_port = htons(port);
	server_sockaddr.sin_addr.s_addr = inet_addr(addr);
	int state = bind(sfd, (struct sockaddr*)
		&server_sockaddr, sizeof(server_sockaddr));
	if (state) {
		return ~0;
	}
	signal(SIGPIPE, SIG_IGN);
	running = running ? running : &run_forever;
	uint8_t *buffer = (uint8_t*)alloca(bufsiz);
	while (*running) {
		size_t rdsize = recvfrom(sfd, buffer, bufsiz, 0,
			(struct sockaddr*)&client_sockaddr, &client_len);
		std::string rdstr((char*)buffer, rdsize);
		handler(rdstr, arg);
	}
	close(sfd);
	return 0;
}

struct cli_arg {
	int fd;
	struct sockaddr_in srv_addr;
};

void *my_udp_cli(const char *addr, uint16_t port)
{
	struct cli_arg *cli = (struct cli_arg*)
		malloc(sizeof(struct cli_arg));
	if (!cli)
		return NULL;
	cli->fd = socket(AF_INET, SOCK_DGRAM, 0);
	cli->srv_addr.sin_family = AF_INET;
	cli->srv_addr.sin_port = htons(port);
	cli->srv_addr.sin_addr.s_addr = inet_addr(addr);
	return cli;
}

void my_udp_cli_close(void *cli)
{
	if (!cli)
		return;
	close(((struct cli_arg*)cli)->fd);
	free(cli);
}


int my_udp_send(void *cliptr, const std::string& s)
{
	if (!cliptr)
		return ~0;
	struct cli_arg *cli = (struct cli_arg*)cliptr;
	return sendto(cli->fd, s.c_str(), s.size(), 0,
		(struct sockaddr*)&cli->srv_addr, sizeof(cli->srv_addr));
}

