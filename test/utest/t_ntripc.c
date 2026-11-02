// NTRIP caster: clients that never send a request must not hold a client
// slot forever. Uses a fake tick counter, so no wall-clock waiting is needed.
#define _DEFAULT_SOURCE
#include <stdint.h>
uint32_t test_tickget(void);
#define tickget test_tickget
#include "../../src/stream.c"
#undef tickget
#include <poll.h>
#include <unistd.h>

static uint32_t now = 1000;

uint32_t test_tickget(void) { return now; }

static int check(int condition, const char *message) {
    if (!condition) fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

static void pump(ntripc_t *ntripc) {
    uint8_t data[64];
    char msg[1024] = "";
    memset(data, 0xd3, sizeof(data));
    writentripc(ntripc, data, sizeof(data), msg);
}

static int nclients(ntripc_t *ntripc) {
    int n = 0;
    for (int i = 0; i < MAXCLI; i++) n += ntripc->tcp->cli[i].state == 2;
    return n;
}

static int connect_client(int port) {
    struct sockaddr_in addr = {0};
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr))) {
        close(sock);
        return -1;
    }
    return sock;
}

// Accept pending connections until the caster holds n clients.
static int wait_clients(ntripc_t *ntripc, int n) {
    for (int i = 0; i < 1000 && nclients(ntripc) != n; i++) {
        pump(ntripc);
        usleep(1000);
    }
    return nclients(ntripc) == n;
}

static int recv_wait(int sock, char *buff, int size) {
    struct pollfd pfd = {sock, POLLIN, 0};
    if (poll(&pfd, 1, 2000) <= 0) return -1;
    return (int)recv(sock, buff, size, 0);
}

int main(void) {
    char msg[1024] = "", buff[1024];
    struct sockaddr_in addr;
    socklen_t len = sizeof(addr);
    int idle[MAXCLI], ok = 1;

    ntripc_t *ntripc = openntripc(":0/TEST", msg);
    if (!check(ntripc != NULL, "open caster")) return 1;
    getsockname(ntripc->tcp->svr.sock, (struct sockaddr *)&addr, &len);
    int port = ntohs(addr.sin_port);

    // Fill all client slots with clients that never send a request.
    for (int i = 0; i < MAXCLI; i++) {
        idle[i] = connect_client(port);
        ok &= check(idle[i] >= 0 && wait_clients(ntripc, i + 1), "accept idle client");
    }
    int rover = connect_client(port);
    ok &= check(rover >= 0, "connect rover");
    pump(ntripc);
    ok &= check(nclients(ntripc) == MAXCLI, "rover not accepted while slots are full");

    // Idle clients are kept up to the timeout and dropped after it.
    now += NTRIP_TOREQ;
    pump(ntripc);
    ok &= check(nclients(ntripc) == MAXCLI, "idle clients kept until timeout");
    now += 1;
    ok &= check(wait_clients(ntripc, 1), "idle clients dropped, rover accepted");
    ok &= check(recv_wait(idle[0], buff, sizeof(buff)) == 0, "idle client disconnected");

    // The rover gets its stream, and is not affected by the request timeout.
    const char *req = "GET /TEST HTTP/1.0\r\nUser-Agent: NTRIP test\r\n\r\n";
    ok &= check(send(rover, req, strlen(req), 0) == (ssize_t)strlen(req), "send request");
    pump(ntripc);
    now += 2 * NTRIP_TOREQ;
    pump(ntripc);
    int n = recv_wait(rover, buff, sizeof(buff));
    ok &= check(n > (int)strlen(NTRIP_RSP_OK_CLI) &&
                    !strncmp(buff, NTRIP_RSP_OK_CLI, strlen(NTRIP_RSP_OK_CLI)),
                "rover receives response and data");
    ok &= check(nclients(ntripc) == 1, "rover kept after timeout");

    for (int i = 0; i < MAXCLI; i++) close(idle[i]);
    close(rover);
    closentripc(ntripc);
    return ok ? 0 : 1;
}
