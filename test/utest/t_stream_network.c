/* Loopback transport tests. Explicit checks remain active in Release builds. */
#define _DEFAULT_SOURCE
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>
static int test_getaddrinfo(const char *, const char *, const struct addrinfo *, struct addrinfo **);
static void test_freeaddrinfo(struct addrinfo *);
static int test_socket(int, int, int);
#define getaddrinfo test_getaddrinfo
#define freeaddrinfo test_freeaddrinfo
#define socket test_socket
#include "../../src/stream.c"
#undef getaddrinfo
#undef freeaddrinfo
#undef socket

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#condition); return 0; } } while (0)

/* Force an IPv6 candidate before an IPv4-only listener, independently of DNS. */
static struct addrinfo *forced_addresses;
static int forced_resolutions,forced_frees;
static int force_no_ipv6,forced_ipv6_sockets;

static int test_socket(int family, int type, int protocol)
{
    if (force_no_ipv6&&family==AF_INET6) {
        forced_ipv6_sockets++;
        errno=EAFNOSUPPORT;
        return -1;
    }
    return socket(family,type,protocol);
}

static int test_getaddrinfo(const char *host, const char *service,
                            const struct addrinfo *hints, struct addrinfo **out)
{
    int wildcard=!host&&force_no_ipv6;
    if (!wildcard&&(!host||strcmp(host,"ipv6-fallback.test")))
        return getaddrinfo(host,service,hints,out);
    struct addrinfo *tail=NULL;
    /* Build both families even when this host has no configured IPv6. */
    for (int i=0;i<2;i++) {
        struct addrinfo *node=calloc(1,sizeof(*node));
        if (!node) abort();
        node->ai_family=i?AF_INET:AF_INET6;
        node->ai_socktype=hints->ai_socktype;
        node->ai_protocol=hints->ai_protocol;
        node->ai_addrlen=i?sizeof(struct sockaddr_in):sizeof(struct sockaddr_in6);
        node->ai_addr=calloc(1,node->ai_addrlen);
        if (!node->ai_addr) abort();
        if (i) {
            struct sockaddr_in *addr=(struct sockaddr_in *)node->ai_addr;
            addr->sin_family=AF_INET;
            addr->sin_port=htons(atoi(service));
            addr->sin_addr.s_addr=htonl(wildcard?INADDR_ANY:INADDR_LOOPBACK);
        } else {
            struct sockaddr_in6 *addr=(struct sockaddr_in6 *)node->ai_addr;
            addr->sin6_family=AF_INET6;
            addr->sin6_port=htons(atoi(service));
            addr->sin6_addr=wildcard?in6addr_any:in6addr_loopback;
        }
        if (tail) tail->ai_next=node; else forced_addresses=node;
        tail=node;
    }
    forced_resolutions++;
    *out=forced_addresses;
    return 0;
}

static void test_freeaddrinfo(struct addrinfo *addresses)
{
    if (addresses!=forced_addresses) { freeaddrinfo(addresses); return; }
    while (addresses) {
        struct addrinfo *next=addresses->ai_next;
        free(addresses->ai_addr); free(addresses); addresses=next;
    }
    forced_addresses=NULL;
    forced_frees++;
}

/* Probe the OS directly: an RTKLIB regression must fail, never become a skip. */
static int ipv6_available(int type, int dual)
{
    int sock=socket(AF_INET6,type,0),error=0,only=dual?0:1;
    if (sock<0) error=errno;
    else {
        struct sockaddr_in6 addr={0};
        addr.sin6_family=AF_INET6;
        addr.sin6_addr=dual?in6addr_any:in6addr_loopback;
        if (setsockopt(sock,IPPROTO_IPV6,IPV6_V6ONLY,&only,sizeof(only))<0||
            bind(sock,(struct sockaddr *)&addr,sizeof(addr))<0||
            (type==SOCK_STREAM&&listen(sock,1)<0)) error=errno;
        close(sock);
    }
    if (!error) return 1;
    if (error==EAFNOSUPPORT||error==EPROTONOSUPPORT||error==ENOPROTOOPT||
        error==EADDRNOTAVAIL||error==ENODEV) {
        fprintf(stderr,"SKIP: IPv6 %s%s unavailable: %s\n",
                type==SOCK_STREAM?"TCP":"UDP",dual?" dual stack":"",strerror(error));
        return 0;
    }
    fprintf(stderr,"FAIL: IPv6 capability probe: %s\n",strerror(error));
    return -1;
}

/* TCP is a byte stream. Small reads exercise accumulation deterministically. */
static int send_bytes(stream_t *stream, const uint8_t *data, int size, int *sent)
{
    if (*sent>=size) return 1;
    int count=strwrite(stream,data+*sent,size-*sent);
    CHECK(count>=0&&count<=size-*sent);
    *sent+=count;
    return 1;
}

static int receive_bytes(stream_t *stream, uint8_t *data, int size, int *received)
{
    if (*received>=size) return 1;
    int remaining=size-*received;
    int count=strread(stream,data+*received,remaining<3?remaining:3);
    CHECK(count>=0&&count<=remaining);
    *received+=count;
    return 1;
}

static int socket_port(socket_t sock)
{
    struct sockaddr_storage addr;
    socklen_t len=sizeof(addr);
    if (getsockname(sock,(struct sockaddr *)&addr,&len)) return -1;
    if (addr.ss_family==AF_INET6) return ntohs(((struct sockaddr_in6 *)&addr)->sin6_port);
    return ntohs(((struct sockaddr_in *)&addr)->sin_port);
}

static void endpoint(char *path, const char *host, int port)
{
    if (strchr(host,':')) sprintf(path,"[%s]:%d",host,port);
    else sprintf(path,"%s:%d",host,port);
}

static int paths(void)
{
    const char *inputs[]={"127.0.0.1:2101","localhost:2101",":2101",
        "user:pa:ss@[::1]:2101/MOUNT:STR;TEST", "[fe80::1%en0]:2101",
        "[::1]/MOUNT","::1","[2001:db8::1]:2101"};
    const char *hosts[]={"127.0.0.1","localhost","","::1","fe80::1%en0",
        "::1","::1","2001:db8::1"};
    const char *ports[]={"2101","2101","2101","2101","2101","","","2101"};
    for (int i=0;i<8;i++) {
        char host[256]="",port[256],user[256],password[256],mount[256],str[256];
        decodetcppath(inputs[i],host,port,user,password,mount,str);
        CHECK(!strcmp(host,hosts[i])&&!strcmp(port,ports[i]));
        if (i==3) CHECK(!strcmp(user,"user")&&!strcmp(password,"pa:ss")&&
                       !strcmp(mount,"MOUNT")&&!strcmp(str,"STR;TEST"));
    }
    int opened_stdin=-1;
    if (fcntl(STDIN_FILENO,F_GETFD)==-1) {
        CHECK(errno==EBADF);
        opened_stdin=open("/dev/null",O_RDONLY);
        CHECK(opened_stdin==STDIN_FILENO);
    }
    stream_t unopened;
    strinit(&unopened);
    CHECK(stropen(&unopened,STR_TCPCLI,STR_MODE_RW,"[::1]:2101"));
    strclose(&unopened);
    CHECK(fcntl(STDIN_FILENO,F_GETFD)!=-1); /* closing before connect must not close stdin */
    if (opened_stdin>=0) close(opened_stdin);
    return 1;
}

static int tcp_exchange(stream_t *server, const char *host, int fallback)
{
    stream_t client;
    char path[512],msg[4096];
    uint8_t received[128],reply[128];
    const uint8_t payload[]="GNSS IPv4/IPv6 stream\r\n";
    int sent=0,n=0,back=0,backsent=0;
    strinit(&client);
    endpoint(path,host,socket_port(((tcpsvr_t *)server->port)->svr.sock));
    CHECK(stropen(&client,STR_TCPCLI,STR_MODE_RW,path));
    for (int i=0;i<1000&&(back<sizeof(payload)||n<sizeof(payload));i++) {
        CHECK(send_bytes(&client,payload,sizeof(payload),&sent));
        CHECK(receive_bytes(server,received,sizeof(payload),&n));
        if (n==sizeof(payload)) CHECK(send_bytes(server,payload,sizeof(payload),&backsent));
        CHECK(receive_bytes(&client,reply,sizeof(payload),&back));
        sleepms(2);
    }
    CHECK(n==sizeof(payload)&&back==sizeof(payload));
    CHECK(!memcmp(received,payload,sizeof(payload))&&!memcmp(reply,payload,sizeof(payload)));
    const char *expected=!strcmp(host,"::1")?"::1":"127.0.0.1";
    tcpsvr_t *tcp=server->port;
    int peer_found=0;
    for (int i=0;i<MAXCLI;i++) {
        if (tcp->cli[i].state==2&&(!strcmp(tcp->cli[i].saddr,expected)||
            (!strcmp(expected,"127.0.0.1")&&!strcmp(tcp->cli[i].saddr,"::ffff:127.0.0.1")))) {
            peer_found=1;
        }
    }
    CHECK(peer_found);
    strstatx(server,msg);
    CHECK(strstr(msg,expected));
    if (fallback) CHECK(forced_resolutions==1&&forced_frees==1);
    strclose(&client);
    strread(server,received,sizeof(received));
    return 1;
}

static int tcp_test(const char *mode)
{
    int listener_fallback=!strcmp(mode,"listener_fallback");
    CHECK(!strcmp(mode,"ipv4")||!strcmp(mode,"ipv6")||!strcmp(mode,"dual")||
          !strcmp(mode,"fallback")||listener_fallback);
    stream_t server;
    strinit(&server);
    const char *bindaddr=!strcmp(mode,"ipv6")?"[::1]:0":
                         (!strcmp(mode,"dual")||listener_fallback)?":0":"127.0.0.1:0";
    force_no_ipv6=listener_fallback;
    CHECK(stropen(&server,STR_TCPSVR,STR_MODE_RW,bindaddr));
    if (listener_fallback) {
        CHECK(forced_ipv6_sockets>0&&forced_resolutions==1&&forced_frees==1);
        CHECK(((tcpsvr_t *)server.port)->svr.addr.ss_family==AF_INET);
    }
    if (!strcmp(mode,"dual")) {
        CHECK(((tcpsvr_t *)server.port)->svr.addr.ss_family==AF_INET6);
        CHECK(tcp_exchange(&server,"127.0.0.1",0));
        CHECK(tcp_exchange(&server,"::1",0));
    } else {
        const char *host=!strcmp(mode,"ipv6")?"::1":
                         !strcmp(mode,"fallback")?"ipv6-fallback.test":"127.0.0.1";
        CHECK(tcp_exchange(&server,host,!strcmp(mode,"fallback")));
    }
    strclose(&server);
    force_no_ipv6=0;
    return 1;
}

static int udp_exchange(stream_t *server, const char *host)
{
    stream_t client;
    char path[512];
    const uint8_t payload[]="GNSS UDP payload";
    uint8_t received[128];
    int n=0;
    strinit(&client);
    endpoint(path,host,socket_port(((udp_t *)server->port)->sock));
    CHECK(stropen(&client,STR_UDPCLI,STR_MODE_W,path));
    CHECK(strwrite(&client,payload,sizeof(payload))==sizeof(payload));
    for (int i=0;i<1000&&!n;i++) { n=strread(server,received,sizeof(received)); sleepms(2); }
    CHECK(n==sizeof(payload)&&!memcmp(received,payload,sizeof(payload)));
    strclose(&client);
    return 1;
}

static int udp_test(const char *mode)
{
    int listener_fallback=!strcmp(mode,"listener_fallback");
    CHECK(!strcmp(mode,"ipv4")||!strcmp(mode,"ipv6")||!strcmp(mode,"dual")||listener_fallback);
    stream_t server;
    strinit(&server);
    const char *bindaddr=!strcmp(mode,"ipv6")?"[::1]:0":
                         (!strcmp(mode,"dual")||listener_fallback)?":0":"127.0.0.1:0";
    force_no_ipv6=listener_fallback;
    CHECK(stropen(&server,STR_UDPSVR,STR_MODE_R,bindaddr));
    if (listener_fallback) {
        CHECK(forced_ipv6_sockets>0&&forced_resolutions==1&&forced_frees==1);
        CHECK(((udp_t *)server.port)->addr.ss_family==AF_INET);
    }
    if (!strcmp(mode,"dual")) {
        CHECK(((udp_t *)server.port)->addr.ss_family==AF_INET6);
        CHECK(udp_exchange(&server,"127.0.0.1"));
        CHECK(udp_exchange(&server,"::1"));
    } else CHECK(udp_exchange(&server,!strcmp(mode,"ipv6")?"::1":"127.0.0.1"));
    strclose(&server);
    force_no_ipv6=0;
    return 1;
}

static int ntrip_test(const char *mode)
{
    CHECK(!strcmp(mode,"ipv4")||!strcmp(mode,"ipv6"));
    stream_t caster,client;
    char addr[512],path[1024];
    const char *host=!strcmp(mode,"ipv6")?"::1":"127.0.0.1";
    uint8_t received[128];
    const uint8_t payload[]="RTCM3 binary payload\xd3\0\x01";
    int n=0,sent=0;
    strinit(&caster); strinit(&client);
    endpoint(addr,host,0);
    sprintf(path,"user:pa:ss@%s/MOUNT",addr);
    CHECK(stropen(&caster,STR_NTRIPCAS,STR_MODE_W,path));
    endpoint(addr,host,socket_port(((ntripc_t *)caster.port)->tcp->svr.sock));
    sprintf(path,"user:pa:ss@%s/MOUNT",addr);
    CHECK(stropen(&client,STR_NTRIPCLI,STR_MODE_R,path));
    for (int i=0;i<1000&&n<sizeof(payload);i++) {
        CHECK(send_bytes(&caster,payload,sizeof(payload),&sent));
        CHECK(receive_bytes(&client,received,sizeof(payload),&n));
        sleepms(2);
    }
    CHECK(n==sizeof(payload)&&!memcmp(received,payload,sizeof(payload)));
    strclose(&client); strclose(&caster);
    return 1;
}

typedef struct { tcpsvr_t *server; int ok; } source_test_t;
static void *source_caster(void *arg)
{
    source_test_t *test=arg;
    char request[1024]="";
    uint8_t received[64];
    int n=0;
    char msg[1024]="";
    for (int i=0;i<1000&&!strstr(request,"\r\n\r\n");i++) {
        int count=readtcpsvr(test->server,(uint8_t *)request+n,sizeof(request)-n-1,msg);
        if (count>0) { n+=count; request[n]='\0'; }
        sleepms(2);
    }
    if (!strstr(request,"\r\n\r\n")||!strstr(request,"SOURCE secret MOUNT\r\n")) return NULL;
    const uint8_t response[]="OK\r\n";
    int sent=0;
    n=0;
    for (int i=0;i<1000&&n<8;i++) {
        if (sent<sizeof(response)-1) {
            int count=writetcpsvr(test->server,response+sent,sizeof(response)-1-sent,msg);
            if (count<0||count>sizeof(response)-1-sent) return NULL;
            sent+=count;
        }
        int count=readtcpsvr(test->server,received+n,8-n<3?8-n:3,msg);
        if (count<0||count>8-n) return NULL;
        n+=count;
        sleepms(2);
    }
    test->ok=n==8&&!memcmp(received,"RTCMtest",8);
    return NULL;
}

static int ntrip_source(void)
{
    char msg[1024]="",path[512];
    source_test_t test={opentcpsvr("[::1]:0",msg),0};
    CHECK(test.server);
    pthread_t thread;
    CHECK(!pthread_create(&thread,NULL,source_caster,&test));
    stream_t source;
    strinit(&source);
    sprintf(path,":secret@[::1]:%d/MOUNT",socket_port(test.server->svr.sock));
    CHECK(stropen(&source,STR_NTRIPSVR,STR_MODE_W,path));
    int sent=0;
    for (int i=0;i<1000&&sent<8;i++) {
        CHECK(send_bytes(&source,(const uint8_t *)"RTCMtest",8,&sent));
        sleepms(2);
    }
    pthread_join(thread,NULL);
    CHECK(sent==8&&test.ok);
    strclose(&source); closetcpsvr(test.server);
    return 1;
}

static int proxy_path(void)
{
    char msg[1024]="";
    strsetproxy("[::1]:8080");
    ntrip_t *ntrip=openntrip("user:pw@[2001:db8::1]:2101/MOUNT",1,msg);
    CHECK(ntrip&& !strcmp(ntrip->url,"http://[2001:db8::1]:2101"));
    CHECK(!strcmp(ntrip->tcp->svr.saddr,"::1")&&ntrip->tcp->svr.port==8080);
    closentrip(ntrip);
    strsetproxy("");
    return 1;
}

/* Test the threaded STRSVR API as well as individual stream endpoints. */
static int stream_server(int tcp_output)
{
    stream_t input,output;
    strinit(&input); strinit(&output);
    CHECK(stropen(&input,STR_TCPSVR,STR_MODE_W,"[::1]:0"));
    char inpath[512],outpath[512];
    endpoint(inpath,"::1",socket_port(((tcpsvr_t *)input.port)->svr.sock));
    /* Different TCP ports must remain distinct; TCP/UDP may share one port. */
    endpoint(outpath,"::1",tcp_output?0:socket_port(((tcpsvr_t *)input.port)->svr.sock));
    CHECK(stropen(&output,tcp_output?STR_TCPSVR:STR_UDPSVR,STR_MODE_R,outpath));
    endpoint(outpath,"::1",socket_port(tcp_output?((tcpsvr_t *)output.port)->svr.sock:
                                                            ((udp_t *)output.port)->sock));
    CHECK(tcp_output?strcmp(inpath,outpath)!=0:!strcmp(inpath,outpath));
    const char *paths[]={inpath,outpath},*logs[]={"",""},*cmds[]={NULL,NULL};
    strconv_t *conversion[]={NULL};
    int types[]={STR_TCPCLI,tcp_output?STR_TCPCLI:STR_UDPCLI};
    int options[]={1000,10,100,4096,1,0,30,0};
    strsvr_t server;
    strsvrinit(&server,1);
    CHECK(strsvrstart(&server,options,types,paths,logs,conversion,cmds,cmds,NULL));
    const uint8_t payload[]="IPv6 stream forwarding";
    uint8_t received[128];
    if (tcp_output) {
        /* Connect the lazy output before forwarding the one-shot input. */
        int ready=0;
        for (int i=0;i<1000&&!ready;i++) {
            CHECK(strwrite(server.stream+1,payload,0)==0);
            CHECK(strread(&output,received,sizeof(received))==0);
            ready=strstat(server.stream+1,NULL)>=2&&strstat(&output,NULL)>=2;
            sleepms(2);
        }
        CHECK(ready);
    }
    int sent=0,n=0;
    for (int i=0;i<1000&&n<sizeof(payload);i++) {
        CHECK(send_bytes(&input,payload,sizeof(payload),&sent));
        if (tcp_output) {
            CHECK(receive_bytes(&output,received,sizeof(payload),&n));
        } else {
            int count=strread(&output,received+n,sizeof(received)-n);
            CHECK(count>=0&&count<=sizeof(payload)-n);
            n+=count;
        }
        sleepms(2);
    }
    strsvrstop(&server,cmds);
    CHECK(n==sizeof(payload)&&!memcmp(received,payload,sizeof(payload)));
    strclose(&input); strclose(&output);
    return 1;
}

int main(int argc, char **argv)
{
    if (argc!=2) return 1;
    signal(SIGPIPE,SIG_IGN);
    int tcp6=!strcmp(argv[1],"tcp_ipv6")||!strcmp(argv[1],"tcp_dual")||
             !strcmp(argv[1],"ntrip_ipv6")||!strcmp(argv[1],"ntrip_source")||
             !strcmp(argv[1],"server")||!strcmp(argv[1],"server_tcp");
    int udp6=!strcmp(argv[1],"udp_ipv6")||!strcmp(argv[1],"udp_dual")||
             !strcmp(argv[1],"server");
    if (tcp6) {
        int available=ipv6_available(SOCK_STREAM,!strcmp(argv[1],"tcp_dual"));
        if (available!=1) return available==0?77:1;
    }
    if (udp6) {
        int available=ipv6_available(SOCK_DGRAM,!strcmp(argv[1],"udp_dual"));
        if (available!=1) return available==0?77:1;
    }
    int ok=0;
    if (!strcmp(argv[1],"paths")) ok=paths();
    else if (!strncmp(argv[1],"tcp_",4)) ok=tcp_test(argv[1]+4);
    else if (!strncmp(argv[1],"udp_",4)) ok=udp_test(argv[1]+4);
    else if (!strcmp(argv[1],"ntrip_source")) ok=ntrip_source();
    else if (!strcmp(argv[1],"proxy")) ok=proxy_path();
    else if (!strcmp(argv[1],"server")) ok=stream_server(0);
    else if (!strcmp(argv[1],"server_tcp")) ok=stream_server(1);
    else if (!strncmp(argv[1],"ntrip_",6)) ok=ntrip_test(argv[1]+6);
    return ok?0:1;
}
