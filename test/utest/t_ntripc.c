// NTRIP caster extended status must not read uninitialized fields.
// Run with -fsanitize=memory -fsanitize-memory-param-retval to check
// the status formatting path, including the variadic sprintf argument.
#include "../../src/rtklib.h"

int main(void) {
    strinitcom();
    stream_t stream;
    strinit(&stream);
    int result = 1;
    // Port zero requests an available port; no external server is needed.
    if (stropen(&stream, STR_NTRIPCAS, STR_MODE_RW, ":0/TEST")) {
        // Poll once to publish the listening state. Without this, extended
        // status returns early and does not read the caster type.
        uint8_t byte;
        strread(&stream, &byte, 1);
        char status[16384] = {0};
        if (strstatx(&stream, status) == 1 && strstr(status, "  type    = 0\n") &&
            strstr(status, "  mntpnt  = TEST\n")) {
            result = 0;
        } else {
            fprintf(stderr, "Unexpected caster status:\n%s", status);
        }
    } else {
        fprintf(stderr, "Cannot open caster: %s\n", stream.msg);
    }

    strclose(&stream);
#ifdef WIN32
    DeleteCriticalSection(&stream.lock);
    WSACleanup();
#else
    pthread_mutex_destroy(&stream.lock);
#endif
    return result;
}
