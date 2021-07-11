#include <stdio.h>
#include <stdlib.h>

void main(int argc, char *argv[]) {
  char * buffer = 0;
  long length;
  FILE * f = fopen ("/opt/RTKLIB/SiRF/GSD4e_4.1.2_P1_RPATCH_10.pd2","rb");

  if (f) {
    fseek (f, 0, SEEK_END);
    length = ftell (f);
    fseek (f, 0, SEEK_SET);
    buffer = malloc (length);
    if (buffer) {
      fread (buffer, 1, length, f);
    }
    fclose (f);
  }
  fprintf(stderr,"L=%ld\n",length);

} 