#include <stdlib.h>
#include "apriltags/apriltag.h"

apriltag_family_t *tag16h6_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->h = 6;
   tf->ncodes = 17;
   tf->codes = calloc(17, sizeof(uint64_t));
   tf->codes[0] = 0x00000000000081ecUL;
   tf->codes[1] = 0x0000000000008d76UL;
   tf->codes[2] = 0x000000000000aa4fUL;
   tf->codes[3] = 0x000000000000b014UL;
   tf->codes[4] = 0x000000000000bb9eUL;
   tf->codes[5] = 0x000000000000743eUL;
   tf->codes[6] = 0x0000000000004d58UL;
   tf->codes[7] = 0x00000000000058e2UL;
   tf->codes[8] = 0x00000000000065e9UL;
   tf->codes[9] = 0x0000000000009b53UL;
   tf->codes[10] = 0x000000000000d0bdUL;
   tf->codes[11] = 0x00000000000033a8UL;
   tf->codes[12] = 0x0000000000006b36UL;
   tf->codes[13] = 0x000000000000dc71UL;
   tf->codes[14] = 0x00000000000046fcUL;
   tf->codes[15] = 0x000000000000a7b4UL;
   tf->codes[16] = 0x0000000000001a2eUL;
   tf->nbits = 16;
   tf->bit_x = calloc(16, sizeof(uint32_t));
   tf->bit_y = calloc(16, sizeof(uint32_t));
   tf->bit_x[0] = 1;
   tf->bit_y[0] = 1;
   tf->bit_x[1] = 2;
   tf->bit_y[1] = 1;
   tf->bit_x[2] = 3;
   tf->bit_y[2] = 1;
   tf->bit_x[3] = 2;
   tf->bit_y[3] = 2;
   tf->bit_x[4] = 4;
   tf->bit_y[4] = 1;
   tf->bit_x[5] = 4;
   tf->bit_y[5] = 2;
   tf->bit_x[6] = 4;
   tf->bit_y[6] = 3;
   tf->bit_x[7] = 3;
   tf->bit_y[7] = 2;
   tf->bit_x[8] = 4;
   tf->bit_y[8] = 4;
   tf->bit_x[9] = 3;
   tf->bit_y[9] = 4;
   tf->bit_x[10] = 2;
   tf->bit_y[10] = 4;
   tf->bit_x[11] = 3;
   tf->bit_y[11] = 3;
   tf->bit_x[12] = 1;
   tf->bit_y[12] = 4;
   tf->bit_x[13] = 1;
   tf->bit_y[13] = 3;
   tf->bit_x[14] = 1;
   tf->bit_y[14] = 2;
   tf->bit_x[15] = 2;
   tf->bit_y[15] = 3;
   tf->width_at_border = 6;
   tf->total_width = 8;
   tf->reversed_border = false;
   return tf;
}

void tag16h6_destroy(apriltag_family_t *tf)
{
   free(tf->codes);
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf);
}
