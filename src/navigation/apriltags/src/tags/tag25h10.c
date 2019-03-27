#include <stdlib.h>
#include "apriltags/apriltag.h"

apriltag_family_t *tag25h10_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->h = 10;
   tf->ncodes = 22;
   tf->codes = calloc(22, sizeof(uint64_t));
   tf->codes[0] = 0x0000000001a3d688UL;
   tf->codes[1] = 0x000000000032dc4dUL;
   tf->codes[2] = 0x00000000006ef361UL;
   tf->codes[3] = 0x0000000000fdf926UL;
   tf->codes[4] = 0x0000000001c915ffUL;
   tf->codes[5] = 0x00000000005e13dbUL;
   tf->codes[6] = 0x00000000000b252aUL;
   tf->codes[7] = 0x0000000001b83679UL;
   tf->codes[8] = 0x00000000010f7b3dUL;
   tf->codes[9] = 0x0000000000e20866UL;
   tf->codes[10] = 0x0000000001613764UL;
   tf->codes[11] = 0x0000000001154989UL;
   tf->codes[12] = 0x0000000001f303d0UL;
   tf->codes[13] = 0x00000000010b86d3UL;
   tf->codes[14] = 0x0000000000aea23fUL;
   tf->codes[15] = 0x000000000185527aUL;
   tf->codes[16] = 0x0000000000efc4a5UL;
   tf->codes[17] = 0x00000000012a7193UL;
   tf->codes[18] = 0x00000000005c770cUL;
   tf->codes[19] = 0x0000000000b75dc2UL;
   tf->codes[20] = 0x000000000047a79dUL;
   tf->codes[21] = 0x0000000000acc1f2UL;
   tf->nbits = 25;
   tf->bit_x = calloc(25, sizeof(uint32_t));
   tf->bit_y = calloc(25, sizeof(uint32_t));
   tf->bit_x[0] = 1;
   tf->bit_y[0] = 1;
   tf->bit_x[1] = 2;
   tf->bit_y[1] = 1;
   tf->bit_x[2] = 3;
   tf->bit_y[2] = 1;
   tf->bit_x[3] = 4;
   tf->bit_y[3] = 1;
   tf->bit_x[4] = 2;
   tf->bit_y[4] = 2;
   tf->bit_x[5] = 3;
   tf->bit_y[5] = 2;
   tf->bit_x[6] = 5;
   tf->bit_y[6] = 1;
   tf->bit_x[7] = 5;
   tf->bit_y[7] = 2;
   tf->bit_x[8] = 5;
   tf->bit_y[8] = 3;
   tf->bit_x[9] = 5;
   tf->bit_y[9] = 4;
   tf->bit_x[10] = 4;
   tf->bit_y[10] = 2;
   tf->bit_x[11] = 4;
   tf->bit_y[11] = 3;
   tf->bit_x[12] = 5;
   tf->bit_y[12] = 5;
   tf->bit_x[13] = 4;
   tf->bit_y[13] = 5;
   tf->bit_x[14] = 3;
   tf->bit_y[14] = 5;
   tf->bit_x[15] = 2;
   tf->bit_y[15] = 5;
   tf->bit_x[16] = 4;
   tf->bit_y[16] = 4;
   tf->bit_x[17] = 3;
   tf->bit_y[17] = 4;
   tf->bit_x[18] = 1;
   tf->bit_y[18] = 5;
   tf->bit_x[19] = 1;
   tf->bit_y[19] = 4;
   tf->bit_x[20] = 1;
   tf->bit_y[20] = 3;
   tf->bit_x[21] = 1;
   tf->bit_y[21] = 2;
   tf->bit_x[22] = 2;
   tf->bit_y[22] = 4;
   tf->bit_x[23] = 2;
   tf->bit_y[23] = 3;
   tf->bit_x[24] = 3;
   tf->bit_y[24] = 3;
   tf->width_at_border = 7;
   tf->total_width = 9;
   tf->reversed_border = false;
   return tf;
}

void tag25h10_destroy(apriltag_family_t *tf)
{
   free(tf->codes);
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf);
}
