#include <stdio.h>
#include "cariboulite.h"
#include "cariboulite_setup.h"
#include "cariboulite_config/cariboulite_config.h"

extern cariboulite_st sys;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("ICE40 FPGA Programming Tool for the CaribouLite Board\n");
        printf("---------------------------\n\n");
        printf("Usage: use the .bin file output only!\n");
        printf("  ice40_programmer <fpga .bin file full path>\n");
        return 0;
    }
    else
    {
        printf("Programming bin file '%s'\n", argv[1]);
    }

    cariboulite_setup_io (&sys, NULL);
    cariboulite_configure_fpga (&sys, argv[1]);
    cariboulite_release_io (&sys);

    return 0;
}