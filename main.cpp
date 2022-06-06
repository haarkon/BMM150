 // Copyright (c) 2020 Antoine TRAN TAN
 // Copyright (c) 2017 Sarah MARSH

#include "mbed.h"
#include "rtos.h"
#include "BMM150.h"

BMM150 compass (PE_6,PE_5,PE_2,PE_4);

int main()
{
    float cap;
    int cr;

    printf("Boot Up - NUCLEO\n");

    cr = compass.initialize(BMM150_PRESETMODE_HIGHACCURACY);
    if (cr!=0) printf("Erreur initialisation\n");
    else printf("Initialisation complete\n");


    while(true) {
        cap = compass.getBearing();
        printf ("heading = %f\n",cap);
        ThisThread::sleep_for(50ms);
    }
}
