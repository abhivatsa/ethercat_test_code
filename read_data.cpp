#include <iostream>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fstream>


/* pointer to shared memory object */
int* ptr;

int main()
{

   /* the size (in bytes) of shared memory object */
    const int SIZE = sizeof(double[20]);
 
    /* shared memory file descriptor */
    int shm_fd;
 
    /* open the shared memory object */
    shm_fd = shm_open("ethercat_data_exchange", O_CREAT | O_RDWR, 0666);
 
    /* memory map the shared memory object */
    ptr = static_cast<int*> (mmap(0, SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, shm_fd, 0));

    int sterile_adapter_attached = 0;

    // while (sterile_adapter_attached == 1)
    // {
    //     sterile_adapter_attached = ptr[0];
    // }

    sleep(3.0);

    double ini_pos, final_pos;

    std::ofstream outfile;
    outfile.open("data.csv", std::ios::out);

    outfile<<"act_pos0,"<<"act_pos1,"<<"act_pos2,"<<"act_torq0,"<<"act_torq1,"<<"act_torq2\n";

    while(1){
        outfile<<ptr[11]<<","<<ptr[14]<<","<<ptr[17]<<","<<ptr[12]<<","<<ptr[15]<<","<<ptr[18]<<"\n";
        usleep(10000);
    }

    outfile.close();

    return 0;
    
}

