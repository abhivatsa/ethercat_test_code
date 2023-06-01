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

/* pointer to shared memory object */
int* ptr1;

int main()
{

   /* the size (in bytes) of shared memory object */
    const int SIZE = sizeof(int[20]);
 
    /* shared memory file descriptor */
    int shm_fd;
 
    /* open the shared memory object */
    shm_fd = shm_open("ethercat_data_exchange", O_CREAT | O_RDWR, 0666);
 
    /* memory map the shared memory object */
    ptr1 = static_cast<int*> (mmap(0, SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, shm_fd, 0));

    ptr1[0] = 1;
    ptr1[1] = 1;
    ptr1[2] = 1;
    ptr1[3] = 1;

    std::cout<<"ptr1[11] : "<<ptr1[11]<<std::endl;
    std::cout<<", ptr1[14] : "<<ptr1[14]<<std::endl;
    std::cout<<", ptr1[17] : "<<ptr1[17]<<std::endl;

    return 0;
    
}