#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>

int main()
{
    /* the size (in bytes) of shared memory object */
    const int SIZE = sizeof(double[20]);
 
    /* name of the shared memory object */
    const char* name = "OS";
 
    /* shared memory file descriptor */
    int shm_fd;
 
    /* pointer to shared memory object */
    int* ptr;
 
    /* open the shared memory object */
    shm_fd = shm_open("ethercat_data_exchange", O_CREAT | O_RDWR, 0666);
 
    /* memory map the shared memory object */
    ptr = mmap(0, SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, shm_fd, 0);



    while(1){
        printf("joint1 pos : %d, joint1 torq : %d\n", ptr[0], ptr[1]); // Actual pos, actual torq joint 1
        printf("joint2 pos : %d, joint2 torq : %d\n", ptr[3], ptr[4]); // Actual pos, actual torq joint 1
        printf("joint3 pos : %d, joint3 torq : %d\n", ptr[6], ptr[7]); // Actual pos, actual torq joint 1

        ptr[2] = 200000; // Desired pos joint1
        ptr[5] = 200000; // Desired pos joint2
        ptr[8] = 200000; // Desired pos joint3

    }
}