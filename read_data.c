#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>

int main()
{
    // Open a pipe to gnuplot
    FILE *gnuplotPipe = popen("gnuplot -persist", "w");
    if (gnuplotPipe == NULL) {
        perror("popen");
        return -1;
    }

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

    // Configure the plot
    fprintf(gnuplotPipe, "set xlabel 'X'\n");
    fprintf(gnuplotPipe, "set ylabel 'Y'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'Integer Data'\n");

    double act_torq = 0, act_torq1 = 0, act_torq2 = 0;

    FILE *fp;
    fp = fopen("data.csv", "a");
    if (fp == NULL) {
        perror("Error opening file");
        return 1;
    }


    double time = 0;

    int ctr = 0;

    while(1){
        printf("joint1 pos : %d, joint1 torq : %d\n", ptr[0], ptr[1]); // Actual pos, actual torq joint 1
        printf("joint2 pos : %d, joint2 torq : %d\n", ptr[3], ptr[4]); // Actual pos, actual torq joint 1
        printf("joint3 pos : %d, joint3 torq : %d\n", ptr[6], ptr[7]); // Actual pos, actual torq joint 1

        ptr[2] = 200000; // Desired pos joint1
        ptr[5] = 200000; // Desired pos joint2
        ptr[8] = 200000; // Desired pos joint3

        ctr++;

        act_torq = ptr[1];
        act_torq1 = ptr[4];
        act_torq2 = ptr[7];

        time = time + 0.001;

        if (fprintf(fp, "%f,%f,%f\n", act_torq, act_torq1, act_torq2) < 0) {
            perror("Error writing to file");
            return 1;
        }

        // fprintf(gnuplotPipe, "%f %f\n", time, act_torq);

        usleep(1000);

    }

    // fprintf(gnuplotPipe, "e\n");

    // // // Close the pipe to gnuplot
    // pclose(gnuplotPipe);
}