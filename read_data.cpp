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
int* ptr;

double conv_pos_to_enc(double pos){

    return 1024*50*pos;

}

int compute_traj_kinematics (unsigned int joint_id, double desired_joint_acc, double desired_joint_vel, double ini_pos, double final_pos);


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

    for (unsigned int jnt_cnt = 0; jnt_cnt < 3; jnt_cnt++){
        
        ini_pos = ptr[11 + 3*jnt_cnt];
        final_pos = 2*M_PI + 0.2;

        compute_traj_kinematics (jnt_cnt, 0.5 , 0.2, ini_pos, final_pos);

    }

    return 0;
    
}

int compute_traj_kinematics (unsigned int joint_id, double desired_joint_acc, double desired_joint_vel, double ini_pos, double final_pos)
{

    double acc_time, cruise_time, deacc_time;

    // Updating the sign of acceleration and velocity
    if (fabs((final_pos - ini_pos)) > 1e-5)
    {
        desired_joint_acc = (final_pos - ini_pos) / fabs( (final_pos - ini_pos) ) * desired_joint_acc;
        desired_joint_vel = (final_pos - ini_pos) / fabs( (final_pos - ini_pos) )  * desired_joint_vel;
    }
    else
    {
        desired_joint_acc = 0;
        desired_joint_vel = 0;
    }

    // 

    double min_pos_travel = 0;

    min_pos_travel = desired_joint_vel * desired_joint_vel / desired_joint_acc;

    if (fabs(final_pos - ini_pos) < 1e-5){
        acc_time = 0;
        cruise_time = 0;
        deacc_time = 0;
    }
    else{

        if (fabs(final_pos - ini_pos) < fabs(min_pos_travel))
        {
            acc_time = sqrt((final_pos - ini_pos) / desired_joint_acc);
            cruise_time = 0;
            deacc_time = acc_time;
        }
        else
        {
            acc_time = desired_joint_vel / desired_joint_acc;
            cruise_time = (final_pos - ini_pos - desired_joint_vel * acc_time) / desired_joint_vel;
            deacc_time = acc_time;
        }

    }

    double current_pos;

    std::cout<<" joint_id : "<<joint_id<<std::endl;
    std::cout<<" desired_joint_acc : "<<desired_joint_acc<<", desired_joint_vel : "<<desired_joint_vel<<std::endl;
    std::cout<<" ini_pos : "<<ini_pos<<", final_pos : "<<final_pos<<std::endl;
    std::cout<<" acc_time : "<<acc_time<<", cruise_time : "<<cruise_time<<std::endl;

    std::cout<<"11 + 3*joint_id + 2 : "<<11 + 3*joint_id + 2<<std::endl;

    double time = 0;
    double total_time = acc_time*2 + cruise_time;

    while (time < total_time && total_time > 0)
    {
        if (time < acc_time)
        {
            current_pos = 0.5 * desired_joint_acc * time * time + ini_pos;
        }

        else if (time < (acc_time + cruise_time))
        {
            current_pos = ini_pos + 0.5 * desired_joint_acc * acc_time * acc_time + desired_joint_acc * acc_time * (time - acc_time);
        }

        else if ( time < (deacc_time + cruise_time + acc_time) )
        {
            current_pos = final_pos - 0.5 * desired_joint_acc *(deacc_time + cruise_time + acc_time - time)*(deacc_time + cruise_time + acc_time - time);
        }

        ptr[11 + 3*joint_id + 2] = conv_pos_to_enc(current_pos);

        // std::cout<<"enc_count : "<<conv_pos_to_enc(current_pos)<<std::endl;

        time = time + 0.005;

        usleep(5000);
    }

    return 0;

}

