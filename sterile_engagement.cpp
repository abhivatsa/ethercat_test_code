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

int joint_torq_lim[3] = {200, 75, 200};

double conv_pos_to_enc(double pos){

    return 1024*50*pos;

}

double conv_enc_to_pos(double pos){

    return pos/(1024*50);

}

int compute_traj_kinematics (unsigned int joint_id, double desired_joint_acc, double desired_joint_vel, double ini_pos, double final_pos);


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

    int sterile_adapter_attached = 1;
    int instrument_attached = 1;

    std::cout<<"line 49 \n";

    double ini_pos, final_pos;

    ptr1[0] = -1;

    while (sterile_adapter_attached == 1)
    {
        sterile_adapter_attached = ptr1[10];
    }

    std::cout<<"sterile_adapter_attached : "<<sterile_adapter_attached<<std::endl;

    ptr1[0] = 0;

    sleep(3.0);

    ptr1[1] = -1;

    for (unsigned int jnt_cnt = 0; jnt_cnt < 3; jnt_cnt++){
        
        ini_pos = conv_enc_to_pos(ptr1[11 + 3*jnt_cnt]);
        // final_pos = 2*M_PI + 0.2;
        final_pos = ini_pos + 2*M_PI + 0.2;

        // if (jnt_cnt != 1)
        // {
            
        // }

        compute_traj_kinematics (jnt_cnt, 0.5, 0.2, ini_pos, final_pos);

    }

    std::cout<<"sterile adapter attached \n";

    ptr1[1] = 0;

    std::cout<<"instrument_attached: "<<instrument_attached<<std::endl;
    ptr1[2] = -1;

    while (instrument_attached == 1)
    {
        instrument_attached = ptr1[8];
    }

    std::cout<<"instrument_attached: "<<instrument_attached<<std::endl;

    ptr1[2] = 0;

    ptr1[3] = -1;

    // for (unsigned int jnt_cnt = 0; jnt_cnt < 3; jnt_cnt++){
        
    //     ini_pos = conv_enc_to_pos(ptr1[11 + 3*jnt_cnt]);
    //     // final_pos = 2*M_PI + 0.2;
    //     final_pos = ini_pos + 2*M_PI + 0.2;

    //     // if (jnt_cnt != 1)
    //     // {
            
    //     // }

    //     compute_traj_kinematics (jnt_cnt, 0.5, 0.2, ini_pos, final_pos);

    // }

    // ptr1[3] = 0;

    int joint_1_pos_cnt, joint_2_pos_cnt, joint_3_pos_cnt; 

    joint_1_pos_cnt = ptr1[11];
    joint_2_pos_cnt = ptr1[14];
    joint_3_pos_cnt = ptr1[17];

    while (instrument_attached == 0){
        ptr1[13] = joint_1_pos_cnt + ptr1[4];
        ptr1[16] = joint_1_pos_cnt + ptr1[5];
        ptr1[19] = joint_1_pos_cnt + ptr1[6];
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

    int torq_break_ctr = 0;
    int loop_ctr = 0;

    bool engaged = false;

    while (time < total_time && total_time > 0 && engaged == false)
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

        ptr1[11 + 3*joint_id + 2] = conv_pos_to_enc(current_pos);

        // std::cout<<"enc_count : "<<conv_pos_to_enc(current_pos)<<std::endl;

        time = time + 0.005;

        usleep(5000);

        loop_ctr++;

        if (fabs(ptr1[11 + 3*joint_id + 1]) > joint_torq_lim[joint_id]){
            torq_break_ctr++;
            // std::cout<<"Actual torq_cross_the limit : "<<ptr1[11 + 3*joint_id + 1]<<std::endl;
        }

        if (loop_ctr > 400){
            if (torq_break_ctr > 200){
                ptr1[11 + 3*joint_id + 2] = ptr1[11 + 3*joint_id];
                std::cout<<" Torque limit crossed "<<std::endl;
                engaged = true;
                break;
            }
            else{
                torq_break_ctr = 0;
            }
            loop_ctr = 0;
        }
    }

    

    return 0;

}
