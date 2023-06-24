/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 ****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>     /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h>    /* sched_setscheduler() */
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <stdbool.h>

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

/** Task period in ns. */
#define PERIOD_NS (1000000)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is  \
                                     guranteed safe to access without \
                                     faulting */

/****************************************************************************/

/* Constants */
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain = NULL;
static ec_domain_state_t domain_state = {};

static ec_slave_config_t *sc_ana_in = NULL;
static ec_slave_config_state_t sc_ana_in_state = {};

/****************************************************************************/

int* data_ptr;
bool initialized_eth = false;

// process data
static uint8_t *domain_pd = NULL;

static int joint_torq1[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int joint_torq2[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int joint_torq3[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Master 0, Slave 0, "DEN-XCR-E-SI"
 * Vendor ID:       0x0000029c
 * Product code:    0x03831001
 * Revision number: 0x00050008
 */

#define denalli_xcr 0x0000029c, 0x03831002 // Vendor Id, Product Code
#define inhouse_slave 0x0000007b, 0x00009252 // Vendor Id, Product Code

#define denalli_xcr_pos0 0, 0 // Alias, Position
#define denalli_xcr_pos1 0, 1 // Alias, Position
#define denalli_xcr_pos2 0, 2 // Alias, Position
#define inhouse_slave_pos3 0, 3 // Alias, Position

static struct 
{
    unsigned int statusword;
    unsigned int position_actual_value;
    unsigned int torque_actual_value;
    unsigned int digital_input_value;
    // unsigned int current_actual_value;
    // unsigned int homing_offset;
    // unsigned int velocity_actual_value;
    unsigned int controlword;
    unsigned int modes_of_operation;
    unsigned int target_position;
    // unsigned int target_velocity;
    // unsigned int target_torque;
} offset, offset1, offset2;


static struct 
{
    unsigned int statusword;
    unsigned int controlword;
    unsigned int instrument_type;
    // unsigned int target_position;
    // unsigned int digital_io;
    // unsigned int target_velocity;
    // unsigned int target_torque;
} offset3;

// /** List record type for PDO entry mass-registration.
//  *
//  * This type is used for the array parameter of the
//  * ecrt_domain_reg_pdo_entry_list()
//  */
// typedef struct {
//     uint16_t alias; /**< Slave alias address. */
//     uint16_t position; /**< Slave position. */
//     uint32_t vendor_id; /**< Slave vendor ID. */
//     uint32_t product_code; /**< Slave product code. */
//     uint16_t index; /**< PDO entry index. */
//     uint8_t subindex; /**< PDO entry subindex. */
//     unsigned int *offset; /**< Pointer to a variable to store the PDO entry's
//                        (byte-)offset in the process data. */
//     unsigned int *bit_position; /**< Pointer to a variable to store a bit
//                                   position (0-7) within the \a offset. Can be
//                                   NULL, in which case an error is raised if the
//                                   PDO entry does not byte-align. */
// } ec_pdo_entry_reg_t;


const static ec_pdo_entry_reg_t domain_regs[] = {
    
    // --------------------- denalli_xcr_pos0 ------------------------ 

    {denalli_xcr_pos0, denalli_xcr, 0x6041, 0, &offset.statusword},             // 6041 0 statusword
    {denalli_xcr_pos0, denalli_xcr, 0x6064, 0, &offset.position_actual_value},  // 6064 0 pos_act_val
    {denalli_xcr_pos0, denalli_xcr, 0x6077, 0, &offset.torque_actual_value},    // 6077 0 torq_act_val
    {denalli_xcr_pos0, denalli_xcr, 0x2600, 0, &offset.digital_input_value},    // 2600 0 digital_input_value
    // {denalli_xcr_pos0, denalli_xcr, 0x2600, 0, &offset.current_actual_value},   // 6078 0 current_actual_value
    // {denalli_xcr_pos0, denalli_xcr, 0x2600, 0, &offset.homing_offset},          // 607C 0 homing_offset
    // {denalli_xcr_pos0, denalli_xcr, 0x606C, 0, &offset.velocity_actual_value},  // 606C 0 vel_act_val

    {denalli_xcr_pos0, denalli_xcr, 0x6040, 0, &offset.controlword},            // 6040 0 control word
    {denalli_xcr_pos0, denalli_xcr, 0x6060, 0, &offset.modes_of_operation},     // 6060 0 mode_of_operation
    {denalli_xcr_pos0, denalli_xcr, 0x607A, 0, &offset.target_position},        // 607A 0 target position
    // {denalli_xcr_pos0, denalli_xcr, 0x60FF, 0, &offset.target_velocity},        // 60FF 0 target velocity
    // {denalli_xcr_pos0, denalli_xcr, 0x6071, 0, &offset.target_torque},          // 6071 0 target torque

    // --------------------- denalli_xcr_pos1 ------------------------ 
    
    {denalli_xcr_pos1, denalli_xcr, 0x6041, 0, &offset1.statusword},             // 6041 0 statusword
    {denalli_xcr_pos1, denalli_xcr, 0x6064, 0, &offset1.position_actual_value},  // 6064 0 pos_act_val
    {denalli_xcr_pos1, denalli_xcr, 0x6077, 0, &offset1.torque_actual_value},    // 6077 0 torq_act_val
    {denalli_xcr_pos1, denalli_xcr, 0x2600, 0, &offset1.digital_input_value},    // 2600 0 digital_input_value
    // {denalli_xcr_pos1, denalli_xcr, 0x2600, 0, &offset1.current_actual_value},   // 6078 0 current_actual_value
    // {denalli_xcr_pos1, denalli_xcr, 0x2600, 0, &offset1.homing_offset},          // 607C 0 homing_offset
    // {denalli_xcr_pos1, denalli_xcr, 0x606C, 0, &offset1.velocity_actual_value},  // 606C 0 vel_act_val

    {denalli_xcr_pos1, denalli_xcr, 0x6040, 0, &offset1.controlword},            // 6040 0 control word
    {denalli_xcr_pos1, denalli_xcr, 0x6060, 0, &offset1.modes_of_operation},     // 6060 0 mode_of_operation
    {denalli_xcr_pos1, denalli_xcr, 0x607A, 0, &offset1.target_position},        // 607A 0 target position
    // {denalli_xcr_pos1, denalli_xcr, 0x60FF, 0, &offset1.target_velocity},        // 60FF 0 target velocity
    // {denalli_xcr_pos1, denalli_xcr, 0x6071, 0, &offset1.target_torque},          // 6071 0 target torque

    // --------------------- denalli_xcr_pos2 ------------------------ 
    
    {denalli_xcr_pos2, denalli_xcr, 0x6041, 0, &offset2.statusword},             // 6041 0 statusword
    {denalli_xcr_pos2, denalli_xcr, 0x6064, 0, &offset2.position_actual_value},  // 6064 0 pos_act_val
    {denalli_xcr_pos2, denalli_xcr, 0x6077, 0, &offset2.torque_actual_value},    // 6077 0 torq_act_val
    {denalli_xcr_pos2, denalli_xcr, 0x2600, 0, &offset2.digital_input_value},    // 2600 0 digital_input_value
    // {denalli_xcr_pos2, denalli_xcr, 0x2600, 0, &offset2.current_actual_value},   // 6078 0 current_actual_value
    // {denalli_xcr_pos2, denalli_xcr, 0x2600, 0, &offset2.homing_offset},          // 607C 0 homing_offset
    // {denalli_xcr_pos2, denalli_xcr, 0x606C, 0, &offset2.velocity_actual_value},  // 606C 0 vel_act_val

    {denalli_xcr_pos2, denalli_xcr, 0x6040, 0, &offset2.controlword},            // 6040 0 control word
    {denalli_xcr_pos2, denalli_xcr, 0x6060, 0, &offset2.modes_of_operation},     // 6060 0 mode_of_operation
    {denalli_xcr_pos2, denalli_xcr, 0x607A, 0, &offset2.target_position},        // 607A 0 target position
    // {denalli_xcr_pos2, denalli_xcr, 0x60FF, 0, &offset2.target_velocity},        // 60FF 0 target velocity
    // {denalli_xcr_pos2, denalli_xcr, 0x6071, 0, &offset2.target_torque},          // 6071 0 target torque

    {}
};

/*****************************************************************************/

void check_domain_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain, &ds);

    if (ds.working_counter != domain_state.working_counter)
    {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain_state.wc_state)
    {
        printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
    {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states)
    {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up)
    {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    printf("Line 279 \n");

    master_state = ms;
}

/*****************************************************************************/

uint16_t update_state(uint16_t status, uint16_t command, int joint_num)
{
    if ( ((status | 65456) ^ 65456) == 0 ){
        printf("Not ready to switch on \n");
        // Not ready to switch on
    }
    else if ( ((status | 65456) ^ 65520 ) == 0 && command != 6){
        printf("Drive %d Switch on Disabled \n", joint_num);
        // Switch on Disabled
        command = 6;
    }
    else if ( ((status | 65424) ^ 65457) == 0 && command != 7){
        printf("Drive %d Ready to Switch on \n", joint_num);
        // Ready to Switch on
        command = 7;
    }
    else if ( ((status | 65424) ^ 65459) == 0 && command != 15){
        printf("Drive %d Switched On \n", joint_num);
        // Switched On
        command = 15;
    }
    else if ( ((status | 65424) ^ 65463) == 0 ){
        // printf(" Operation Enabled \n");
        // Operation Enabled
    }
    // else if ( ((status | 65424) ^ 65431) == 0){
    //     printf("STS_QUICK_STOP_ACTIVE  \n");
    // }
    else if ( ((status | 65456) ^ 65471) == 0){
        // Fault Reaction Active
    }
    else if ( ((status | 65456) ^ 65464) == 0 && command != 128){
        // Fault
        command = 128;
    }
    else{
        // printf("Line 430 status: %d, command : %d\n", status, command);
    }

    return command;
}

/*****************************************************************************/

void cyclic_task()
{
    // receive process data

    /** Fetches received frames from the hardware and processes the datagrams.
     *
     * Queries the network device for received frames by calling the interrupt
     * service routine. Extracts received datagrams and dispatches the results to
     * the datagram objects in the queue. Received datagrams, and the ones that
     * timed out, will be marked, and dequeued.
     *
     * Has to be called cyclically by the realtime application after
     * ecrt_master_activate() has returned.
     */

    ecrt_master_receive(master);

    /** Determines the states of the domain's datagrams.
     *
     * Evaluates the working counters of the received datagrams and outputs
     * statistics, if necessary. This must be called after ecrt_master_receive()
     * is expected to receive the domain datagrams in order to make
     * ecrt_domain_state() return the result of the last process data exchange.
     */

    // printf("line 331 \n");

    ecrt_domain_process(domain);

    // // check process data state
    check_domain_state();

    static uint16_t command = 0;
    static uint16_t command1 = 0;
    static uint16_t command2 = 0;

    uint16_t status, status1, status2;

    uint32_t dio0, dio1, dio2;
    
    int act_pos, act_pos1, act_pos2;
    int act_torq, act_torq1, act_torq2;

    status = EC_READ_U16(domain_pd + offset.statusword);
    status1 = EC_READ_U16(domain_pd + offset1.statusword);
    status2 = EC_READ_U16(domain_pd + offset2.statusword);

    act_pos = EC_READ_S32(domain_pd + offset.position_actual_value);
    act_pos1 = EC_READ_S32(domain_pd + offset1.position_actual_value);
    act_pos2 = EC_READ_S32(domain_pd + offset2.position_actual_value);

    act_torq = EC_READ_S16(domain_pd + offset.torque_actual_value);
    act_torq1 = EC_READ_S16(domain_pd + offset1.torque_actual_value);
    act_torq2 = EC_READ_S16(domain_pd + offset2.torque_actual_value);

    dio0 = EC_READ_U32(domain_pd + offset.digital_input_value);
    dio1 = EC_READ_U32(domain_pd + offset1.digital_input_value);
    dio2 = EC_READ_U32(domain_pd + offset2.digital_input_value);

    data_ptr[8] = dio0;
    data_ptr[9] = dio1;
    data_ptr[10] = dio2;

    data_ptr[11] = act_pos;
    data_ptr[14] = act_pos1;
    data_ptr[17] = act_pos2;

    data_ptr[12] = update_torque(act_torq, 0);
    data_ptr[15] = update_torque(act_torq, 1);
    data_ptr[18] = update_torque(act_torq, 2);

    // printf("dio drive 1 : %ld \n", dio0);
    // printf("dio drive 2 : %ld \n", dio1);
    // printf("dio drive 3 : %ld \n", dio2);

    // ************ update status and command for 1st Drive ************

    // printf(" status : %d, pos_value: %d \n", status, act_pos);

    command = update_state(status, command, 0);
    EC_WRITE_U16(domain_pd + offset.controlword, command);

    // // ************ update status and command for 2nd Drive ************

    // printf(" status1 : %d, pos_value: %d \n", status1, act_pos1);

    command1 = update_state(status1, command1, 1);
    EC_WRITE_U16(domain_pd + offset1.controlword, command1);
    

    // // ************ update status and command for 3rd Drive ************

    // printf(" status2 : %d, pos_value: %d \n", status2, act_pos2);

    command2 = update_state(status2, command2, 2);
    EC_WRITE_U16(domain_pd + offset2.controlword, command2);

    EC_WRITE_S8(domain_pd + offset.modes_of_operation, 8);
    EC_WRITE_S8(domain_pd + offset1.modes_of_operation, 8);
    EC_WRITE_S8(domain_pd + offset2.modes_of_operation, 8);
    

    if (initialized_eth == false)
    {

        

        if ( ((status | 65424) ^ 65463) == 0  && ((status1 | 65424) ^ 65463) == 0 && ((status2 | 65424) ^ 65463) == 0)
        {
            initialized_eth = true;
        }

        data_ptr[13] = act_pos;
        data_ptr[16] = act_pos1;
        data_ptr[19] = act_pos2;

    }
    else{

        // printf("joint pos 1 : %d, joint pos 2 : %d, joint pos 3 : %d, \n", act_pos, act_pos1, act_pos2);
        // printf("joint torq 1 : %d, joint torq 2 : %d, joint torq 3 : %d, \n", act_torq, act_torq1, act_torq2);

        // printf("data joint1 : %d \n", data_ptr[13]);
        // printf("data joint2 : %d \n", data_ptr[16]); 
        // printf("data joint3 : %d \n", data_ptr[19]);
        // printf("act_pos : %d \n", act_pos);
        // printf("act_pos : %d  \n", act_pos1); 
        // printf("act_pos : %d  \n", act_pos2);

        // EC_WRITE_S8(domain1_pd + offset.modes_of_operation, 1);
        // EC_WRITE_S8(domain1_pd + offset1.modes_of_operation, 1);
        // EC_WRITE_S8(domain1_pd + offset2.modes_of_operation, 1);

        EC_WRITE_S32(domain_pd + offset.target_position, data_ptr[13]);
        EC_WRITE_S32(domain_pd + offset1.target_position, data_ptr[16]);
        EC_WRITE_S32(domain_pd + offset2.target_position, data_ptr[19]);

    }

    ecrt_domain_queue(domain);
    ecrt_master_send(master);
}

int update_torque(int act_torq, int joint_id){

    if (joint_id == 0){

        joint_torq1[0] = joint_torq1[1];
        joint_torq1[1] = joint_torq1[2];
        joint_torq1[2] = joint_torq1[3];
        joint_torq1[3] = joint_torq1[4];
        joint_torq1[4] = joint_torq1[5];
        joint_torq1[5] = joint_torq1[6];
        joint_torq1[6] = joint_torq1[7];
        joint_torq1[7] = joint_torq1[8];
        joint_torq1[8] = joint_torq1[9];
        joint_torq1[9] = act_torq;

        return (int)(joint_torq1[0] + joint_torq1[1] + joint_torq1[2] + joint_torq1[3] + joint_torq1[4] + joint_torq1[5] + joint_torq1[6] + joint_torq1[7] + joint_torq1[8] + joint_torq1[9])/10;
    }

    if (joint_id == 1){

        joint_torq2[0] = joint_torq2[1];
        joint_torq2[1] = joint_torq2[2];
        joint_torq2[2] = joint_torq2[3];
        joint_torq2[3] = joint_torq2[4];
        joint_torq2[4] = joint_torq2[5];
        joint_torq2[5] = joint_torq2[6];
        joint_torq2[6] = joint_torq2[7];
        joint_torq2[7] = joint_torq2[8];
        joint_torq2[8] = joint_torq2[9];
        joint_torq2[9] = act_torq;

        return (int)(joint_torq2[0] + joint_torq2[1] + joint_torq2[2] + joint_torq2[3] + joint_torq2[4] + joint_torq2[5] + joint_torq2[6] + joint_torq2[7] + joint_torq2[8] + joint_torq2[9])/10;
    }
    joint_torq1[10], joint_torq2[10], joint_torq3[10];

    if (joint_id == 2){

        joint_torq3[0] = joint_torq3[1];
        joint_torq3[1] = joint_torq3[2];
        joint_torq3[2] = joint_torq3[3];
        joint_torq3[3] = joint_torq3[4];
        joint_torq3[4] = joint_torq3[5];
        joint_torq3[5] = joint_torq3[6];
        joint_torq3[6] = joint_torq3[7];
        joint_torq3[7] = joint_torq3[8];
        joint_torq3[8] = joint_torq3[9];
        joint_torq3[9] = act_torq;

        return (int)(joint_torq3[0] + joint_torq3[1] + joint_torq3[2] + joint_torq3[3] + joint_torq3[4] + joint_torq3[5] + joint_torq3[6] + joint_torq3[7] + joint_torq3[8] + joint_torq3[9])/10;
    }

    

}

/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/
void pdo_mapping(ec_slave_config_t *sc)
{

    /* Define RxPdo */

    ecrt_slave_config_pdo_assign_clear(sc, 2);

    ecrt_slave_config_pdo_assign_add(sc, 2, 0x1600);
    ecrt_slave_config_pdo_assign_add(sc, 2, 0x1601);
    ecrt_slave_config_pdo_assign_add(sc, 2, 0x1602);

    ecrt_slave_config_pdo_mapping_clear(sc, 0x1600);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1601);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1602);

    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6040, 0, 16); /* 0x6040:0/16bits, control word */ 
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x607A, 0, 32); /* 0x607a:0/32bits, Position set Point */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6060, 0, 8); /* 0x6060:0/8bits, mode_of_operation */

    ecrt_slave_config_pdo_mapping_add(sc, 0x1601, 0x60FF, 0, 32); /* 0x60FF:0/32bits, target_velocity */ 
    ecrt_slave_config_pdo_mapping_add(sc, 0x1601, 0x6071, 0, 16); /* 0x6071:0/16bits, target torque */ 

    /* Define TxPdo */

    ecrt_slave_config_pdo_assign_clear(sc, 3);

    ecrt_slave_config_pdo_assign_add(sc, 3, 0x1A00);
    ecrt_slave_config_pdo_assign_add(sc, 3, 0x1A01);
    ecrt_slave_config_pdo_assign_add(sc, 3, 0x1A02);

    ecrt_slave_config_pdo_mapping_clear(sc, 0x1A00);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1A01);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1A02);

    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6041, 0, 16); /* 0x6041:0/16bits, Statusword */ 
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6064, 0, 32); /* 0x6064:0/32bits, Actual Position */ 
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6077, 0, 16); /* 0x6077:0/16bits, Torque Actual Value */ 
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x2600, 0, 32); /* 0x2600:0/32bits, Digital Input Value */ 

    ecrt_slave_config_pdo_mapping_add(sc, 0x1A01, 0x6078, 0, 16); /* 0x6077:0/16bits, Current actual value */ 
    // ecrt_slave_config_pdo_mapping_add(sc, 0x1A01, 0x607C, 0, 32); /* 0x6077:0/16bits, Homing offset */ 
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A01, 0x606C, 0, 32); /* 0x606C:0/32bits, velocity_actual_value */ 

}

/****************************************************************************/

int main(int argc, char **argv)
{
    ec_slave_config_t *sc, *sc1, *sc2;
    struct timespec wakeup_time;
    int ret = 0;

    // Retrieving Master

    master = ecrt_request_master(0);
    if (!master)
    {
        return -1;
    }

    /** Creates a new process data domain.
     *
     * For process data exchange, at least one process data domain is needed.
     * This method creates a new process data domain and returns a pointer to the
     * new domain object. This object can be used for registering PDOs and
     * exchanging them in cyclic operation.
     *
     * This method allocates memory and should be called in non-realtime context
     * before ecrt_master_activate().
     *
     * \return Pointer to the new domain on success, else NULL.
     */

    domain = ecrt_master_create_domain(master);
    if (!domain)
    {
        return -1;
    }

    if (!(sc = ecrt_master_slave_config(
              master, denalli_xcr_pos0, denalli_xcr)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    pdo_mapping(sc);

    if (!(sc1 = ecrt_master_slave_config(
              master, denalli_xcr_pos1, denalli_xcr)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    pdo_mapping(sc1);


    if (!(sc2 = ecrt_master_slave_config(
              master, denalli_xcr_pos2, denalli_xcr)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    pdo_mapping(sc2);


    /** Registers a bunch of PDO entries for a domain.
     *
     * This method has to be called in non-realtime context before
     * ecrt_master_activate().
     *
     * \see ecrt_slave_config_reg_pdo_entry()
     *
     * \attention The registration array has to be terminated with an empty
     *            structure, or one with the \a index field set to zero!
     * \return 0 on success, else non-zero.
     */

    printf("Domain register before...\n");

    if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs))
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master))
    {
        return -1;
    }

    /** Returns the domain's process data.
     *
     * - In kernel context: If external memory was provided with
     * ecrt_domain_external_memory(), the returned pointer will contain the
     * address of that memory. Otherwise it will point to the internally allocated
     * memory. In the latter case, this method may not be called before
     * ecrt_master_activate().
     *
     * - In userspace context: This method has to be called after
     * ecrt_master_activate() to get the mapped domain process data memory.
     *
     * \return Pointer to the process data memory.
     */

    printf("line 661 \n");

    if (!(domain_pd = ecrt_domain_data(domain)))
    {
        return -1;
    }

    /* Set priority */

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
    }

    /* Lock memory */

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }

    stack_prefault();

    ecrt_master_set_send_interval(master, 1000);

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* start in future */
    wakeup_time.tv_nsec = 0;

    /* shared memory file descriptor */
    int shm_fd; 

    /* the size (in bytes) of shared memory object */
    const int SIZE = sizeof(int[40]);

    /* create the shared memory object */
    shm_fd = shm_open("ethercat_data_exchange", O_CREAT | O_RDWR, 0666);

    /* configure the size of the shared memory object */
    ftruncate(shm_fd, SIZE);

    /* memory map the shared memory object */
    data_ptr = mmap(0, SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

    while (1)
    {
        // printf("line 697 \n");
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                              &wakeup_time, NULL);
        if (ret)
        {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        cyclic_task();

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC)
        {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    return ret;
}


