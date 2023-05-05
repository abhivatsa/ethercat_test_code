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

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_ana_in = NULL;
static ec_slave_config_state_t sc_ana_in_state = {};

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

/* Master 0, Slave 0, "Synapticon Circulo 9"
 * Vendor ID:       0x000022d2
 * Product code:    0x00000301
 * Revision number: 0x0c000000
 */

#define synapticon_circulo9 0x000022d2, 0x00000301 // Vendor Id, Product Code

#define synapticon_circulo9_pos0 0, 0 // Alias, Position
// #define denalli_xcr_pos1 0, 1 // Alias, Position
// #define denalli_xcr_pos2 0, 2 // Alias, Position

static struct 
{
    unsigned int statusword;
    unsigned int position_actual_value;
    unsigned int velocity_actual_value;
    unsigned int controlword;
    unsigned int modes_of_operation;
    unsigned int target_position;
    unsigned int target_velocity;
} offset, offset1, offset2;

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

const static ec_pdo_entry_reg_t domain1_regs[] = {
    // --------------------- denalli_xcr_pos0 ------------------------ 
    {synapticon_circulo9_pos0, synapticon_circulo9, 0x6041, 0, &offset.statusword},             // 6041 0 statusword
    {synapticon_circulo9_pos0, synapticon_circulo9, 0x6064, 0, &offset.position_actual_value},  // 6064 0 pos_act_val
    {synapticon_circulo9_pos0, synapticon_circulo9, 0x606C, 0, &offset.velocity_actual_value},  // 606C 0 vel_act_val
    {synapticon_circulo9_pos0, synapticon_circulo9, 0x6040, 0, &offset.controlword},            // 6040 0 control word
    {synapticon_circulo9_pos0, synapticon_circulo9, 0x6060, 0, &offset.modes_of_operation},     // 6060 0 mode_of_operation
    {synapticon_circulo9_pos0, synapticon_circulo9, 0x607A, 0, &offset.target_position},        // 607A 0 target position
    {synapticon_circulo9_pos0, synapticon_circulo9, 0x60FF, 0, &offset.target_velocity},        // 60FF 0 target velocity
    // // --------------------- denalli_xcr_pos1 ------------------------ 
    // {denalli_xcr_pos1, denalli_xcr, 0x6041, 0, &offset1.statusword},            // 6041 0 statusword
    // {denalli_xcr_pos1, denalli_xcr, 0x6064, 0, &offset1.position_actual_value}, // 6064 0 pos_act_val
    // {denalli_xcr_pos1, denalli_xcr, 0x606C, 0, &offset1.velocity_actual_value}, // 606C 0 vel_act_val
    // {denalli_xcr_pos1, denalli_xcr, 0x6040, 0, &offset1.controlword},           // 6040 0 control word
    // {denalli_xcr_pos1, denalli_xcr, 0x6060, 0, &offset1.modes_of_operation},    // 6060 0 mode_of_operation
    // {denalli_xcr_pos1, denalli_xcr, 0x607A, 0, &offset1.target_position},       // 607A 0 target position
    // {denalli_xcr_pos1, denalli_xcr, 0x60FF, 0, &offset1.target_velocity},       // 60FF 0 target velocity
    // // --------------------- denalli_xcr_pos2 ------------------------ 
    // {denalli_xcr_pos2, denalli_xcr, 0x6041, 0, &offset2.statusword},            // 6041 0 statusword
    // {denalli_xcr_pos2, denalli_xcr, 0x6064, 0, &offset2.position_actual_value}, // 6064 0 pos_act_val
    // {denalli_xcr_pos2, denalli_xcr, 0x606C, 0, &offset2.velocity_actual_value}, // 606C 0 vel_act_val
    // {denalli_xcr_pos2, denalli_xcr, 0x6040, 0, &offset2.controlword},           // 6040 0 control word
    // {denalli_xcr_pos2, denalli_xcr, 0x6060, 0, &offset2.modes_of_operation},    // 6060 0 mode_of_operation
    // {denalli_xcr_pos2, denalli_xcr, 0x607A, 0, &offset2.target_position},       // 607A 0 target position
    // {denalli_xcr_pos2, denalli_xcr, 0x60FF, 0, &offset2.target_velocity},       // 60FF 0 target velocity
    {}
};

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Controlword */
    {0x6060, 0x00, 8}, /* Modes of operation */
    {0x6071, 0x00, 16}, /* Target Torque */
    {0x607a, 0x00, 32}, /* Target position */
    {0x60ff, 0x00, 32}, /* Target velocity */
    {0x60b2, 0x00, 16}, /* Torque offset */
    {0x2701, 0x00, 32}, /* Tuning command */
    {0x60fe, 0x01, 32}, /* Physical outputs */
    {0x60fe, 0x02, 32}, /* Bit mask */
    {0x2703, 0x00, 32}, /* User MOSI */
    {0x60b1, 0x00, 32}, /* Velocity offset */
    {0x6041, 0x00, 16}, /* Statusword */
    {0x6061, 0x00, 8}, /* Modes of operation display */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x606c, 0x00, 32}, /* Velocity actual value */
    {0x6077, 0x00, 16}, /* Torque actual value */
    {0x2401, 0x00, 16}, /* Analog input 1 */
    {0x2402, 0x00, 16}, /* Analog input 2 */
    {0x2403, 0x00, 16}, /* Analog input 3 */
    {0x2404, 0x00, 16}, /* Analog input 4 */
    {0x2702, 0x00, 32}, /* Tuning status */
    {0x60fd, 0x00, 32}, /* Digital inputs */
    {0x2704, 0x00, 32}, /* User MISO */
    {0x20f0, 0x00, 32}, /* Timestamp */
    {0x60fc, 0x00, 32}, /* Position demand internal value */
    {0x606b, 0x00, 32}, /* Velocity demand value */
    {0x6074, 0x00, 16}, /* Torque demand */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 7, slave_0_pdo_entries + 0}, /* Receive PDO1 mapping */
    {0x1601, 2, slave_0_pdo_entries + 7}, /* Receive PDO2 mapping */
    {0x1602, 2, slave_0_pdo_entries + 9}, /* Receive PDO3 mapping */
    {0x1a00, 5, slave_0_pdo_entries + 11}, /* Transmit PDO1 mapping */
    {0x1a01, 5, slave_0_pdo_entries + 16}, /* Transmit PDO2 mapping */
    {0x1a02, 1, slave_0_pdo_entries + 21}, /* Transmit PDO3 mapping */
    {0x1a03, 5, slave_0_pdo_entries + 22}, /* Transmit PDO4 mapping */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 3, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 4, slave_0_pdos + 3, EC_WD_DISABLE},
    {0xff}
};



/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
    {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state)
    {
        printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
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

uint16_t update_state(uint16_t status, uint16_t command)
{
    if ( ((status | 65456) ^ 65456) == 0 ){
        printf("Not ready to switch on \n");
        // Not ready to switch on
    }
    else if ( ((status | 65456) ^ 65520 ) == 0 && command != 6){
        printf("Switch on Disabled \n");
        // Switch on Disabled
        command = 6;
    }
    else if ( ((status | 65424) ^ 65457) == 0 && command != 7){
        printf("Ready to Switch on \n");
        // Ready to Switch on
        command = 7;
    }
    else if ( ((status | 65424) ^ 65459) == 0 && command != 15){
        // Switched On
        printf("Switched On \n");
     // ecrt_slave_config_sdo16( sc, 0x1C13, 2, 0x1A01 ); /* list all TxPdo in 0x1800:1-3*/
    // ecrt_slave_config_sdo16( sc, 0x1C13, 3, 0x1A02 ); /* list all TxPdo in 0x1800:1-3*/       command = 15;
    }
    else if ( ((status | 65424) ^ 65463) == 0 ){
        // Operation Enabled
        printf("Operation Enabled \n");
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

    // printf(" Switch on Disabled %d \n", ((status | 65456) ^ 65520 ) == 0);
    // printf(" Ready to Switch on %d \n", ((status | 65424) ^ 65457) == 0);
    // printf(" Switched On %d \n", ((status | 65424) ^ 65459) == 0 );
    // printf(" Operation Enabled %d \n", ((status | 65424) ^ 65463) == 0 );

    // printf("Line 246 status: %d, command : %d\n", status, command);

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

    ecrt_domain_process(domain1);

    // check process data state
    check_domain1_state();

    static uint16_t command = 0;
    static uint16_t command1 = 0;
    static uint16_t command2 = 0;

    uint16_t status, status1, status2;
    
    int act_pos, act_pos1, act_pos2;

    // static uint16_t command = AKD_CMD_ENA_QSTOP; 
    // short act_torq;

    status = EC_READ_U16(domain1_pd + offset.statusword);
    // status1 = EC_READ_U16(domain1_pd + offset1.statusword);
    // status2 = EC_READ_U16(domain1_pd + offset2.statusword);

    act_pos = EC_READ_S32(domain1_pd + offset.position_actual_value);
    // act_pos1 = EC_READ_S32(domain1_pd + offset1.position_actual_value);
    // act_pos2 = EC_READ_S32(domain1_pd + offset2.position_actual_value);


    // ************ update status and command for 1st Drive ************

    // printf(" status : %d, pos_value: %d \n", status, act_pos);

    command = update_state(status, command);

    // printf(" command : %d, \n", command);

    EC_WRITE_U16(domain1_pd + offset.controlword, command);


    // if ( ((status | 65424) ^ 65463) == 0 ){
    //     EC_WRITE_S8(domain1_pd + offset.modes_of_operation, 1);
    //     EC_WRITE_S32(domain1_pd + offset.target_position, 800000);
    // }

    // // ************ update status and command for 2nd Drive ************

    // printf(" status1 : %d, pos_value: %d \n", status1, act_pos1);

    // command1 = update_state(status1, command1);

    // EC_WRITE_U16(domain1_pd + offset1.controlword, command1);

    // if ( ((status1 | 65424) ^ 65463) == 0 ){
    //     EC_WRITE_S8(domain1_pd + offset1.modes_of_operation, 1);
    //     EC_WRITE_S32(domain1_pd + offset1.target_position, 800000);
    // }

    // // ************ update status and command for 3rd Drive ************

    // printf(" status2 : %d, pos_value: %d \n", status2, act_pos2);

    // command2 = update_state(status2, command2);

    // EC_WRITE_U16(domain1_pd + offset2.controlword, command2);

    // if ( ((status2 | 65424) ^ 65463) == 0 ){
    //     EC_WRITE_S8(domain1_pd + offset2.modes_of_operation, 1);
    //     EC_WRITE_S32(domain1_pd + offset2.target_position, 800000);
    // }

    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;
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

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
        return -1;
    }


    if (!(sc = ecrt_master_slave_config(
              master, synapticon_circulo9_pos0, synapticon_circulo9)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    /* Configure Denalli XCR Drive 1 PDO */
    printf("Configuring Synapticon Circulo 9 Drive 1 PDO...\n");

    /* *********************** Clear RxPdo Drive 1 **************************** */

    // /** Add a configuration value for an 8-bit SDO.
    //  *
    //  * This method has to be called in non-realtime context before
    //  * ecrt_master_activate().
    //  *
    //  * \see ecrt_slave_config_sdo().
    //  *
    //  * \retval  0 Success.
    //  * \retval <0 Error code.
    //  */
    // int ecrt_slave_config_sdo8(
    //         ec_slave_config_t *sc, /**< Slave configuration */
    //         uint16_t sdo_index, /**< Index of the SDO to configure. */
    //         uint8_t sdo_subindex, /**< Subindex of the SDO to configure. */
    //         uint8_t value /**< Value to set. */
    //         );


    // ec_pdo_info_t slave_0_rec_pdos[] = {
    //     {0x1600, 4, slave_0_rec_pdo_channel1},
    //     {0x1601, 2, slave_0_rec_pdo_channel2},
    //     {0x1602, 2, slave_0_rec_pdo_channel3},
    // };

    // /** Add a configuration value for a 32-bit SDO.
    //  * 
    //  * This method has to be called in non-realtime context before
    //  * ecrt_master_activate().
    //  *
    //  * \see ecrt_slave_config_sdo().
    //  *
    //  * \retval  0 Success.
    //  * \retval <0 Error code.
    //  */
    // int ecrt_slave_config_sdo32(
    //         ec_slave_config_t *sc, /**< Slave configuration */
    //         uint16_t sdo_index, /**< Index of the SDO to configure. */
    //         uint8_t sdo_subindex, /**< Subindex of the SDO to configure. */
    //         uint32_t value /**< Value to set. */
    //         );
    

    ecrt_slave_config_sdo8( sc, 0x1C12, 0, 0 ); /* clear sm pdo 0x1C00 */
    ecrt_slave_config_sdo8( sc, 0x1600, 0, 0 ); /* clear RxPdo 0x1600 */
    ecrt_slave_config_sdo8( sc, 0x1601, 0, 0 ); /* clear RxPdo 0x1601 */
    ecrt_slave_config_sdo8( sc, 0x1602, 0, 0 ); /* clear RxPdo 0x1602 */

    printf("Configure to zeros");
    // sleep(10);

    /* Define RxPdo */

    ecrt_slave_config_sdo32( sc, 0x1600, 1, 0x60400010 ); /* 0x6040:0/16bits, control word */ 
    ecrt_slave_config_sdo32( sc, 0x1600, 2, 0x607a0020 ); /* 0x607a:0/32bits, Position set Point */ 
    ecrt_slave_config_sdo32( sc, 0x1600, 3, 0x60ff0020 ); /* 0x60ff:0/32bits, Velocity set point */ 
    ecrt_slave_config_sdo32( sc, 0x1600, 4, 0x60600008 ); /* 0x6060:0/8bits, mode_of_operation */ 
    ecrt_slave_config_sdo8( sc, 0x1600, 0, 4); /* set number of PDO entries for 0x1600 */

    ecrt_slave_config_sdo16( sc, 0x1C12, 1, 0x1600 ); /* list all RxPdo in 0x1C00:1-2*/
    // ecrt_slave_config_sdo16( sc, 0x1C12, 2, 0x1601 ); /* list all RxPdo in 0x1C00:1-2*/
    // ecrt_slave_config_sdo16( sc, 0x1C12, 3, 0x1602 ); /* list all RxPdo in 0x1C00:1-2*/

    ecrt_slave_config_sdo8( sc, 0x1C12, 0, 1 ); /* set number of RxPDO */

    ecrt_slave_config_pdo_mapping_clear(sc, 0x1600);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1601);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1602);

    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6040, 0, 16); /* 0x6040:0/16bits, control word */ 
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x607a, 0, 32); /* 0x607a:0/32bits, Position set Point */
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x60ff, 0, 32); /* 0x60ff:0/32bits, Velocity set point */  
    ecrt_slave_config_pdo_mapping_add(sc, 0x1600, 0x6060, 0, 8); /* 0x6060:0/8bits, mode_of_operation */ 

    /* Define TxPdo */

    ecrt_slave_config_pdo_mapping_clear(sc, 0x1A00);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1A01);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1A02);
    ecrt_slave_config_pdo_mapping_clear(sc, 0x1A03);

    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6041, 0, 16); /* 0x6041:0/16bits, Statusword */ 
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6064, 0, 32); /* 0x6064:0/32bits, Actual Position */ 
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x606C, 0, 32); /* 0x606C:0/32bits, Actual Velocity */ 
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A00, 0x6061, 0, 8); /* 0x6061:0/8bits, Operation Mode display */

    printf("Configuring PDOs...\n");
    // if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs))
    // {
    //     fprintf(stderr, "Failed to configure PDOs.\n");
    //     return -1;
    // }

    // if (!(sc = ecrt_master_slave_config(
    //           master, denalli_xcr_pos1, denalli_xcr)))
    // {
    //     fprintf(stderr, "Failed to get slave configuration.\n");
    //     return -1;
    // }

    // if (ecrt_slave_config_pdos(sc, EC_END, slave_1_pdo_syncs))
    // {
    //     fprintf(stderr, "Failed to configure PDOs.\n");
    //     return -1;
    // }

    // if (!(sc = ecrt_master_slave_config(
    //           master, denalli_xcr_pos2, denalli_xcr)))
    // {
    //     fprintf(stderr, "Failed to get slave configuration.\n");
    //     return -1;
    // }

    // if (ecrt_slave_config_pdos(sc, EC_END, slave_2_pdo_syncs))
    // {
    //     fprintf(stderr, "Failed to configure PDOs.\n");
    //     return -1;
    // }

    // sleep(100);


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

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs))
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    // sleep(10);

    printf("Domain register after...\n");

    printf("Activating master...\n");
    if (ecrt_master_activate(master))
    {
        return -1;
    }

    // sleep(10);


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

    if (!(domain1_pd = ecrt_domain_data(domain1)))
    {
        return -1;
    }

    // sleep(10);


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

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* start in future */
    wakeup_time.tv_nsec = 0;

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


