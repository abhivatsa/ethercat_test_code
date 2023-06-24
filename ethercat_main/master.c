#include "./master.h"

/*****************************************************************************/

void check_domain_state(void)
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

uint16_t update_state(uint16_t status, uint16_t command, int joint_num)
{
    if ( ((status | 65456) ^ 65456) == 0 ){
        printf("Not ready to switch on \n");
        // Not ready to switch on
    }
    else if ( ((status | 65456) ^ 65520 ) == 0){
        printf("Drive %d Switch on Disabled \n", joint_num);
        // Switch on Disabled
        command = 6;
    }
    else if ( ((status | 65424) ^ 65457) == 0){
        printf("Drive %d Ready to Switch on \n", joint_num);
        // Ready to Switch on
        command = 7;
    }
    else if ( ((status | 65424) ^ 65459) == 0){
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

    // printf("line 133 \n");

    ecrt_domain_process(domain1);
    ecrt_domain_process(domain2);

    // // check process data state
    check_domain_state();

    // printf("domain1 %lu \n", ecrt_domain_size(domain1));
    // printf("%lu \n", ecrt_domain_size(domain2));



    // ************ For Domain 1 **************

    int domain1_slave_cnt = 2;
    static uint16_t domain1_command[1] = {0, 0};

    uint16_t domain1_status[domain1_slave_cnt];

    uint32_t domain1_dio[domain1_slave_cnt];
    
    int domain1_act_pos[domain1_slave_cnt];
    int domain1_act_torq[domain1_slave_cnt];

    for (int jnt_ctr = 0; jnt_ctr < domain1_slave_cnt; jnt_ctr++){
        // printf("1st joint \n");
        domain1_status[jnt_ctr] = EC_READ_U16(domain1_pd + d1_offset[jnt_ctr].statusword);
        domain1_act_pos[jnt_ctr] = EC_READ_S32(domain1_pd + d1_offset[jnt_ctr].position_actual_value);
        domain1_act_torq[jnt_ctr] = EC_READ_S16(domain1_pd + d1_offset[jnt_ctr].torque_actual_value);
        domain1_dio[jnt_ctr] = EC_READ_U32(domain1_pd + d1_offset[jnt_ctr].digital_input_value);

        domain1_command[jnt_ctr] = update_state(domain1_status[jnt_ctr], domain1_command[jnt_ctr], 0);

        EC_WRITE_U16(domain1_pd + d1_offset[jnt_ctr].controlword, domain1_command[jnt_ctr]);
        EC_WRITE_S8(domain1_pd + d1_offset[jnt_ctr].modes_of_operation, 8);
        EC_WRITE_S32(domain1_pd + d1_offset[jnt_ctr].target_position, 200000);
    }

    // *************** For Domain 2 ***************
    int domain2_slave_cnt = 1;
    static uint16_t domain2_command[1] = {0};

    uint16_t domain2_status[domain2_slave_cnt];

    uint32_t domain2_dio[domain2_slave_cnt];
    
    int domain2_act_pos[domain2_slave_cnt];
    int domain2_act_torq[domain2_slave_cnt];

    for (int jnt_ctr = 0; jnt_ctr < 1; jnt_ctr++){
        // printf("3rd joint \n");
        domain2_status[jnt_ctr] = EC_READ_U16(domain2_pd + d2_offset[jnt_ctr].statusword);
        domain2_act_pos[jnt_ctr] = EC_READ_S32(domain2_pd + d2_offset[jnt_ctr].position_actual_value);
        domain2_act_torq[jnt_ctr] = EC_READ_S16(domain2_pd + d2_offset[jnt_ctr].torque_actual_value);
        domain2_dio[jnt_ctr] = EC_READ_U32(domain2_pd + d2_offset[jnt_ctr].digital_input_value);

        domain2_command[jnt_ctr] = update_state(domain2_status[jnt_ctr], domain2_command[jnt_ctr], 0);

        EC_WRITE_U16(domain2_pd + d2_offset[jnt_ctr].controlword, domain2_command[jnt_ctr]);
        EC_WRITE_S8(domain2_pd + d2_offset[jnt_ctr].modes_of_operation, 8);
        EC_WRITE_S32(domain2_pd + d2_offset[jnt_ctr].target_position, 200000);
    }


    ecrt_domain_queue(domain1);
    ecrt_domain_queue(domain2);
    ecrt_master_send(master);
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
    ecrt_slave_config_sync_manager(sc, 2, EC_DIR_OUTPUT, EC_WD_ENABLE);

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
    ecrt_slave_config_pdo_mapping_add(sc, 0x1601, 0x607C, 0, 32); /* 0x607C:0/32bits, homing offset */ 

    // ecrt_slave_config_reg_pdo_entry_pos(sc, 2, 0, 0, domain1, 0);

    /* Define TxPdo */

    ecrt_slave_config_sync_manager(sc, 3, EC_DIR_INPUT, EC_WD_ENABLE);

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
    ecrt_slave_config_pdo_mapping_add(sc, 0x1A01, 0x606C, 0, 32); /* 0x606C:0/32bits, velocity_actual_value */ 
    

}

/****************************************************************************/

int main(int argc, char **argv)
{
    struct timespec wakeup_time;
    int ret = 0;

    int local_pos = 0;

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

    for (int jnt_ctr = 0; jnt_ctr < 2; jnt_ctr++)
    {
        ec_slave_config_t *sc;

        if (!(sc = ecrt_master_slave_config(
                master, 0, jnt_ctr, denalli_xcr)))
        {
            fprintf(stderr, "Failed to get slave configuration.\n");
            return -1;
        }

        // ec_sdo_request_t *sdo_req;

        // if (!(sdo_req = ecrt_slave_config_create_sdo_request(sc, 0x6064, 0, 32)))
        // {
        //     fprintf(stderr, "Failed to create SDO request.\n");
        //     return -1;
        // }

        // ecrt_sdo_request_timeout(sdo_req, 10000);

        // int value_chk = EC_READ_S32(ecrt_sdo_request_data(sdo_req));

        // printf("line 277 : %d \n", value_chk);

        pdo_mapping(sc);     

        ec_pdo_entry_reg_t domain_regs[] = {
    
            // --------------------- denalli_xcr_pos0 ------------------------ 
            {0, jnt_ctr, denalli_xcr, 0x6041, 0, &d1_offset[jnt_ctr-local_pos].statusword},             // 6041 0 statusword
            {0, jnt_ctr, denalli_xcr, 0x6064, 0, &d1_offset[jnt_ctr-local_pos].position_actual_value},  // 6064 0 pos_act_val
            {0, jnt_ctr, denalli_xcr, 0x6077, 0, &d1_offset[jnt_ctr-local_pos].torque_actual_value},    // 6077 0 torq_act_val
            {0, jnt_ctr, denalli_xcr, 0x2600, 0, &d1_offset[jnt_ctr-local_pos].digital_input_value},    // 2600 0 digital_input_value
            {0, jnt_ctr, denalli_xcr, 0x6078, 0, &d1_offset[jnt_ctr-local_pos].current_actual_value},   // 6078 0 current_actual_value
            {0, jnt_ctr, denalli_xcr, 0x606C, 0, &d1_offset[jnt_ctr-local_pos].velocity_actual_value},  // 606C 0 vel_act_val
            {0, jnt_ctr, denalli_xcr, 0x607C, 0, &d1_offset[jnt_ctr-local_pos].homing_offset},
            {0, jnt_ctr, denalli_xcr, 0x6040, 0, &d1_offset[jnt_ctr-local_pos].controlword},            // 6040 0 control word
            {0, jnt_ctr, denalli_xcr, 0x6060, 0, &d1_offset[jnt_ctr-local_pos].modes_of_operation},     // 6060 0 mode_of_operation
            {0, jnt_ctr, denalli_xcr, 0x607A, 0, &d1_offset[jnt_ctr-local_pos].target_position},        // 607A 0 target position
            {0, jnt_ctr, denalli_xcr, 0x60FF, 0, &d1_offset[jnt_ctr-local_pos].target_velocity},        // 60FF 0 target velocity
            {0, jnt_ctr, denalli_xcr, 0x6071, 0, &d1_offset[jnt_ctr-local_pos].target_torque},          // 6071 0 target torque
            {}

        };

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

        if (ecrt_domain_reg_pdo_entry_list(domain1, domain_regs))
        {
            fprintf(stderr, "PDO entry registration failed!\n");
            return -1;
        }
        
    }

    domain2 = ecrt_master_create_domain(master);
    if (!domain2)
    {
        return -1;
    }

    local_pos = 2;

    for (int jnt_ctr = 2; jnt_ctr < 3; jnt_ctr++)
    {
        ec_slave_config_t *sc;

        if (!(sc = ecrt_master_slave_config(
                master, 0, jnt_ctr, denalli_xcr)))
        {
            fprintf(stderr, "Failed to get slave configuration.\n");
            return -1;
        }

        // ec_sdo_request_t *sdo_req;

        // if (!(sdo_req = ecrt_slave_config_create_sdo_request(sc, 0x6064, 0, 32)))
        // {
        //     fprintf(stderr, "Failed to create SDO request.\n");
        //     return -1;
        // }

        // ecrt_sdo_request_timeout(sdo_req, 10000);

        // int value_chk = EC_READ_S32(ecrt_sdo_request_data(sdo_req));

        // printf("line 277 : %d \n", value_chk);

        pdo_mapping(sc);     

        ec_pdo_entry_reg_t domain_regs[] = {
    
            // --------------------- denalli_xcr_pos0 ------------------------ 
            {0, jnt_ctr, denalli_xcr, 0x6041, 0, &d2_offset[jnt_ctr-local_pos].statusword},             // 6041 0 statusword
            {0, jnt_ctr, denalli_xcr, 0x6064, 0, &d2_offset[jnt_ctr-local_pos].position_actual_value},  // 6064 0 pos_act_val
            {0, jnt_ctr, denalli_xcr, 0x6077, 0, &d2_offset[jnt_ctr-local_pos].torque_actual_value},    // 6077 0 torq_act_val
            {0, jnt_ctr, denalli_xcr, 0x2600, 0, &d2_offset[jnt_ctr-local_pos].digital_input_value},    // 2600 0 digital_input_value
            {0, jnt_ctr, denalli_xcr, 0x6078, 0, &d2_offset[jnt_ctr-local_pos].current_actual_value},   // 6078 0 current_actual_value
            {0, jnt_ctr, denalli_xcr, 0x606C, 0, &d2_offset[jnt_ctr-local_pos].velocity_actual_value},  // 606C 0 vel_act_val
            {0, jnt_ctr, denalli_xcr, 0x607C, 0, &d2_offset[jnt_ctr-local_pos].homing_offset},
            {0, jnt_ctr, denalli_xcr, 0x6040, 0, &d2_offset[jnt_ctr-local_pos].controlword},            // 6040 0 control word
            {0, jnt_ctr, denalli_xcr, 0x6060, 0, &d2_offset[jnt_ctr-local_pos].modes_of_operation},     // 6060 0 mode_of_operation
            {0, jnt_ctr, denalli_xcr, 0x607A, 0, &d2_offset[jnt_ctr-local_pos].target_position},        // 607A 0 target position
            {0, jnt_ctr, denalli_xcr, 0x60FF, 0, &d2_offset[jnt_ctr-local_pos].target_velocity},        // 60FF 0 target velocity
            {0, jnt_ctr, denalli_xcr, 0x6071, 0, &d2_offset[jnt_ctr-local_pos].target_torque},          // 6071 0 target torque
            {}

        };

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

        if (ecrt_domain_reg_pdo_entry_list(domain2, domain_regs))
        {
            fprintf(stderr, "PDO entry registration failed!\n");
            return -1;
        }
        
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

    if (!(domain1_pd = ecrt_domain_data(domain1)))
    {
        return -1;
    }

    if (!(domain2_pd = ecrt_domain_data(domain2)))
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

    // /* shared memory file descriptor */
    // int shm_fd; 

    // /* the size (in bytes) of shared memory object */
    // const int SIZE = sizeof(int[20]);

    // /* create the shared memory object */
    // shm_fd = shm_open("ethercat_data_exchange", O_CREAT | O_RDWR, 0666);

    // /* configure the size of the shared memory object */
    // ftruncate(shm_fd, SIZE);

    // /* memory map the shared memory object */
    // data_ptr = mmap(0, SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

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