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

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_domain_t *domain2 = NULL;
static ec_domain_state_t domain2_state = {};

// process data
static uint8_t *domain1_pd = NULL;
static uint8_t *domain2_pd = NULL;

/* Master 0, Slave 0, "DEN-XCR-E-SI"
 * Vendor ID:       0x0000029c
 * Product code:    0x03831001
 * Revision number: 0x00050008
 */

#define denalli_xcr 0x0000029c, 0x03831002 // Vendor Id, Product Code
#define inhouse_slave 0x0000007b, 0x00009252 // Vendor Id, Product Code

struct joint_pdos
{
    unsigned int statusword;
    unsigned int position_actual_value;
    unsigned int torque_actual_value;
    unsigned int digital_input_value;
    unsigned int current_actual_value;
    unsigned int homing_offset;
    unsigned int velocity_actual_value;
    unsigned int controlword;
    unsigned int modes_of_operation;
    unsigned int target_position;
    unsigned int target_velocity;
    unsigned int target_torque;
} d1_offset[2], d2_offset[1];
