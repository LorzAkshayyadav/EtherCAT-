#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include "ecrt.h"
#include <iostream>

using namespace std;

#define VENDOR_ID 0x0000029C
#define PRODUCT_CODE 0x5EE1

// Define constants for the single slave
#define SLAVE_POSITION 0
#define TARGET_POSITION_INDEX 0x607A
#define CONTROL_WORD_INDEX 0x6040
#define STATUS_WORD_INDEX 0x6041
#define ACTUAL_POSITION_INDEX 0x6064
#define ACTUAL_VELOCITY_INDEX 0x606C
#define OPERATION_MODE_INDEX 0x6060

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};
static ec_slave_config_t *slave_config = NULL;
static ec_slave_config_state_t slave_config_state = {};

// Process data
uint8_t *domain1_pd = NULL;

// Offsets for PDO entries
unsigned int off_control_word;
unsigned int off_status_word;
unsigned int off_target_position;
unsigned int off_actual_position;
unsigned int off_actual_velocity;
unsigned int off_operation_mode;

const ec_pdo_entry_reg_t domain1_pdo_entries[] = {
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, CONTROL_WORD_INDEX, 0x00, &off_control_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, TARGET_POSITION_INDEX, 0x00, &off_target_position},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, OPERATION_MODE_INDEX, 0x00, &off_operation_mode},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, STATUS_WORD_INDEX, 0x00, &off_status_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, ACTUAL_POSITION_INDEX, 0x00, &off_actual_position},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, ACTUAL_VELOCITY_INDEX, 0x00, &off_actual_velocity},

    {}};

const ec_pdo_entry_info_t slave_pdo_entries[] = {
    {CONTROL_WORD_INDEX, 0x00, 16},    // Control Word
    {TARGET_POSITION_INDEX, 0x00, 32}, // Target Position
    {OPERATION_MODE_INDEX, 0x00, 8},   // Mode of Operation
    {STATUS_WORD_INDEX, 0x00, 16},     // Status Word
    {ACTUAL_POSITION_INDEX, 0x00, 32}, // Actual Position
    {ACTUAL_VELOCITY_INDEX, 0x00, 32}, // Actual Velocity

};

// PDO mapping
const ec_pdo_info_t slave_pdos[] = {
    {0x1600, 3, slave_pdo_entries + 0}, // RxPDO
    {0x1A00, 3, slave_pdo_entries + 3}, // TxPDO
};

// Sync manager configuration
const ec_sync_info_t slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_pdos + 1, EC_WD_DISABLE},
    {0xff}};

// Initialize EtherCAT
int initialize_ethercat()
{
    // Request master
    master = ecrt_request_master(0);
    if (!master)
    {
        cerr << "Failed to request master." << endl;
        return -1;
    }
    cout << "EtherCAT master requested successfully." << endl;
    // Check master state
    ec_master_state_t master_state;
    ecrt_master_state(master, &master_state);
    if (master_state.slaves_responding == 0)
    {
        cerr << "No slaves responding" << endl;
        return -1;
    }
    // Create domain
    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
        cerr << "Failed to create domain." << endl;
        return -1;
    }
    cout << "Domain created successfully." << endl;

    // Get slave configuration
    if (!(slave_config = ecrt_master_slave_config(master, 0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE)))
    {
        cerr << "Failed to get slave configuration." << endl;
        return -1;
    }
    cout << "Slave configuration successful." << endl;

    // Configure PDOs
    if (ecrt_slave_config_pdos(slave_config, EC_END, slave_syncs))
    {
        cerr << "Failed to configure PDOs." << endl;
        return -1;
    }
    cout << "PDO configuration successful." << endl;

    // Register PDO entries
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_pdo_entries))
    {
        cerr << "PDO entry registration failed!" << endl;
        return -1;
    }
    cout << "PDO entries registered successfully." << endl;

    // Activate master
    if (ecrt_master_activate(master))
    {
        cerr << "Failed to activate master." << endl;
        return -1;
    }
    cout << "Master activated successfully." << endl;

    // Check if the domain is valid before accessing data
    if (domain1 == NULL)
    {
        cerr << "Domain is not initialized properly." << endl;
        return -1;
    }
    else
    {
        cout << "Domain is valid, attempting to retrieve data." << endl;
    }

    // Get domain data
    domain1_pd = ecrt_domain_data(domain1);
    if (!domain1_pd)
    {
        cerr << "Failed to get domain data." << endl;
        return -1;
    }
    cout << "Domain data retrieved successfully." << endl;

    return 0;
}

// Move to position
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
void cyclic_task(int target_pos,bool &temp)
{

    // Process EtherCAT master and domain
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // Check the domain state
    check_domain1_state();

    // Read status and actual values
    uint16_t status = EC_READ_U16(domain1_pd + off_status_word);
    int actual_pos = EC_READ_S32(domain1_pd + off_actual_position);
    int actual_vel = EC_READ_S32(domain1_pd + off_actual_velocity);

    // Log status
    cout << "Status: 0x" << hex << status
         << ", Position: " << dec << actual_pos
         << ", Velocity: " << actual_vel << endl;

    // Fault handling
    if (status & (1 << 3))
    { // Fault detected
        cerr << "Fault detected in the drive!" << endl;
        EC_WRITE_U16(domain1_pd + off_control_word, 0x0080); // Fault reset
    }

    else if (!(status & (1 << 2)))
    {
        uint16_t control_word = 0x0000;

        if ((status & 0x006F) == 0x0021)
        {
            control_word |= (1 << 1); // Enable Voltage
        }
        else if ((status & 0x006F) == 0x0023)
        {
            control_word |= (1 << 1) | (1 << 0); // Switch On + Enable Voltage
        }
        else if ((status & 0x006F) == 0x0027)
        {
            control_word |= (1 << 1) | (1 << 0) | (1 << 3); // Enable Operation
        }

       
        EC_WRITE_U16(domain1_pd + off_control_word, control_word);
        EC_WRITE_S8(domain1_pd + off_operation_mode, 8); // 8 = Cyclic Synchronous Position mode
        EC_WRITE_S32(domain1_pd + off_target_position, target_pos);
    }

    if (status & (1 << 10))
    { 
        cout << "Target position reached!" << endl;
        temp=true;

        return;
    }

    // Send EtherCAT process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    usleep(10000);

}

int main()
{
    if (initialize_ethercat() != 0)
    {
        cerr << "Failed to initialize EtherCAT." << endl;
        return 1;
    }

    cout << "EtherCAT initialized successfully with " << master_state.slaves_responding << " slave(s)." << endl;

    int target_pos;
    cout << "what position you want to set in slave device" << endl;

    cin >> target_pos;

    ec_slave_config_state_t slave_state;
    ecrt_slave_config_state(slave_config, &slave_state);

    if (slave_state.operational != EC_AL_STATE_OP)
    {
        cerr << "Error: Slave is not in OPERATIONAL state." << endl;

        return 0;
    }
    bool temp=false;
    while (!temp)
    {
       
        cyclic_task(target_pos,temp);
        
    }

    EC_WRITE_U16(domain1_pd + off_control_word, 0x0006); // Disable Operation
    cout << "Drive moved to target position and set to safe state." << endl;

    if (master)
        ecrt_release_master(master);

    return 0;
}
