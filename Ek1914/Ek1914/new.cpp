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

#define VENDOR_ID 0x00000002
#define PRODUCT_CODE 0x077a2c52


// Define constants for the single slave
#define SLAVE_POSITION 0
#define OUTPUT0_INDEX 0x7010

#define INPUT0_INDEX 0x6010



// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

// Process data
uint8_t *domain1_pd = NULL;


// Offsets for PDO entries (byte offsets in process image)
unsigned int off_outputs;
unsigned int off_inputs;

const ec_pdo_entry_reg_t domain1_pdo_entries[] = {
    // Register the whole byte that contains OUT0..OUT3
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, OUTPUT0_INDEX, 0x01, &off_outputs},

    // Register the whole byte that contains IN0..IN2
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, INPUT0_INDEX, 0x01, &off_inputs},

    {}
};

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x7000, 0x01, 8}, /* FSoE Master CMD */
    {0x7001, 0x01, 1}, /* OutputChannel1 */
    {0x7001, 0x02, 1}, /* OutputChannel2 */
    {0x0000, 0x00, 6}, /* Gap */
    {0x7000, 0x03, 16}, /* FSoE Master ConnID */
    {0x7000, 0x02, 16}, /* FSoE Master CRC_0 */
    {0x7010, 0x01, 1}, /* Output 0 */
    {0x7010, 0x02, 1}, /* Output 1 */
    {0x7010, 0x03, 1}, /* Output 2 */
    {0x7010, 0x04, 1}, /* Output 3 */
    {0x7010, 0x05, 1}, /* Safety Linked Output 0 */
    {0x7010, 0x06, 1}, /* Safety Linked Output 1 */
    {0x0000, 0x00, 10}, /* Gap */
    {0x6000, 0x01, 8}, /* FSoE Slave CMD */
    {0x6001, 0x01, 1}, /* InputChannel1 */
    {0x6001, 0x02, 1}, /* InputChannel2 */
    {0x0000, 0x00, 6}, /* Gap */
    {0x6000, 0x03, 16}, /* FSoE Slave ConnID */
    {0x6000, 0x02, 16}, /* FSoE Slave CRC_0 */
    {0x6010, 0x01, 1}, /* Input 0 */
    {0x6010, 0x02, 1}, /* Input 1 */
    {0x6010, 0x03, 1}, /* Input 2 */
    {0x6010, 0x04, 1}, /* Input 3 */
    {0x0000, 0x00, 12}, /* Gap */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 6, slave_0_pdo_entries + 0}, /* FSOE RxPDO-Map */
    {0x1601, 7, slave_0_pdo_entries + 6}, /* DIO RxPDO-Map Outputs */
    {0x1a00, 6, slave_0_pdo_entries + 13}, /* FSOE TxPDO-Map */
    {0x1a01, 5, slave_0_pdo_entries + 19}, /* DIO TxPDO-Map Inputs */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_0_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 2, slave_0_pdos + 2, EC_WD_DISABLE},
    {0xff}
};


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

void cyclic_task( bool &temp)
{

    // Process EtherCAT master and domain
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // Check the domain state
    check_domain1_state();
    uint8_t *outByte = (uint8_t *)(domain1_pd + off_outputs);
    uint8_t *inByte  = (uint8_t *)(domain1_pd + off_inputs);
    uint16_t input0_status = EC_READ_BIT(inByte, 0);
    int input1_status = EC_READ_BIT(inByte, 1);
    int output0_status = EC_READ_BIT(outByte, 0);
    int output1_status = EC_READ_BIT(outByte, 1);

    cout << "INPUT0: " << dec << input0_status
         << ", INPUT1: " << dec << input1_status
         << ", OUTPUT0: " << output0_status 
         << ", OUTPUT1: "<<output1_status<<"\n";
    if(!(input1_status && output0_status))
    {
        EC_WRITE_BIT(outByte,0,1);
    }
    
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    usleep(10000);
    return;
}

int main()
{   
    ec_slave_config_t *sc;
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
    if (!(sc = ecrt_master_slave_config(master, 0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE)))
    {
        cerr << "Failed to get slave configuration." << endl;
        return -1;
    }
    cout << "Slave configuration successful." << endl;

    if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs))
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

    cout << "EtherCAT initialized successfully with " << master_state.slaves_responding << " slave(s)." << endl;

  
    bool temp = false;
    while (!temp)
    {
       cyclic_task(temp);
    }

    if (master)
        ecrt_release_master(master);

    return 0;
}
