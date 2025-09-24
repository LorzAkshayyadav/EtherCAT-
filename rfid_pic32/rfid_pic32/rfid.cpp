#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <iostream>
#include "ecrt.h"

using namespace std;

// EtherCAT Slave Definitions (adjust vendor/product if needed)
#define VENDOR_ID       0x0000007b
#define PRODUCT_CODE    0x00009252

#define SLAVE_POSITION  0   // position on bus (usually 0)
#define CONTROL_WORD    0x7000
#define STATUS_WORD     0x6000
#define HEADER          0x6000
#define INSTRUMENT_TYPE 0x6000
#define QC_STATUS       0x6000
#define RFID_UID_START  0x6000  // UID sub-indices start from SubIndex 5 (as in your original)

// EtherCAT global objects
static ec_master_t *master = NULL;
static ec_domain_t *domain1 = NULL;
static ec_master_state_t master_state = {};
static ec_domain_state_t domain1_state = {};
uint8_t *domain1_pd = NULL;

// Offsets for PDO entries (populated by ecrt_domain_reg_pdo_entry_list)
unsigned int off_control_word;
unsigned int off_status_word;
unsigned int off_header;
unsigned int off_instrument_type;
unsigned int off_rfid_uid[6];
unsigned int off_qc_status;
uint8_t cmd = 0;

// -------------------------------
// PDO registration array
// Note: each entry is {domain, slave position, vendor, product, index, subindex, &offset}
// Make sure index/subindex match the slave's object dictionary
// -------------------------------
const ec_pdo_entry_reg_t domain1_pdo_entries[] = {
    /* outputs from master to slave */
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, CONTROL_WORD,    0x01, &off_control_word},       // 0x7000.01 -> control_word
    /* inputs from slave to master */
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, STATUS_WORD,     0x01, &off_status_word},        // 0x6000.01 -> status_word
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, HEADER,          0x02, &off_header},             // 0x6000.02 -> header (subindex 2)
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, INSTRUMENT_TYPE, 0x03, &off_instrument_type},    // 0x6000.03 -> instrument_type
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, QC_STATUS,       0x17, &off_qc_status},          // 0x6000.17 -> qc_status

    /* RFID UID bytes - adjust subindices to match slave's dictionary.
       Here we register 6 subindices for the UID (example: subindices 5..10 or 5..0x10 as in your original).
       Please verify the exact subindex numbers on your PIC32 slave SII (or ESI) file. */
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, RFID_UID_START,  0x05, &off_rfid_uid[0]}, // 0x6000.05 -> UID byte 0 ->instrument version
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, RFID_UID_START,  0x06, &off_rfid_uid[1]}, // 0x6000.06 -> UID byte 1  ->mfg country code
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, RFID_UID_START,  0x07, &off_rfid_uid[2]}, // 0x6000.07 -> UID byte 2  ->Mfg_Plant_code
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, RFID_UID_START,  0x08, &off_rfid_uid[3]}, // 0x6000.08 -> UID byte 3   Mfg_Date
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, RFID_UID_START,  0x09, &off_rfid_uid[4]}, // 0x6000.09 -> UID byte 4   Mfg_Month
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, RFID_UID_START,  0x16, &off_rfid_uid[5]}, // 0x6000.10 -> UID byte 5 Mfg_year
    {}
};

// -------------------------------
// PDO / Sync info for the slave
// The entries below must reflect your slave PDO configuration. You had a large list.
// I kept your pdo entry layout and grouping; confirm these match the slave's ESI/SII.
// -------------------------------
ec_pdo_entry_info_t slave_pdo_entries[] = {
    /* 0x7000 - outputs */
    {0x7000, 0x01, 8},
    {0x7000, 0x02, 8},
    {0x7000, 0x03, 8},
    {0x7000, 0x04, 8},
    {0x7000, 0x05, 16},
    {0x7000, 0x06, 8},
    {0x7000, 0x07, 8},
    {0x7000, 0x08, 8},
    {0x7000, 0x09, 8},
    {0x7000, 0x10, 8},
    {0x7000, 0x11, 8},
    {0x7000, 0x12, 16},
    {0x7000, 0x13, 8},
    {0x7000, 0x14, 8},
    {0x7000, 0x15, 8},
    {0x7000, 0x16, 8},
    {0x7000, 0x17, 8},

    /* 0x6000 - inputs */
    {0x6000, 0x01, 8},
    {0x6000, 0x02, 8},
    {0x6000, 0x03, 8},
    {0x6000, 0x04, 8},
    {0x6000, 0x05, 16},
    {0x6000, 0x06, 8},
    {0x6000, 0x07, 8},
    {0x6000, 0x08, 8},
    {0x6000, 0x09, 8},
    {0x6000, 0x10, 8},
    {0x6000, 0x11, 8},
    {0x6000, 0x12, 16},
    {0x6000, 0x13, 8},
    {0x6000, 0x14, 8},
    {0x6000, 0x15, 8},
    {0x6000, 0x16, 8},
    {0x6000, 0x17, 8},
};

ec_pdo_info_t slave_pdos[] = {
    {0x1600, 17, slave_pdo_entries + 0},   // outputs (1600) - 17 entries
    {0x1a00, 17, slave_pdo_entries + 17},  // inputs  (1a00) - 17 entries
};

ec_sync_info_t slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, slave_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// Helper: prints domain state changes
void check_domain1_state(void)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) {
        printf("Domain1: State %u.\n", ds.wc_state);
    }
    domain1_state = ds;
}

// simple state machine example for control byte
int update_status(int c){
    if (c == 0)    c = 65;
    else if (c==65) c = 66;
    else if (c==66) c = 67;
    return c;
}
int hee=0;
void cyclic_task() {
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    /* domain1_pd becomes valid only after ecrt_master_activate() and domain registration */
    if (!domain1_pd) {
        cerr << "domain1_pd is NULL. Domain not active?" << endl;
        usleep(10000);
        return;
    }

    uint16_t status = EC_READ_U16(domain1_pd + off_status_word);
    uint8_t header = EC_READ_U8(domain1_pd + off_header);
    uint8_t instrument_type = EC_READ_U8(domain1_pd + off_instrument_type);
    uint16_t control = EC_READ_U16(domain1_pd + off_control_word);

    cmd = update_status(cmd);
    cout << "cmd is: " << (int)cmd << endl;

   
    if(hee==0)
    {
        hee=26;
        cout<<"writing to header"<<endl;
        EC_WRITE_U8(domain1_pd + off_header,hee);

    }
     // Write back control word if needed (uncomment if required)
    if(hee!=0 &&cmd!=67) EC_WRITE_U16(domain1_pd + off_control_word, cmd);
    uint8_t rfid_uid[6];
    for (int i = 0; i < 6; i++) {
        rfid_uid[i] = EC_READ_U8(domain1_pd + off_rfid_uid[i]);
    }

    cout << hex << "Status: 0x" << status << dec
         << ", Header: " << (int)header
         << ", Instrument Type: " << (int)instrument_type
         << ", RFID UID: ";
    for (int i = 0; i < 6; i++) {
        cout << (int)rfid_uid[i];
        if (i < 5) cout << "-";
    }
    cout << endl;

    check_domain1_state();
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);

    usleep(10000); // 10 ms cycle
}

int main() {
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

    if (ecrt_slave_config_pdos(sc, EC_END, slave_syncs))
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
    // 8) main cyclic loop
    while (1) {
        cyclic_task();
    }

    return 0;
}
