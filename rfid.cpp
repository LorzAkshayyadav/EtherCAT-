#include <errno.h>   
#include <stdio.h>           
#include <stdint.h>         
#include <unistd.h>          
#include <iostream>          
#include "ecrt.h"            
  

using namespace std;

// EtherCAT Slave Definitions
#define VENDOR_ID  7B000000
#define PRODUCT_CODE 0x00009252

#define SLAVE_POSITION 0
#define CONTROL_WORD 0x7000
#define STATUS_WORD 0x6000
#define HEADER 0x6000
#define INSTRUMENT_TYPE 0x6000
#define QC_STATUS 0X6000
#define RFID_UID_START 0x6000  // UID starts from SubIndex 5

// EtherCAT
static ec_master_t *master = NULL;
static ec_domain_t *domain1 = NULL;
static ec_master_state_t master_state = {};
static ec_domain_state_t domain1_state = {};
uint8_t *domain1_pd = NULL;


uint8_t *domain1_pd = NULL;

// Offsets for PDO entries
unsigned int off_control_word;
unsigned int off_status_word;
unsigned int off_header;
unsigned int off_instrument_type;
unsigned int off_rfid_uid[6];  // Array for UID offsets
unsigned int off_qc_status;
uint8_t cmd = 0;

// PDO Entry Registration (Including RFID UID Bytes)
const ec_pdo_entry_reg_t domain1_pdo_entries[] = {
    {SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, CONTROL_WORD, &off_control_word},
    {SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, STATUS_WORD,01, &off_status_word},
    {SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, HEADER,02,&off_header},
    {SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, INSTRUMENT_TYPE,03, &off_instrument_type},
    {SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, QC_STATUS,17, &off_qc_status},
    {SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, RFID_UID_START , 05, &off_rfid_uid[0]},
    {SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, RFID_UID_START , 06, &off_rfid_uid[1]},
    {SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, RFID_UID_START , 07, &off_rfid_uid[2]},
    {SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, RFID_UID_START , 08, &off_rfid_uid[3]},
    {SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, RFID_UID_START , 09, &off_rfid_uid[4]},
    {SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, RFID_UID_START , 16, &off_rfid_uid[5]},
    {}
};

const ec_pdo_entry_info_t slave_pdo_entries[] = {
    {CONTROL_WORD, 01, 16},   
    {STATUS_WORD, 01, 16}, 
    {HEADER, 02, 8},   
    {INSTRUMENT_TYPE, 03, 8},
    {RFID_UID_START , 05,16},
    {RFID_UID_START , 06,8},
    {RFID_UID_START , 07,8},
    {RFID_UID_START , 08, 8},
    {RFID_UID_START , 09, 8},
    {RFID_UID_START , 16 ,8},     
    {QC_STATUS, 17, 8}, 
};

// PDO mapping
const ec_pdo_info_t slave_pdos[] = {
    {0x1600, 1, slave_pdo_entries + 0}, // RxPDO
    {0x1A00, 10, slave_pdo_entries + 1}, // TxPDO
};

// Sync manager configuration
const ec_sync_info_t slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_pdos + 1, EC_WD_DISABLE},
    {0xff}};
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
    uint16_t update_status(uint16_t status, uint16_t cmd) {
        // Fault handling
        if (status & (1 << 3)) {
            return 128;
        }
        
        if ((status & 0x006F) && (status & (1 << 6))) {
            return 128; //->it will do fault reset
        }
    
        if (((status | 65456) ^ 65520) == 0 && cmd != 6) {
            cmd = 6; //->it will take drive to ready to switch on{110}
        } 
        else if (((status | 65424) ^ 65457) == 0 && cmd != 7) {
            cmd = 7; //->it will switched on the drive {111}
        } 
        else if (((status | 65424) ^ 65459) == 0 && cmd != 15) {
            cmd = 15;// ->it will enable the operation {1111}
        } 
        else if (((status | 65424) ^ 65463) == 0) {
            std::cout << "Operation Enabled \n";
        }
    
        return cmd;
    }

// Read updated PDOs from EtherCAT
void cyclic_task() {
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    uint16_t status = EC_READ_U16(domain1_pd + off_status_word);
    uint8_t header = EC_READ_U8(domain1_pd + off_header);
    uint8_t instrument_type = EC_READ_U8(domain1_pd + off_instrument_type);

    uint8_t rfid_uid[6];
    for (int i = 0; i < 6; i++) {
        rfid_uid[i] = EC_READ_U8(domain1_pd + off_rfid_uid[i]);
    }

    // Print the updated RFID data
    cout << "Status: 0x" << hex << status
         << ", Header: " << (int)header
         << ", Instrument Type: " << (int)instrument_type
         << ", RFID UID: ";

    for (int i = 0; i < 6; i++) {
        printf("%02X ", rfid_uid[i]);
    }
    cout << endl;

    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    usleep(10000);
}

// Main Function
int main() {
    setup_spi();  // Initialize SPI for RFID communication

    master = ecrt_request_master(0);
    if (!master) {
        cerr << "Failed to request EtherCAT Master." << endl;
        return -1;
    }
    cout << "EtherCAT Master requested successfully." << endl;

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        cerr << "Failed to create domain." << endl;
        return -1;
    }
    cout << "Domain created successfully." << endl;

    // Get Slave Configuration
    ec_slave_config_t *sc;
    if (!(sc = ecrt_master_slave_config(master, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE))) {
        cerr << "Failed to get slave configuration." << endl;
        return -1;
    }
    cout << "Slave configuration successful." << endl;

    // Activate Master
    if (ecrt_master_activate(master)) {
        cerr << "Failed to activate master." << endl;
        return -1;
    }
    cout << "Master activated successfully." << endl;

    domain1_pd = ecrt_domain_data(domain1);
    if (!domain1_pd) {
        cerr << "Failed to get domain data." << endl;
        return -1;
    }
    cout << "Domain data retrieved successfully." << endl;

    while (1) {
       
        cyclic_task();       
    }

    return 0;
}
