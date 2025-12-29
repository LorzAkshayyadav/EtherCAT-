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
#include <thread>
#include <atomic>
#include <chrono>
#include <math.h>
#include <vector>
#include <cstdint>
#include <cstring>
#include <climits>
using namespace std;

#define VENDOR_ID 0x00000009
#define PRODUCT_CODE 0x00009253
#define CSV_CSP_MODULE_ID 0x00119800
#define CSP_MODULE_ID 0x00219800
#define CSV_MODULE_ID 0x00319800

// Two-slot device (Axis0 + Axis1).
static const uint8_t F030_SUB0 = 2; // number of slots / module entries

// Slot increments from ESI.
#define PDO_INC 0x0010   // SlotPdoIncrement="16"
#define OBJ_INC 0x0800   // SlotIndexIncrement="#x800"

// Define constants for the single slave (axis0 base indices)
#define SLAVE_POSITION 0
#define TARGET_POSITION_INDEX 0x607A
#define TARGET_VELOCITY_INDEX 0x60FF
#define TARGET_TORQUE_INDEX 0x6071
#define CONTROL_WORD_INDEX 0x6040
#define STATUS_WORD_INDEX 0x6041
#define ERROR_CODE_INDEX 0x603F
#define RELATIVE_POSITION_INDEX 0x6063
#define ACTUAL_POSITION_INDEX 0x6064
#define ACTUAL_VELOCITY_INDEX 0x606C
#define ACTUAL_TORQUE_INDEX 0x6077
#define OPERATION_MODE_INDEX 0x6060

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

// Process data
uint8_t *domain1_pd = NULL;

constexpr unsigned int INVALID_OFFSET = UINT_MAX;
static const size_t NUM_AXES = 2;

#define AX_OBJ(axis, idx) ((uint16_t)((idx) + (axis)*OBJ_INC))
#define AX_RXPDO(base, axis) ((uint16_t)((base) + (axis)*PDO_INC))
#define AX_TXPDO(base, axis) ((uint16_t)((base) + (axis)*PDO_INC))

enum class ModuleSelection
{
    CSV_CSP,
    CSP,
    CSV
};

static ModuleSelection selected_module = ModuleSelection::CSV_CSP; // change to match desired module

struct AxisOffsets
{
    unsigned int control_word;
    unsigned int status_word;
    unsigned int error_code;
    unsigned int target_position;
    unsigned int target_velocity;
    unsigned int target_torque;
    unsigned int actual_position;
    unsigned int relative_position;
    unsigned int actual_velocity;
    unsigned int actual_torque;
    unsigned int operation_mode_disp;
};

struct PdoOffsets
{
    AxisOffsets axis[2];
};

static PdoOffsets offsets = {
    {
        {INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET,
         INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET},
        {INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET,
         INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET, INVALID_OFFSET},
    }};

static inline bool has_offset(unsigned int off)
{
    return off != INVALID_OFFSET;
}

static inline uint16_t to_le16(uint16_t v) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return v;
#else
    return __builtin_bswap16(v);
#endif
}

static int mdp_write_f030(ec_slave_config_t *sc,
                          uint8_t f030_sub0,
                          const std::vector<uint32_t> &selected_modules)
{
    // NOTE: your firmware expects SI0 as UINT16 during CompleteAccess
    std::vector<uint8_t> buf(2 + 4 * f030_sub0, 0);

    uint16_t si0 = to_le16((uint16_t)f030_sub0);
    std::memcpy(&buf[0], &si0, 2);

    for (size_t i = 0; i < selected_modules.size() && i < f030_sub0; ++i) {
        uint32_t le = to_le32(selected_modules[i]);
        std::memcpy(&buf[2 + 4 * i], &le, 4);
    }

    return ecrt_slave_config_complete_sdo(sc, 0xF030, buf.data(), buf.size());
}


// --- PDO tables per module (two slots with slot/object increments) ---

// CSV+CSP dynamic module (0x1600/0x1A00 + increments).
static const ec_pdo_entry_info_t csv_csp_rx_ax0[] = {
    {AX_OBJ(0, CONTROL_WORD_INDEX), 0x00, 16},
    {AX_OBJ(0, TARGET_TORQUE_INDEX), 0x00, 16},
};
static const ec_pdo_entry_info_t csv_csp_rx_ax1[] = {
    {AX_OBJ(1, CONTROL_WORD_INDEX), 0x00, 16},
    {AX_OBJ(1, TARGET_TORQUE_INDEX), 0x00, 16},
};
static const ec_pdo_entry_info_t csv_csp_tx_ax0[] = {
    {AX_OBJ(0, ERROR_CODE_INDEX), 0x00, 16},
    {AX_OBJ(0, STATUS_WORD_INDEX), 0x00, 16},
    {AX_OBJ(0, RELATIVE_POSITION_INDEX), 0x00, 32},
    {AX_OBJ(0, ACTUAL_POSITION_INDEX), 0x00, 32},
    {AX_OBJ(0, ACTUAL_VELOCITY_INDEX), 0x00, 32},
    {AX_OBJ(0, ACTUAL_TORQUE_INDEX), 0x00, 16},
};
static const ec_pdo_entry_info_t csv_csp_tx_ax1[] = {
    {AX_OBJ(1, ERROR_CODE_INDEX), 0x00, 16},
    {AX_OBJ(1, STATUS_WORD_INDEX), 0x00, 16},
    {AX_OBJ(1, RELATIVE_POSITION_INDEX), 0x00, 32},
    {AX_OBJ(1, ACTUAL_POSITION_INDEX), 0x00, 32},
    {AX_OBJ(1, ACTUAL_VELOCITY_INDEX), 0x00, 32},
    {AX_OBJ(1, ACTUAL_TORQUE_INDEX), 0x00, 16},
};

static const ec_pdo_info_t pdos_csv_csp[] = {
    {AX_RXPDO(0x1600, 0), 2, csv_csp_rx_ax0},
   {AX_RXPDO(0x1600, 1), 2, csv_csp_rx_ax1},
    {AX_TXPDO(0x1A00, 0), 6, csv_csp_tx_ax0},
   {AX_TXPDO(0x1A00, 1), 6, csv_csp_tx_ax1},
};

static const ec_sync_info_t syncs_csv_csp[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, pdos_csv_csp + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, pdos_csv_csp + 2, EC_WD_DISABLE},
    {0xff}};

static const ec_pdo_entry_reg_t domain_regs_csv_csp[] = {
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, CONTROL_WORD_INDEX), 0x00, &offsets.axis[0].control_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, TARGET_TORQUE_INDEX), 0x00, &offsets.axis[0].target_torque},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, ERROR_CODE_INDEX), 0x00, &offsets.axis[0].error_code},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, STATUS_WORD_INDEX), 0x00, &offsets.axis[0].status_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, RELATIVE_POSITION_INDEX), 0x00, &offsets.axis[0].relative_position},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, ACTUAL_POSITION_INDEX), 0x00, &offsets.axis[0].actual_position},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, ACTUAL_VELOCITY_INDEX), 0x00, &offsets.axis[0].actual_velocity},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, ACTUAL_TORQUE_INDEX), 0x00, &offsets.axis[0].actual_torque},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, CONTROL_WORD_INDEX), 0x00, &offsets.axis[1].control_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, TARGET_TORQUE_INDEX), 0x00, &offsets.axis[1].target_torque},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, ERROR_CODE_INDEX), 0x00, &offsets.axis[1].error_code},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, STATUS_WORD_INDEX), 0x00, &offsets.axis[1].status_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, RELATIVE_POSITION_INDEX), 0x00, &offsets.axis[1].relative_position},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, ACTUAL_POSITION_INDEX), 0x00, &offsets.axis[1].actual_position},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, ACTUAL_VELOCITY_INDEX), 0x00, &offsets.axis[1].actual_velocity},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, ACTUAL_TORQUE_INDEX), 0x00, &offsets.axis[1].actual_torque},
    {}};

// CSP-only module (0x1601/0x1A01 + increments).
static const ec_pdo_entry_info_t csp_rx_ax0[] = {
    {AX_OBJ(0, CONTROL_WORD_INDEX), 0x00, 16},
    {AX_OBJ(0, TARGET_POSITION_INDEX), 0x00, 32},
};
static const ec_pdo_entry_info_t csp_rx_ax1[] = {
    {AX_OBJ(1, CONTROL_WORD_INDEX), 0x00, 16},
    {AX_OBJ(1, TARGET_POSITION_INDEX), 0x00, 32},
};
static const ec_pdo_entry_info_t csp_tx_ax0[] = {
    {AX_OBJ(0, STATUS_WORD_INDEX), 0x00, 16},
    {AX_OBJ(0, ACTUAL_POSITION_INDEX), 0x00, 32},
};
static const ec_pdo_entry_info_t csp_tx_ax1[] = {
    {AX_OBJ(1, STATUS_WORD_INDEX), 0x00, 16},
    {AX_OBJ(1, ACTUAL_POSITION_INDEX), 0x00, 32},
};

static const ec_pdo_info_t pdos_csp[] = {
    {AX_RXPDO(0x1601, 0), 2, csp_rx_ax0},
    {AX_RXPDO(0x1601, 1), 2, csp_rx_ax1},
    {AX_TXPDO(0x1A01, 0), 2, csp_tx_ax0},
    {AX_TXPDO(0x1A01, 1), 2, csp_tx_ax1},
};

static const ec_sync_info_t syncs_csp[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, pdos_csp + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, pdos_csp + 2, EC_WD_DISABLE},
    {0xff}};

static const ec_pdo_entry_reg_t domain_regs_csp[] = {
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, CONTROL_WORD_INDEX), 0x00, &offsets.axis[0].control_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, TARGET_POSITION_INDEX), 0x00, &offsets.axis[0].target_position},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, STATUS_WORD_INDEX), 0x00, &offsets.axis[0].status_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, ACTUAL_POSITION_INDEX), 0x00, &offsets.axis[0].actual_position},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, CONTROL_WORD_INDEX), 0x00, &offsets.axis[1].control_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, TARGET_POSITION_INDEX), 0x00, &offsets.axis[1].target_position},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, STATUS_WORD_INDEX), 0x00, &offsets.axis[1].status_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, ACTUAL_POSITION_INDEX), 0x00, &offsets.axis[1].actual_position},
    {}};

// CSV-only module (0x1602/0x1A02 + increments).
static const ec_pdo_entry_info_t csv_rx_ax0[] = {
    {AX_OBJ(0, CONTROL_WORD_INDEX), 0x00, 16},
    {AX_OBJ(0, TARGET_VELOCITY_INDEX), 0x00, 32},
};
static const ec_pdo_entry_info_t csv_rx_ax1[] = {
    {AX_OBJ(1, CONTROL_WORD_INDEX), 0x00, 16},
    {AX_OBJ(1, TARGET_VELOCITY_INDEX), 0x00, 32},
};
static const ec_pdo_entry_info_t csv_tx_ax0[] = {
    {AX_OBJ(0, STATUS_WORD_INDEX), 0x00, 16},
    {AX_OBJ(0, ACTUAL_VELOCITY_INDEX), 0x00, 32},
};
static const ec_pdo_entry_info_t csv_tx_ax1[] = {
    {AX_OBJ(1, STATUS_WORD_INDEX), 0x00, 16},
    {AX_OBJ(1, ACTUAL_VELOCITY_INDEX), 0x00, 32},
};

static const ec_pdo_info_t pdos_csv[] = {
    {AX_RXPDO(0x1602, 0), 2, csv_rx_ax0},
    {AX_RXPDO(0x1602, 1), 2, csv_rx_ax1},
    {AX_TXPDO(0x1A02, 0), 2, csv_tx_ax0},
    {AX_TXPDO(0x1A02, 1), 2, csv_tx_ax1},
};

static const ec_sync_info_t syncs_csv[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, pdos_csv + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, pdos_csv + 2, EC_WD_DISABLE},
    {0xff}};

static const ec_pdo_entry_reg_t domain_regs_csv[] = {
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, CONTROL_WORD_INDEX), 0x00, &offsets.axis[0].control_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, TARGET_VELOCITY_INDEX), 0x00, &offsets.axis[0].target_velocity},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, STATUS_WORD_INDEX), 0x00, &offsets.axis[0].status_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(0, ACTUAL_VELOCITY_INDEX), 0x00, &offsets.axis[0].actual_velocity},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, CONTROL_WORD_INDEX), 0x00, &offsets.axis[1].control_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, TARGET_VELOCITY_INDEX), 0x00, &offsets.axis[1].target_velocity},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, STATUS_WORD_INDEX), 0x00, &offsets.axis[1].status_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, AX_OBJ(1, ACTUAL_VELOCITY_INDEX), 0x00, &offsets.axis[1].actual_velocity},
    {}};

struct ModuleMapping
{
    ModuleSelection selection;
    uint32_t module_id;
    const ec_sync_info_t *syncs;
    const ec_pdo_entry_reg_t *domain_regs;
    const char *label;
    uint8_t startup_moo;
};

static const ModuleMapping module_mappings[] = {
    {ModuleSelection::CSV_CSP, CSV_CSP_MODULE_ID, syncs_csv_csp, domain_regs_csv_csp, "CSV+CSP", 0x0A},
    {ModuleSelection::CSP, CSP_MODULE_ID, syncs_csp, domain_regs_csp, "CSP", 0x08},
    {ModuleSelection::CSV, CSV_MODULE_ID, syncs_csv, domain_regs_csv, "CSV", 0x09},
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
int cmd = 0;
uint16_t update_status(uint16_t status, uint16_t cmd) {
    // Fault handling
   // cout<<"we are applying CIA402"<<endl;
    if (status & (1 << 3)) {
    cout<<"fault reset happening"<<endl;
        return 128;
    }
    
    //if ((status & 0x006F) && (status & (1 << 6))) {
      // cout<<"Enabling voltage"<<endl;
       // return 128; //->it will do fault reset
   // }

    if (((status | 65456) ^ 65520) == 0 && cmd != 6) {
        cout<<"taking it to ready to switch on"<<endl;
        cmd = 6; //->it will take drive to ready to switch on{110}
    } 
    else if (((status | 65424) ^ 65457) == 0 && cmd != 7) {
    cout<<"switching on"<<endl;
        cmd = 7; //->it will switched on the drive {111}
    } 
    else if (((status | 65424) ^ 65459) == 0 && cmd != 15) {
    cout<<"operation enabled"<<endl;
        cmd = 15;// ->it will enable the operation {1111}
    } 
    else if (((status | 65424) ^ 65463) == 0) {
        std::cout << "Operation Enabled \n";
    }

    return cmd;
}


atomic<bool> first_loop{true};
int pos=0;
int step=0;
void cyclic_task(int target_pos, bool temp)
{

    // Process EtherCAT master and domain
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // Check the domain state
    check_domain1_state();
    
    const size_t axis = 0;
    AxisOffsets &ax = offsets.axis[axis];

    uint16_t status = has_offset(ax.status_word) ? EC_READ_U16(domain1_pd + ax.status_word) : 0;
    int actual_pos = has_offset(ax.actual_position) ? EC_READ_S32(domain1_pd + ax.actual_position) : 0;
    int actual_vel = has_offset(ax.actual_velocity) ? EC_READ_S32(domain1_pd + ax.actual_velocity) : 0;
   // int op_mode = has_offset(ax.operation_mode_disp) ? EC_READ_S8(domain1_pd + ax.operation_mode_disp) : 0;

   cout << "Status: 0x" << hex << status
       << ", Position: " << dec << actual_pos
       << ", Velocity: " << actual_vel <<"\n";
    
    //cmd = update_status(status, cmd);
    //if (has_offset(ax.control_word))
    //{
   //     EC_WRITE_U16(domain1_pd + ax.control_word, cmd);
   // }
   // cout<<"Target :"<<target_pos<<endl;
   // if (((status | 65424) ^ 65463) == 0)
   // {
    	
    	// if (first_loop.exchange(false)) {   // true -> false atomically
      //  pos = actual_pos;
      //  if (target_pos > actual_pos) step = 100;
      //  else step = -100;

       // cout << "steps are " << dec << step << endl;
       // }
    	//else
    	//{
        //        if (selected_module == ModuleSelection::CSV && has_offset(ax.target_velocity))
        //        {
        //            EC_WRITE_S32(domain1_pd + ax.target_velocity, target_pos);
         //           std::cout << "target velocity: " << dec << target_pos << "\n";
        //        }
         //       else if (selected_module == ModuleSelection::CSV_CSP && has_offset(ax.target_torque))
          //      {
           //         EC_WRITE_S16(domain1_pd + ax.target_torque, static_cast<int16_t>(target_pos));
          //          std::cout << "target torque: " << dec << target_pos << "\n";
           //     }
          //      else if (has_offset(ax.target_position) && has_offset(ax.actual_position))
          //      {
           //         if(abs(target_pos-actual_pos)<100)
          //          {
             //           step=target_pos-actual_pos;
             //           cout<<"step"<<dec<<step<<endl;
            //        }

              //      pos = pos + step ;
              //      EC_WRITE_S32(domain1_pd + ax.target_position, pos);
               //     std::cout << "target position: "<< dec<< pos<<"\n";
              //  }
    	//}
    	
    //}



    // Send EtherCAT process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    usleep(5000);
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

    const ModuleMapping *mapping = nullptr;
    for (const auto &m : module_mappings)
    {
        if (m.selection == selected_module)
        {
            mapping = &m;
            break;
        }
    }

    if (!mapping)
    {
        cerr << "Invalid module selection requested." << endl;
        return -1;
    }

   // --- Module selection per slot (PREOP startup SDOs) ---
// Configure 2 slots
if (ecrt_slave_config_sdo8(sc, 0xF030, 0x00, 2)) {
    cerr << "Failed to queue SDO 0xF030:0" << endl;
    return -1;
}

// Slot 1 (axis0) module ID
if (ecrt_slave_config_sdo32(sc, 0xF030, 0x01, CSV_CSP_MODULE_ID)) {   // or CSP_MODULE_ID / CSV_MODULE_ID
    cerr << "Failed to queue SDO 0xF030:1" << endl;
    return -1;
}

// Slot 2 (axis1) module ID
if (ecrt_slave_config_sdo32(sc, 0xF030, 0x02, CSP_MODULE_ID)) {       // example: different module per slot
    cerr << "Failed to queue SDO 0xF030:2" << endl;
    return -1;
}

cout << "MDP module selection queued (0xF030)." << endl;

    // Set initial Mode of Operation via startup SDO according to chosen module.
    if (ecrt_slave_config_sdo8(sc, OPERATION_MODE_INDEX, 0x00, mapping->startup_moo))
    {
        cerr << "Failed to queue startup SDO for mode of operation." << endl;
        return -1;
    }

    if (ecrt_slave_config_pdos(sc, EC_END, mapping->syncs))
    {
        cerr << "Failed to configure PDOs." << endl;
        return -1;
    }
    cout << "PDO configuration successful." << endl;

    // Register PDO entries
    if (ecrt_domain_reg_pdo_entry_list(domain1, mapping->domain_regs))
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

   
    atomic<bool> temp(false);  // Use atomic to ensure safe update in different threads
    atomic<int> target_pos(0);
    

    

    // Input thread: will allow updating target_pos without blocking the loop
thread input_thread([&]() {
    int new_pos;
    while (cin >> new_pos) {
        first_loop.store(true);     // reset for new target
        target_pos.store(new_pos);  // update target
    }
});



    // Main loop: running the cyclic task
    while (!temp) {
        cyclic_task(target_pos.load(), temp.load());  // Pass the value of temp
 // Perform cyclic task with current target position
    }

    // Wait for the input thread to finish
    input_thread.join();

    
    return 0;
}
