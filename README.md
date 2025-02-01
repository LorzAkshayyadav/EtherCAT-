# Running-Servo-Motor-Drive-Using-EtherCAT-
Communicating with servo motor drive using EtherCAT protocol.  
 ***there is very define order to excute a communication between drive and master*** 
 > 1) Include all necessary libraries which you will be needed for your task</br>
 > 2) Define your drive</br> 
 > 3) Define the parameters you want manipulate or read</br> 
 > 4) Define Ethercat structs and variables related to master and slave</br> 
 > 5) Allocate memory for domain process data</br> 
 > 6) Assign varaibles with suitable data type{usually unsigned int} for offsets 
   for PDO enteries</br> 
 > 7) Create domain pdo enteries array for the parameters we are going to read 
   and write.</br> 
 > 8) Define slave pdo entries array which contains bitsize info of our parameter 
   along index and subindex</br> 
 > 9) Now do pdo mapping , define an slave_pdos arrays where you devide our 
   parameters according to their transmission type {RxPDO or TxPDO)</br> 
 > 10) Sync manager configuration , define an array {slave_syncs} which contains 
   which slave pdo act as output and which one as input</br> 
 > 11) Initialize ethercat {check for all possible errors }</br> 
 > 12) defining cycle task fn the main fn where we will read and write from slave.</br>
 
 **We will now explore each order step wise and will look in the details**
## Step 1:Include all necessary libraries which you will be needed for your task 
 ```
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
```
## Step 2: Define your drive
```
#define VENDOR_ID 0x0000029C
#define PRODUCT_CODE 0x5EE1
there are two way to define your drive you can do that way too but in case of multiple slaves better go with this way
#define {Driver Name} {Vendor_Id}, {Product_Code}
#define Beckhoff_EL2004 0x00000002, 0x07d43052

```
## Step 3: Define the parameters you want manipulate or read
```
As our task is to rotate our servo motor to a target position so the parameter we need to access are these :
#define SLAVE_POSITION 1
#define TARGET_POSITION_INDEX 0x607A
#define CONTROL_WORD_INDEX 0x6040
#define STATUS_WORD_INDEX 0x6041
#define ACTUAL_POSITION_INDEX 0x6064
#define ACTUAL_VELOCITY_INDEX 0x606C
#define OPERATION_MODE_INDEX 0x6060
usually we need Index and Subindex so we define both but in our case the subindex is zero only so we didn't defined
#define TARGET_POSITION {Index},{Subindex}
```
## Step 4: Define Ethercat structs and variables related to master and slave
```
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};
static ec_slave_config_t *slave_config = NULL;
static ec_slave_config_state_t slave_config_state = {};

```
## Step 5: Allocate memory for domain process data
```
We alocate memory for pdo's domain
uint8_t *domain1_pd = NULL;
```
## Step 6:  Assign varaibles with suitable data type{usually unsigned int} for offsets
```
The EtherCAT master must know these offsets to correctly read/write data.
unsigned int off_control_word;
unsigned int off_status_word;
unsigned int off_target_position;
unsigned int off_actual_position;
unsigned int off_actual_velocity;
unsigned int off_operation_mode;

```
## Step 7: Create domain pdo enteries array for the parameters we are going to read and write
```
# Keep one thing in mind always put recieve parameters together and transmit parameter together 
const ec_pdo_entry_reg_t domain1_pdo_entries[] = {
   //{Slave_alias _address,Slave position, vendor id, product code, PDO Entry Index , PDO Entry Subindex ,*offset } 
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, CONTROL_WORD_INDEX, 0x00, &off_control_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, TARGET_POSITION_INDEX, 0x00, &off_target_position},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, OPERATION_MODE_INDEX, 0x00, &off_operation_mode},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, STATUS_WORD_INDEX, 0x00, &off_status_word},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, ACTUAL_POSITION_INDEX, 0x00, &off_actual_position},
    {0, SLAVE_POSITION, VENDOR_ID, PRODUCT_CODE, ACTUAL_VELOCITY_INDEX, 0x00, &off_actual_velocity},

    {}};
```
## Step 8: Define slave pdo entries array which contains bitsize info of our parameter along index and subindex
```
const ec_pdo_entry_info_t slave_pdo_entries[] = {
   //{parameter_index,parameter_subindex,bit_length}
    {CONTROL_WORD_INDEX, 0x00, 16},    // Control Word
    {TARGET_POSITION_INDEX, 0x00, 32}, // Target Position
    {OPERATION_MODE_INDEX, 0x00, 8},   // Mode of Operation
    {STATUS_WORD_INDEX, 0x00, 16},     // Status Word
    {ACTUAL_POSITION_INDEX, 0x00, 32}, // Actual Position
    {ACTUAL_VELOCITY_INDEX, 0x00, 32}, // Actual Velocity
};
# top 3 parameters we are going to use to write so together and last three parameter we are going to use to read
```
## Step 9: Now do pdo mapping , define an slave_pdos arrays where you devide our  parameters according to their transmission type {RxPDO or TxPDO)
```
// first 3 parameter as mapped as RxPDO to write to slave on master command and last 3 parameter assigned as TxPDO to read
const ec_pdo_info_t slave_pdos[] = {
   // {PDO Index ,no_of pdo enteries , adrress to base of array enteries }
    {0x1600, 3, slave_pdo_entries + 0}, // RxPDO
    {0x1A00, 3, slave_pdo_entries + 3}, // TxPDO
};
// for example in our case we are regestring control word to Mode of operation as RxPDO ,
 Control word is at 0th index of slave pdo enteries array so this will act as base address ,
 in TxPDO Status word to Actual velocity , our status word is at 3rd index so base address will
 be slave_pdo_enteries + 3 //

```
## Step 10: Sync manager configuration , define an array {slave_syncs} which contains which slave pdo act as output and which one as input
```
const ec_sync_info_t slave_syncs[] = {
   //{ sync manager index , Data direction , no of pdo assigned , pointer to pdo config , watchdog setting}
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_pdos + 1, EC_WD_DISABLE},
    {0xff}};
/*
 Sync Manager 0 & 1
These are typically used for mailbox communication (CoE, FoE, etc.).
They are not assigned any PDOs.
Watchdog is disabled.

Sync Manager 2 & 3
These are for Process Data (PDO) communication.
SM2 is output (master → slave), and SM3 is input (slave → master).
slave_pdos + 0 and slave_pdos + 1 point to structures that define the PDOs.

0xFF Terminator
This marks the end of the array so that iterating code can detect it.
   */
same explaination of base address goes here
```
## Step 11: initialize ethercat {check for all possible errors }
```
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


```
## Step 12: defining cycle task fn the main fn where we will read and write from slave.
![Status_word_table]([Screenshot 2025-01-31 180659.png](https://github.com/LorzAkshayyadav/Running-Servo-Motor-Drive-Using-EtherCAT-/blob/58a2d0e2492bf244473ebb1b18719e746fb60563/Screenshot%202025-01-31%20180659.png))
```
We have to verify that drive is in motion state or not for that we will check lower 7 bit of Status Word and check if they are like that or X11 0111 ,and will check if there is falult and according using control word we will write to drive enable the operation 
void cyclic_task(int target_pos, bool &temp)
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
    {
        cerr << "Fault detected in the drive!" << endl;
        EC_WRITE_U16(domain1_pd + off_control_word, 0x0080);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        usleep(10000);
        return;
    }

    //  "Switch On Disabled"
    if ((status & 0x006F) == 0x0040)
    {
        cerr << "Drive in 'Switch On Disabled' state, resetting fault..." << endl;
        EC_WRITE_U16(domain1_pd + off_control_word, 0x0080); // Fault Reset
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        usleep(10000);
        return;
    }
// Here we are checking that drive is in enable operation mode or not if not we will first put it in enable operation mode after that only we will write our target position
    if ((status & 0x006F) != 0x0027)
    {
        uint16_t control_word = 0x0000;

        if ((status & 0x006F) == 0x0021)
        {
            control_word |= (1 << 1) | (1 << 2); // Enable Voltage
        }
        else if ((status & 0x006F) == 0x0023)
        {
            control_word |= (1 << 1) | (1 << 0) | (1 << 2); // Switch On + Enable Voltage
        }
        else if ((status & 0x006F) == 0x0027)
        {
            control_word |= (1 << 1) | (1 << 0) | (1 << 3) | (1 << 2); // Enable Operation
        }

        EC_WRITE_U16(domain1_pd + off_control_word, control_word);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        usleep(10000);
        return;
    }

    EC_WRITE_S8(domain1_pd + off_operation_mode, 8); // 8 = Cyclic Synchronous Position mode
    EC_WRITE_S32(domain1_pd + off_target_position, target_pos);

    if (status & (1 << 10)) //if 10th bit got set that means our set target has been reached and we can return 
    {
        cout << "Target position reached!" << endl;
        temp = true;

        return;
    }

    // Send EtherCAT process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    usleep(10000);
    return;
}
```



 

