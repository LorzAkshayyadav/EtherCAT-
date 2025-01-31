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
 > 7) create domain pdo enteries array for the parameters we are going to read 
   and write.</br> 
 > 8) define slave pdo entries array which contains bitsize info of our parameter 
   along index and subindex</br> 
 > 9) now do pdo mapping , define an slave_pdos arrays where you devide our 
   parameters according to their transmission type {RxPDO or TxPDO)</br> 
 > 10) sync manager configuration , define an array {slave_syncs} which contains 
   which slave pdo act as output and which one as input</br> 
 > 11) initialize ethercat {check for all possible errors }</br> 
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
## Step 7: create domain pdo enteries array for the parameters we are going to read and write
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
## Step 8: define slave pdo entries array which contains bitsize info of our parameter along index and subindex
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
## Step 9: 

 

