# Running-Servo-Motor-Drive-Using-EtherCAT-
Communicating with servo motor drive using EtherCAT protocol.  
 ***there is very define order to excute a communication between drive and master*** 
 > Include all necessary libraries which you will be needed for your task
 > Define your drive
 > Define the parameters you want manipulate or read
 > Define Ethercat structs and variables related to master and slave
 > Allocate memory for domain process data
 > assign varaibles with suitable data type{usually unsigned int} for offsets 
   for PDO enteries
 > create domain pdo enteries array for the parameters we are going to read 
   and write.
 > define slave pdo entries array which contains bitsize info of our parameter 
   along index and subindex
 > now do pdo mapping , define an slave_pdos arrays where you devide our 
   parameters according to their transmission type {RxPDO or TxPDO)
 > sync manager configuration , define an array {slave_syncs} which contains 
   which slave pdo act as output and which one as input
 > initialize ethercat {check for all possible errors }
 > defining cycle task fn the main fn where we will read and write from slave.
   

