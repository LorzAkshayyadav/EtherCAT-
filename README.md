# Running-Servo-Motor-Drive-Using-EtherCAT-
Communicating with servo motor drive using EtherCAT protocol.  
 ***there is very define order to excute a communication between drive and master*** 
 > Include all necessary libraries which you will be needed for your task</br>  
 > Define your drive</br> 
 > Define the parameters you want manipulate or read</br> 
 > Define Ethercat structs and variables related to master and slave</br> 
 > Allocate memory for domain process data</br> 
 > assign varaibles with suitable data type{usually unsigned int} for offsets 
   for PDO enteries</br> 
 > create domain pdo enteries array for the parameters we are going to read 
   and write.</br> 
 > define slave pdo entries array which contains bitsize info of our parameter 
   along index and subindex</br> 
 > now do pdo mapping , define an slave_pdos arrays where you devide our 
   parameters according to their transmission type {RxPDO or TxPDO)</br> 
 > sync manager configuration , define an array {slave_syncs} which contains 
   which slave pdo act as output and which one as input</br> 
 > initialize ethercat {check for all possible errors }</br> 
 > defining cycle task fn the main fn where we will read and write from slave.</br> 
   

