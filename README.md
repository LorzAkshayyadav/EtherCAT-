# Running-Servo-Motor-Drive-Using-EtherCAT-
Communicating with servo motor drive using EtherCAT protocol.  
 ***there is very define order to excute a communication between drive and master*** 
 > 1) Include all necessary libraries which you will be needed for your task</br>
 > 2) Define your drive</br> 
 > 3) Define the parameters you want manipulate or read</br> 
 > 4) Define Ethercat structs and variables related to master and slave</br> 
 > 5) Allocate memory for domain process data</br> 
 > 6) assign varaibles with suitable data type{usually unsigned int} for offsets 
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
   

