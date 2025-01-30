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
 
 **We will now explore each order step wise and will look in the details**
 ## Step 1:Include all necessary libraries which you will be needed for your task  
 "' #include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include "ecrt.h"
#include <iostream>   "'
 

