# Node State Machine
fsm.c implements the state machine for the Node subsystem as shown here:

![State_Diagram_14-5-22 (1)](https://user-images.githubusercontent.com/67641046/169704065-c144127e-2833-4674-a9cf-f29355fdd1c9.png)

In order to compile: 
- Ensure that your gcc version is up to date with gcc --version (my version is 11.2.0).
- cd into the /fsm subdirectory and do gcc fsm.c -o fsm
- ./fsm
