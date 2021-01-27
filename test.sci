A=[1,1,1,1,1,1;-1,1,0.5,-0.5,-0.5,0.5;0,0,sqrt(3)/2,-sqrt(3)/2,sqrt(3)/2,-sqrt(3)/2;1,-1,1,-1,-1,1];
B=[1/6,-1/3,0,1/6;1/6,1/3,0,-1/6;1/6,1/6,1/(2*sqrt(3)),1/6;1/6,-1/6,-1/(2*sqrt(3)),-1/6;1/6,-1/6,1/(2*sqrt(3)),-1/6;1/6,1/6,-1/(2*sqrt(3)),1/6];

//
// Attitude (Z), Speed (Z) e Accel (Z) sono espresse in m, m/sec e m/sec^2 e rispetto al sistema inerziale (terra)
// Attitude (R,PY), Speed (R,P,Y) e Accel (R,P,Y) sono espresse in rad, rad/sec e rad/sec^2 e rispetto al Body Frame del Drone
//

// ************
// **** RC ****
// ************
RC_REQ_ATTITUDE=[30;%pi/6;0;0];
RC_REQ_SPEED=[0;0;0;0]; // Calcolati in M1
RC_REQ_ACCEL=[0;0;0;0]; // Calcolati in M1

// *************
// **** IMU ****
// *************
IMU_EST_ATTITUDE=[0;1;0;0];
IMU_EST_SPEED=[0;0;0;0];
IMU_EST_ACCEL=[25;0;0;0];
IMU_EST_ATTITUDE_PREV=[0;1;0;0];
IMU_EST_SPEED_PREV=[0;0;0;0];
IMU_EST_ACCEL_PREV=[25;0;0;0];

// *************
// **** PWM ****
// *************
PWM_FREQUENCY=490; // Espresso in Hz

// Duty Cycle Espresso in % in un range [%min, %max] definito nel modo seguente
//            %min = (1 − (1÷PWM_FREQUENCY−0.001)÷(1÷PWM_FREQUENCY))×100
//            %max = (1 − (1÷PWM_FREQUENCY−0.002)÷(1÷PWM_FREQUENCY))×100
PWM_DUTY_CYCLE=[0;0;0;0;0;0];

// ************
// **** M1 ****
// ************
// Description..:  Calcola le variazioni da eseguire (rispetto allo stato attuale e a quanto è richiesto) per raggiungere il target
//
// Parameters..:   dt (reattività espressa in secondi come intervallo di tempo [0,dt])
//
// Input.......:   RC_REQ_ATTITUDE (Altezza in m rispetto al livello del mare e inclinazioni 3D in rad)
//                 RC_REQ_SPEED (Speed (Z) in m/sec e velocità angolari in rad/sec)
//                 RC_REQ_ACCEL (Accel (Z) in m/sec^2 e accelerazioni angolari in rad/sec^2)
//                 IMU_EST_ATTITUDE (Stima Altezza e inclinazioni 3D in rad)
//                 IMU_EST_SPEED (Stima Speed (Z) in m/sec e velocità angolari in rad/sec)
//                 IMU_EST_ACCEL (Stima Accel (Z) espressa in m/sec^2 e accelerazioni angolari in rad/sec^2)
//
// Output......:   RC_REQ_SPEED = Speed (Z) in m/sec e velocità angolari richieste in rad/sec
//                 RC_REQ_ACCEL = Accel (Z) in m/sec^2, accelerazioni angolari 3D in rad/sec^2
//                 M1_OUT = Variazioni Richieste (Variazione Accel (Z) in m/sec^2 e variazioni accelerazioni angolari 3D in rad/sec^2)
//
dt=1;
RC_REQ_SPEED=(RC_REQ_ATTITUDE-IMU_EST_ATTITUDE)/dt; // Calcolo velocità richieste
RC_REQ_ACCEL=(RC_REQ_SPEED - IMU_EST_SPEED)/dt; // calcolo accelerazioni richieste
M1_OUT=RC_REQ_ACCEL-IMU_EST_ACCEL

// ************
// **** M2 ****
// ************
//   Trasformazione: Variazione Moto Richiesto -> Variazioni Forze Motori Richieste
//
M2_IN=M1_OUT;
M2_OUT=B*M2_IN

// ************
// **** M3 ****
// ************
//   Controller PID: Forze Motori Richieste -> Forze Motori corrette

// Stato iniziale:
M3_P_ERR_PREV=[0;0;0;0;0;0];
M3_I_ERR=[0;0;0;0;0;0];
M3_D_ERR=[0;0;0;0;0;0];

// Calcolo PID ad ogni ciclo
M3_IN=M2_OUT;
M3_P_ERR=M3_IN;
M3_I_ERR=M3_I_ERR + (M3_P_ERR - M3_P_ERR_PREV);
M3_D_ERR=(M3_P_ERR - M3_P_ERR_PREV);
kp = 2;
ki = 1;
kd = 1;
M3_OUT = kp*M3_P_ERR+ki*M3_I_ERR+kd*M3_D_ERR

// ************
// **** M4 ****
// ************
//   Trasformazione: Forze Motori -> PWM
//
M4_IN=M3_OUT;
kdt=3;
M4_DUTY_CYCLE=kdt*sqrt(M4_IN);
PWM_DUTY_CYCLE=PWM_DUTY_CYCLE+M4_DUTY_CYCLE

