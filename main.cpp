#include "mbed.h"
#include "SHA256.h"
#include "string"
#include "sstream"
#include "deque"
#include "vector"
//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

#define timingPin D4

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6          //0x08
#define L3Lpin D10           //0x10
#define L3Hpin D2          //0x20

#define PWMpin D9

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};


//Status LED
DigitalOut led1(LED1);
DigitalOut timePin(timingPin);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);
/************************************************************************/
//IO
RawSerial pc(SERIAL_TX, SERIAL_RX);
typedef struct {
  uint32_t type;           /* Tells printing function what type of message this is */
  int64_t int_package;    /* int message when that type is required */
  float    float_package;  /* float message when that type is required */
  float    float_package_2;
} mail_t;

Mail<mail_t, 16> mail_box;
Queue<void, 8> inCharQ;
/************************************************************************/
//Threads
Thread serial_out_thread(osPriorityNormal);
Thread serial_in_thread(osPriorityNormal);
Thread hashing_thread(osPriorityNormal);
Thread motor_cntrl_thread(osPriorityAboveNormal, 1024);
/************************************************************************/
//Mutexes
Mutex key_mut;
Mutex nonce_mut;
Mutex hash_count_mut;

Mutex Tune_mut;
Mutex max_speed_mut;
Mutex end_position_mut;
Mutex continuous_rotation_mut;
/************************************************************************/
//Hashing setup   
SHA256 h;
uint8_t sequence[]={0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,0x20,0x53,0x79,
0x73,0x74,0x65,0x6D,0x73,0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,0x20,0x61,
0x6E,0x64,0x20,0x64,0x6F,0x20,0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,0x74,
0x68,0x69,0x6E,0x67,0x73,0x21,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint64_t* key =(uint64_t*)((int)sequence + 48);
uint64_t* nonce =(uint64_t*)((int)sequence + 56);
uint8_t hash[32];    
int16_t hash_count = 0;
uint64_t new_key = 0;
uint64_t set_nonce = 0;
/************************************************************************/
//Timers & Tickers
Timer hash_rate_print;
Timer position_change_rate;
Timer velocity_measure;
/************************************************************************/
//Tune
vector<char> Tune;
/************************************************************************/
//Motor
PwmOut motor_pwm(PWMpin);
float max_speed = 0;
float end_position = 0;
float freq_rotation = 0;
bool continuous_rotation = false;

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed
//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards
//motor offset
int8_t orState = 0;    //Rotot offset at motor state 0
//motor state track
int16_t stateCount = 0;
/************************************************************************/
//Test Items
/************************************************************************/

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
    }
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}
 
void mailbox_add(uint32_t type, int64_t int_package = 0, float float_package = 0.0, float float_package_2 = 0.0){
    mail_t *mail = mail_box.alloc();
    
    mail->type = type;           
    mail->int_package = int_package;       
    mail->float_package = float_package;  
    mail->float_package_2 = float_package_2;
    
    mail_box.put(mail);

}
int8_t intStateOld = 0;
int8_t intState = 0;
void positionChange(){
    
    intState = readRotorState();
    if (intState != intStateOld) {
        if((intState > intStateOld && !(intState == 5 && intStateOld ==0)) || (intState == 0 && intStateOld == 5)){
            stateCount++;
        }else{
            stateCount--;
        }   
        motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        intStateOld = intState;
    }
    if(intState == 0){
        float period_us = velocity_measure.read_us();
        velocity_measure.reset();
        freq_rotation = 1000000 / period_us;
    }
    
    
} 



void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}



void set_key(string input){
    input.erase(0,1);
    
    if(input.length() != 16){
        mailbox_add(4);
    }else{
        stringstream ss;
        
        ss << hex << input;
        
        key_mut.lock();
        ss >> new_key;
        mailbox_add(3, new_key); 
        key_mut.unlock();
          
    }
}

void set_speed(string input){
    input.erase(0,1);
    stringstream ss;
    float speed;
    ss << input;
    ss >> speed;
    
    if(speed > 0 && speed < 1000){
        max_speed_mut.lock();
        max_speed = speed;
        mailbox_add(14, 0, max_speed);
        max_speed_mut.unlock();
    }else if(speed == 0){
        max_speed_mut.lock();
        max_speed = 1000;
        max_speed_mut.unlock();
        mailbox_add(15);
    }else{
        mailbox_add(4);
    }
}

void set_rotation(string input){
    input.erase(0,1);
    stringstream ss;
    float rotations;
    ss << input;
    ss >> rotations;
    
    
    core_util_critical_section_enter();
    int current_pos = stateCount;
    core_util_critical_section_exit();
    if(rotations == 0){
        continuous_rotation_mut.lock();
        continuous_rotation = true;
        continuous_rotation_mut.unlock();
    }else if(rotations > -1000 && rotations < 1000){
        continuous_rotation_mut.lock();
        continuous_rotation = false;
        continuous_rotation_mut.unlock();
        end_position_mut.lock();
        end_position = (rotations * 6) + current_pos;
        end_position_mut.unlock();
    }else{
        mailbox_add(4);
    }
    
}

void set_tune(string input){
    input.erase(0,1);
    
    char parsed_tune[128];
    int parsed_pointer = 0;
    int parsing_pointer = 0;

    while(parsing_pointer < input.size()){
        char temp = input[parsing_pointer++];
        if(input[parsing_pointer] == '#'){
            switch(temp){
                case 'A' :
                    temp = 'b';
                    break;
                case 'B' :
                    temp = 'C';
                    break;
                case 'C' :
                    temp = 'd';
                    break;
                case 'D' :
                    temp = 'e';
                    break;
                case 'E' :
                    temp = 'F';
                    break;
                case 'F' :
                    temp = 'g';
                    break;
                case 'G' :
                    temp = 'a';
                    break;
            }
            parsing_pointer++;
        }else if(input[parsing_pointer] == '^'){
            switch(temp){
                case 'A' :
                    temp = 'a';
                    break;
                case 'B' :
                    temp = 'b';
                    break;
                case 'C' :
                    temp = 'B';
                    break;
                case 'D' :
                    temp = 'd';
                    break;
                case 'E' :
                    temp = 'e';
                    break;
                case 'F' :
                    temp = 'E';
                    break;
                case 'G' :
                    temp = 'g';
                    break;
            }
            parsing_pointer++;
        }
        parsed_tune[parsed_pointer++] = temp;
        int repititions = input[parsing_pointer++] - '0';
        
        for(int i = 0; i < repititions - 1; i++){
            parsed_tune[parsed_pointer++] = temp;
        }
    }
    Tune_mut.lock();
    Tune.clear();
    for(int i = 0; i < parsed_pointer; i++){
        Tune.push_back(parsed_tune[i]);
    }
    Tune_mut.unlock();
}   
        

void tune_play(char note){
    float period = 1000;
    switch(note){
        case 'a':
            period = 150;
            break;
        case 'A':
            period = 142;
            break;
        case 'b':
            period = 134;
            break;
        case 'B':
            period = 127;
            break;
        case 'C':
            period = 119;
            break;
        case 'd':
            period = 113;
            break;
        case 'D':
            period = 106;
            break;
        case 'e':
            period = 100;
            break;
        case 'E':
            period = 94;
            break;
        case 'F':
            period = 89;
            break;
        case 'g':
            period = 84;
            break;
        case 'G':
            period = 80;
            break;
    }
    
    motor_pwm.period_us(period);
}


void parse_input(string input){
    switch(input[0]){
        case 'R' :
            set_rotation(input);
            break;
        case 'V' :
            set_speed(input);
            break;
        case 'K' :
            set_key(input);
            break;
        case 'T' :
            set_tune(input);
            break;
        default :
            mailbox_add(2);
    }
}


void message_input_thread(){
    pc.attach(&serialISR);
    string temp = "";
    while(1){
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
        if(newChar != '\r'){
            temp.push_back(newChar);
        }else{
            parse_input(temp);
            temp = "";
        }
    }
} 

void message_print_thread(){
    //Runs in its own thread, and is the only part of the program that can write to serial.
    while (true) {
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) {
            mail_t *mail = (mail_t*)evt.value.p;
            
            switch(mail->type){
                case 0: //Test message
                    pc.printf("%i\r\n", mail->int_package);
                    break;
                case 1: //Startup message
                    pc.printf("\r\n--------------- Starting Up ---------------\r\n");
                    break;
                case 2: //Input command error
                    pc.printf("Unknown Command\r\n");
                    break;
                case 3: //Prints the current key being used
                    key_mut.lock();
                    pc.printf("New Key Set: %llx\r\n", mail->int_package);
                    key_mut.unlock();
                    break;
                case 4: //Input command format error
                    pc.printf("Command Formatting Error\r\n");
                    break;
                case 5: //Prints the current nonce
                    nonce_mut.lock();
                    pc.printf("Nonce: %llx\r\n", set_nonce);
                    nonce_mut.unlock();
                    break;
                case 6: //Prints current hash count
                    ///pc.printf("Hash Rate: %i\r\n", mail->int_package);
                    pc.printf("%i\r\n", mail->int_package);
                    break;
                case 7:
                    pc.printf("Max Velocity: %d Target Position: %.3f Current Position: %.3f\r\n", mail->int_package, mail->float_package, mail->float_package_2);
                    break;
                case 8:
                    pc.printf("Rotation Torque: %.3f Velocity Torque: %d, Final Torque: %.3f\r\n", mail->float_package, mail->int_package, mail->float_package_2);
                    break;
                case 9:
                    pc.printf("Oscillation Period: %.3f\r\n", mail->float_package);
                    break; 
                case 10:
                    pc.printf("Position error %d Position Differential %.3f Position Integral %.3f\r\n", mail->int_package, mail->float_package, mail->float_package_2);
                    break;
                case 11:
                    pc.printf("Velocity torque: %.3f Max Velocity: %d Current Velocity: %.3f\r\n", mail->float_package, mail->int_package, mail->float_package_2);
                    break;
                case 12:
                    pc.printf("PWM: %.3f\r\n", mail->float_package);
                    break;
                case 14:
                    pc.printf("Max Speed Set At: %.3f\r\n", mail->float_package);
                    break;
                case 15:
                    pc.printf("Running At Max Speed\r\n");
                    break;
                default:
                    pc.printf("Unknown type message added to print queue. Please ensure corresponding case is in the message_print_thread function.\r\n");
            }
            mail_box.free(mail);
        }
        
    } 
}

void motorControlTick(){
    motor_cntrl_thread.signal_set(0x1);
}

void motor_controller_thread(){
    Ticker motorControlTicker;
    motorControlTicker.attach_us(&motorControlTick,100000);
    int iteration = 0;
    int last_pos = 0;
    int current_pos;
    float velocity = 0.0;
    
    float velocity_error = 0;
    float prev_velocity_error = 0;
    
    int prev_position_error = 0;
    int position_error = 0;
    float position_error_differential = 0;
    
    
    float k_pr = 0.4;//8;
    float k_dr = 0.3;
    float k_ir = 0.01;//.009;
    float max_position_integral = 20;
    float min_position_integral = -20;
    
    float k_ps = 0.045;
    //float k_is = 0.001;
    float k_is = 0.06;
    float max_velocity_integral = 20;
    float min_velocity_integral = -20;
    
    float velocity_error_integral = 0;
    float position_error_integral = 0;
    
    int tune_ptr = 0;
    
    while(1){
        motor_cntrl_thread.signal_wait(0x1);
        
        core_util_critical_section_enter();
        current_pos = stateCount;
        velocity = freq_rotation;
        core_util_critical_section_exit();
        
        
        if(velocity > 10000) velocity = 0;
        
        float time_stamp_read = position_change_rate.read();
        position_change_rate.reset();
        
        /**** Position Controller ****/
        end_position_mut.lock();
        position_error = end_position - current_pos;
        end_position_mut.unlock();
        position_error_differential = (position_error - prev_position_error) / time_stamp_read;
        prev_position_error = position_error;
        
        position_error_integral += position_error * time_stamp_read;
        if(position_error_integral > max_position_integral) position_error_integral = max_position_integral;
        if(position_error_integral < min_position_integral) position_error_integral = min_position_integral;
        
        float T_r = (k_pr*position_error) + (k_dr * position_error_differential) + (k_ir * position_error_integral);
        /*****************************/
        
        /**** Speed Controller Here ****/
        max_speed_mut.lock();
        velocity_error = max_speed - velocity;
        max_speed_mut.unlock();
        velocity_error_integral += velocity_error * time_stamp_read;
        if(velocity_error_integral > max_velocity_integral) velocity_error_integral = max_velocity_integral;
        if(velocity_error_integral < min_velocity_integral) velocity_error_integral = min_velocity_integral;
        
        
        float T_s = (k_ps*velocity_error) + (k_is*velocity_error_integral);
        T_s = current_pos > last_pos ? T_s : -T_s;
        /*****************************/
        
        float T;
        
        continuous_rotation_mut.lock();
        if(continuous_rotation){
            T = T_s;
        }else if(position_error_differential < 0){
            T = T_s > T_r ? T_r : T_s;
        }else{
            T = T_s < T_r ? T_r : T_s;
        }
        continuous_rotation_mut.unlock();
     
        float pwm;
        
        if(T < 0){
            core_util_critical_section_enter();
            lead = -2;
            core_util_critical_section_exit();
            pwm = -T;
        }else{
            core_util_critical_section_enter();
            lead = 2;
            core_util_critical_section_exit();
            pwm = T;
            
        }
        
        if(pwm > 1){
            pwm = 1;
        }
        
        if(position_error * position_error < 16){
            pwm = 0;
        }
  
        //mailbox_add(12, 0, position_error);
        if(iteration++ == 9){
            iteration = 0;
            //mailbox_add(7, max_speed, end_position, current_pos);
            //mailbox_add(8, T_s, T_r, T);
            //mailbox_add(9, 0, (max_index - min_index)/10);
            //mailbox_add(10, position_error, position_error_differential, position_error_integral);
            //mailbox_add(11, max_speed, T_s, velocity);
            //mailbox_add(12, temp_lead);
            
            hash_count_mut.lock();
            mailbox_add(6, hash_count);
            hash_count = 0;
            hash_count_mut.unlock();
            
            Tune_mut.lock();
            if(Tune.size() != 0){
                if(tune_ptr >= Tune.size()) tune_ptr = 0;
                tune_play(Tune[tune_ptr]);
                tune_ptr++;
            }
            Tune_mut.unlock();
        }
        last_pos = current_pos;
        motor_pwm.write(pwm);
    }
}

int main() {    
    /**** Motor Setup ****/
    motor_pwm.period(2e-3);
    motor_pwm.write(1);
    //Run the motor synchronisation
    orState = motorHome();
    I1.fall(&positionChange);
    I2.fall(&positionChange);
    I3.fall(&positionChange);
    I1.rise(&positionChange);
    I2.rise(&positionChange);
    I3.rise(&positionChange);
    
    /**** Initialise Threads ****/
    serial_out_thread.start(message_print_thread);
    serial_in_thread.start(message_input_thread);
    //hashing_thread.start(perform_hashing_thread);
    motor_cntrl_thread.start(motor_controller_thread);
    
        
    /**** Initialise Timers ****/   
    position_change_rate.start();
    velocity_measure.start();
    hash_rate_print.start();
    /**** Startup Message ****/
    mailbox_add(1);



    while(1){
        
        key_mut.lock();
        *key = new_key;
        key_mut.unlock();
        nonce_mut.lock();
        *nonce = set_nonce;
        nonce_mut.unlock();
        
        h.computeHash(hash, sequence, 64);
        hash_count_mut.lock();
        hash_count++;
        hash_count_mut.unlock();
        
        if(hash[0] == 0 && hash[1] == 0){
           // mailbox_add(5);
        }
        nonce_mut.lock();
        set_nonce++;        
        nonce_mut.unlock();
    };
}
