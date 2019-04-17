/*
 * arena_traversal.c
 *
 *  Created: 2/3/2019 9:52:43 AM
 *  Author: dell
*/
/* 
 * Team Id:           1873 
 * Author List:       VINIT NARAYAN JHA , MOHAMMAD AZHARUDDIN , SAURAV KUMAR , PALLAV BHALLA   
 * Filename:          arena_traversal_final.c       
 * Theme:             Thirsty Crow (TC)  
 * Functions:         adjust_in_mm(),uart_rx(),uart_tx(char data),decode_and_assign_vertices(),decode_into_index(char AR_cell[7]), assign_two_vertices(int decode_index,int decode_index_identifier), initialize_start_locations(), 
 *                     minDistance(int dist[],int sptSet[]), printPath(int parent[], int j, int case_flag), printSolution(int dist[], int n,int parent[],int finalNode, int case_flag),
 *       dijkstra(int graph[V][V], int src, int finalNode, int case_flag), move_after_LR_path_formed(), create_LR_path(int graph[V][V],int initial_vertex,int final_vertex)
 *       update_destination(int odd_even_value), select_shortest_path_and_move(int graph[V][V],int current_location), check_edge_case(int x_diff , int y_diff),
 *          update_axis(LR_path_counter), rotate_servo_and_transmit_after_run(),main(),move_forward_and_adjust()
 * Global Variables:  ADC_Conversion(unsigned char), ADC_Value, flag, Left_white_line,Center_white_line,Right_white_line, ShaftCountLeft,ShaftCountRight
 *                    vertices_array[57][2],decode_base_array[57][7],pitcher_axis[4],pitcher_cell[7],pebble_axis[4],pebble_cell[7],robot_start[8],pebble_to_transmit,pitcher_to_transmit,data,destination_name,update_destination_counter,
 *                    first_time_src_var,water_pitcher_visited_count,destination_axis,front_axis,current_location,pebble_1_vertex_1,pebble_1_vertex_2,pebble_2_vertex_1,pebble_2_vertex_2,pebble_3_vertex_1,pebble_3_vertex_2,final_vertex,
 *                    LR_path[V],vertex_path_array_counter,vertex_path_array[V],threshold,V,rotation_value,back_value,distances[8],change_of_models,pitcher_vertex_1,pitcher_vertex_2
*/  
 
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
unsigned char ADC_Conversion(unsigned char);  
unsigned char ADC_Value;                       
unsigned char Left_white_line = 0;              //Left_white_line: Takes input from left white line sensor
unsigned char Center_white_line = 0;            //Center_white_line: Takes input from centre white line sensor
unsigned char Right_white_line = 0;             //Right_white_line:Takes input from right white line sensor
volatile unsigned long int ShaftCountLeft = 0;  //ShaftCountLeft: to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //ShaftCountRight: to keep track of right position encoder
volatile unsigned int Degrees;                  //Degrees: its accepts angle in degrees for turning
#define threshold 0x10                          // threshold: At what value of white line sensor it will respond
#define line_thres 0x15                         // line_thres : At what value of white line sensor it will respond(it is used in 'adjust_in_mm') function for better sensitivity
#define V 54                                    // V:Total Number of vertex in Adjancy Matrix ( 54 * 54 )
#define rotation_value 160                     // rotation_value:The encoder is calibrated in such way such this rotation_value gives a rotation pf 120 degrees
#define back_value 500                         // back_value: The amount of back motion robot (through motors) will move
//These are used in uart_tx,uart_rx functions 
#define RX  (1<<4)                             
#define TX  (1<<3)
#define TE  (1<<5)
#define RE  (1<<7)
static int vertex_path_array[V];                 //vertex_path_array[V] : This array stores sequence of vertices that is decoded later on. 
char LR_path[V];                                 //LR_path[V] : This array contains sequence of 'L','R' that is used to move the robot 
int vertex_path_array_counter;                  //vertex_path_array_counter:acts as a counter for vertex_path_array[] 
int final_vertex;                               //final_vertex: It stores final 'vertex number' where it has to go
int pitcher_vertex_1,pitcher_vertex_2;          // pitcher_vertex_1 : It stores one of the two 'vertex number' provided to the location of pitcher according to the pitcher_cell and pitcher axis.
                                                // eg. if after concatenation of pitcher_cell = "16" and pitcher_axis = "2-2" ie. "16-2-2" , after searching in 'decode_base_array' and then in 'vertices_array' we get 14 , 20 as two 'vertex number' , one of them is assigned to pitcher_vertex_1/2.(two vertices of open sides of AR_object)
int pebble_1_vertex_1,pebble_1_vertex_2,pebble_2_vertex_1,pebble_2_vertex_2,pebble_3_vertex_1,pebble_3_vertex_2;  // pebble_i_vertex_j : (i = 1->3 , j = 1->2)(eg. pebble_1_vertex_1 / pebble_3_vertex_2)(two vertices of open sides of AR_object)
                                                // it stores one of the two 'vertex number' provided to the location of pebble according to the pebble_cell and pebble_axis.
                                                // eg. if after concatenation of pebble_cell1 = "16" and pebble_axis1 = "2-2" ie. "16-2-2" , after searching in 'decode_base_array' and then in 'vertices_array' we get 14 , 20 as two 'vertex number' , one of them is assigned to pebble_1_vertex_1.                                                                                 
int current_location;                          //current_location: stores the current location('vertex number') of the bot
int front_axis = 1;                            //front_axis: The axis in which the bot is currently looking at(its nose is pointing at)(eg,"1"/"2"/"3")(By default =1 because either at START-1/2 the front_axis = 1) 
int destination_axis;                          // destination_axis: This is the axis in which the AR_object is placed (the two sides/vertices in which it is open)(eg."16-1-1" the axis is "1-1" or "1" as stored in destination_axis)
int water_pitcher_visited_count = 0;            //water_pitcher_visited_count :counts how many times water pitcher has been visited ,when == 3 it means that the whole task is complete now.
int update_destination_counter = 0;             //update_destination_counter :updates the odd even value so that a alternate sequence is obtained for going to pebble and pitcher alternately
int  first_time_src_var = 1;                    //first_time_src_var: It is used to add initial vertex to vertex_array_path[] array. 
char destination_name;                          // Stores the destination name ;either 'W'=water pitcher , 'P' = pebble
volatile unsigned char data;                    // stores the data during transmission through xbee
  
// All of below variables are declared with '\0' because in the main function these variables are provided as input from python script to Atmega board(Robot)  
char robot_start[8] =  {"\0"};               // robot_start : This stores the start location (START-1/2)      
char pebble_cell1[7] = {"\0"};              // pebble_cell1 : This stores the cell number of pebble 1
char pebble_axis1[4] =  {"\0"};             // pebble_axis1 : This stores the axis of pebble 1
char pebble_cell2[7] = {"\0"};              // pebble_cell2 : This stores the cell number of pebble 2
char pebble_axis2[4] =  {"\0"};             // pebble_axis2 : This stores the axis of pebble 2
char pebble_cell3[7] = {"\0"};              // pebble_cell3 : This stores the cell number of pebble 3
char pebble_axis3[4] =  {"\0"};          // pebble_axis3 : This stores the axis of pebble 3
char pitcher_cell[7] = {"\0"};             // pitcher_cell : This stores the cell number of pitcher
char pitcher_axis[4] = {"\0"};             // pitcher_axis : This stores the axis of pitcher
char water_pitcher_string[13] = {"\0"};        // water_pitcher_string : This stores the string 'Water Pitcher'
char pebble1_string[7] = {"\0"};      // pebble1_string : This stores the string 'Pebble1'
char pebble2_string[7] = {"\0"};      // pebble2_string : This stores the string 'Pebble2'
char pebble3_string[7] = {"\0"};       // pebble3_string : This stores the string 'Pebble3'
char update_animation;                          //update_animation: It stores the charector indicating that at which pebble and pitcher the robot is going to next and helps in changing that particular AR object :
                                                //(the charectors are named 'a','b','c','d' rather that 'p1'(pebble-1) and like these because storing 'p1' requires string and copying data ie. using strcpy() is creating lag due to slow input buffer)
                                                // when update_animation == a       -->      pebble_1_vertex_1/pebble_1_vertex_2
                                                // when update_animation == b       -->      pebble_2_vertex_1/pebble_2_vertex_2
                                                // when update_animation == c       -->      pebble_3_vertex_1/pebble_3_vertex_2
                                                // when update_animation == d       -->      pitcher_vertex_1/pitcher_vertex_2
int water_pitcher_animation_count = 1;         // water_pitcher_animation_count: This variable stores the count ie. the number of times the the level of water has been raised.
                                               // when, water_pitcher_animation_count == 1       -->        initial rise in water level
                                               //       water_pitcher_animation_count == 2       -->        middle rise in water level
                                               //       water_pitcher_animation_count == 3       -->        full rise in water level
int distances[8];                            // distances[8] : This array is used to calculate shortest distance. It stores distances between 'current_location' and destination vertices.
char decode_base_array[57][7] = {                             // decode_base_array[57][7] : It stores string patterns made of concatenated pebble/pitcher cell and pebble/pitcher axis, It is used to find concatenated string eg,"16-1-1" and return the index eg,18 here.
                              "5-1-1","5-2-2","5-3-3",        // "5-1-1" means the two vertices of "5"th cell and "1-1" axis , and later it returns the its index that is 0 , for resolving those two 'vertex number'
                              "10-1-1","10-2-2","10-3-3",
         "15-1-1","15-2-2","15-3-3",
                              "2-1-1","2-2-2","2-3-3",
                              "6-1-1","6-2-2","6-3-3",
         "11-1-1","11-2-2","11-3-3",
  "16-1-1","16-2-2","16-3-3",
  "1-1-1","1-2-2","1-3-3",
                                 "3-1-1","3-2-2","3-3-3",
  "7-1-1","7-2-2","7-3-3",
                                 "12-1-1","12-2-2","12-3-3",
                                 "17-1-1","17-2-2","17-3-3",
                                 "4-1-1","4-2-2","4-3-3",
                                 "8-1-1","8-2-2","8-3-3",
                                 "13-1-1","13-2-2","13-3-3",
  "18-1-1","18-2-2","18-3-3",
  "9-1-1","9-2-2","9-3-3",
  "14-1-1","14-2-2","14-3-3",
  "19-1-1","19-2-2","19-3-3"
  };
int vertices_array[57][2] = {                                //vertices_array[57][2] : It stores the two vertices according to index returned on searching decode_base_array[57][7] for a string like "16-1-1", These are the two vertices open (AR_object) according to a cell number and pebble/pitcher axis.
                             {0,12},{3,8},{4,7},             //the numbers present inside these {} sets are referred as 'vertex number' 
                             {1,13},{4,9},{5,8},
                             {2,14},{5,10},{6,9},
                             {7,22},{11,17},{12,16},
                             {8,23},{12,18},{13,17},
                             {9,24},{13,19},{14,18},
                             {10,25},{14,20},{15,19},
                             {16,33},{21,28},{22,27},
                             {17,34},{22,29},{23,28},
                             {18,35},{23,30},{24,29},
                             {19,36},{12,31},{25,30},
                             {20,37},{25,32},{26,31},
                             {28,43},{33,39},{34,38},
                             {29,44},{34,40},{35,39},
                             {30,45},{35,41},{36,40},
                             {31,46},{36,42},{37,41},
                             {39,51},{43,48},{44,47},
                             {40,52},{44,49},{45,48},
                             {41,53},{45,50},{46,49}
                             };
/* 'vertex number' concept :
 
 'concatenated string of pebble/pitcher cell and pitcher/pebble axis'   'vertex number'(vertex 1 , vertex 2)
------------------------------------------------------------------------------------------------------------------- 
 "5-1-1"                                                                        0,12
 "5-2-2"                                                                         3,8
 "5-3-3"                                                                         4,7
 "10-1-1"                                                                        1,13
 "10-2-2"                                                                        4,9
 "10-3-3"                                                                        5,8   
 "15-1-1"                                                                        2,14     
 "15-2-2"                                                                        5,10
 "15-3-3"                                                                        6,9
 "2-1-1"                   7,22
 "2-2-2"                                                                         11,17
 "2-3-3"                                                                         12,16 
 "6-1-1"                                                                         8,23
 "6-2-2"                                                                         12,18
 "6-3-3"                                                                         13,17
 "11-1-1"                                                                        9,24
 "11-2-2"                                                                        13,19
 "11-3-3"                                                                        14,18
 "16-1-1"                                                                        10,25
 "16-2-2"                                                                        14,20
 "16-3-3"                                                                        15,19
 "1-1-1"                                                                         16,33
 "1-2-2"                                                                         21,28
 "1-3-3"                                                                         22,27
 "3-1-1"                                                                         17,34
 "3-2-2"                                                                         22,29
 "3-3-3"                                                                         23,28
 "7-1-1"                                                                         18,35
 "7-2-2"                                                                         23,30
 "7-3-3"                                                                         24,29
 "12-1-1"                                                                        19,36
 "12-2-2"                                                                        12,31
 "12-3-3"                  25,30
 "17-1-1"                     20,37
 "17-2-2"                  25,32
 "17-3-3"                  26,31
 "4-1-1"                   28,43
 "4-2-2"                   33,39
 "4-3-3"                                                                         34,38
 "8-1-1"                                                                         29,44
 "8-2-2                                                                          34,40
 "8-3-3"                      35,39    
 "13-1-1"                                                                        30,45
 "13-2-2"                                                                        35,41
 "13-3-3"                                                                        36,40
 "18-1-1"                                                                        31,46
 "18-2-2"                   36,42
 "18-3-3"                        37,41  
 "9-1-1"                      39,51
 "9-2-2"                   43,48
 "9-3-3"                   44,47   
 "14-1-1"                  40,52
 "14-2-2"                  44,49 
 "14-3-3"                  45,48 
 "19-1-1"                  41,53
 "19-2-2"                  45,50
 "19-3-3"                  46,49
*/
/* 
 * Function Name: uart_tx   
 * Input:   data -> The value to be sent to python script is stored in 'data' variable , in this function 'data' is assigned to UDR0 register so it can be transmitted 
 *                back to python script.      
 * Output:   Does not returns anything . Assigns value in 'data' variable to UDR0 register for transmitting back to python script.  
 * Logic:         A halt is created using a while loop that stops when 5th bit of data stored in UCSR0A register becomes equal to 1. After that 'data' variable is assigned
 *                to UDR0 register. If UDRE0(5th bit of UCSR0A) is one, the buffer is empty, and therefore ready to be written. The UDRE0 Flag can generate a
 *   Data Register Empty interrupt . Demo: UCSR0A (00010000) & TE (00010000) != 0 ie. 1 value so , while(!1) = while(0) , so loop ends and UDR0 is assigned 'data'
 *                and is successfully sent to python script.      
 * Example Call:  uart_tx(data) 
*/ 
  
void uart_tx(char data)
{
 while(!(UCSR0A & TE));                        //waiting to transmit
 UDR0 = data;
}
/* 
 * Function Name: ISR   
 * Input:   USART0_RX_vect(It refers to the vector or memory which will be executed when the there is an Interrupt ).      
 * Output:   Does not returns anything . Assigns value in 'data' variable to UDR0 register for transmitting back to python script.  
 * Logic:         It is a USART0 receive complete interrupt.This Interrupt is executed whenever there is a value recieved and that value will be given to 
 *                the data variable.      
 * Example Call:  This function is called internally whenever there is an interrupt, ISR(USART0_RX_vect). 
*/
ISR(USART0_RX_vect)              // This function is called internally whenever there is an interrupt.
{
 data = UDR0;
}
/* 
 * Function Name: uart_rx   
 * Input:   None. It is called automatically by 'main' function to check whether UDR0 register received any data.       
 * Output:   Returns value stored in UDR0 register to the 'main' function.  
 * Logic:         A halt is created using a while loop that stops when 7th bit of data stored in UCSR0A register becomes equal to 1. After that value stored in UDR0 register 
 *                is returned to 'main' function. If RXC0(7th bit of UCSR0A) is set then there is unread data in the receive buffer and cleared when the receive
 *                buffer is empty . Demo: UCSR0A (10000000) & RE (10000000) != 0 ie. 1 value so , while(!1) = while(0) , so loop ends and UDR0 is returned to 'main' function.      
 * Example Call:  uart_rx() 
*/ 
char uart_rx()
{
 while(!(UCSR0A & RE));                        //Waiting to receive , this loop will end when 5th bit of UCSR0A = 1 and then value stored in UDR0 is returned.
 return UDR0;
}
/* 
 * Function Name: uart0_init   
 * Input:   None  
 * Output:   None   
 * Logic:         It initializes the registers for communication.
 * Example Call:  uart0_init()  
*/
void uart0_init()
{
 UCSR0B = 0x00;                            //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F;                             //9600BPS at 14745600Hz
 UBRR0H = 0x00;
 UCSR0B = 0x98;
 //UCSR0C = 3<<1;                            //setting 8-bit character and 1 stop bit
 //UCSR0B = RX | TX;
}
/* 
 * Function Name: buzzer_pin_config()                          
 * Input:    None     
 * Output:    Does not returns anything .  
 * Logic:         Initializes the direction of pins to be used as(input or output) and giving the initial value to the port pins.      
 * Example Call:  buzzer_pin_config () 
*/
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;  //Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;  //Setting PORTC 3 logic low to turnoff buzzer
}
/* 
 * Function Name: buzzer_on()        
 * Input:    None     
 * Output:    Does not returns anything .  
 * Logic:         Giving high value to the pin where buzzer pin is connected      
 * Example Call:  buzzer_on () 
*/
void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}
/* 
 * Function Name: buzzer_off()                                  
 * Input:    None     
 * Output:    Does not returns anything .  
 * Logic:         Giving low value to the pin where buzzer pin is connected      
 * Example Call:  buzzer_off () 
*/
void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}
/* 
 * Function Name: adc_pin_config()                           
 * Input:    None     
 * Output:    Does not returns anything .  
 * Logic:         Initializes the direction of pins to be used as(input or output) and giving the initial value to the port pins.      
 * Example Call:  adc_pin_config () 
*/
void adc_pin_config (void)
{
 DDRF = 0x00;
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}
//initializes the ADC registers for its operation
void adc_init()
{
 ADCSRA = 0x00;
 ADCSRB = 0x00;  //MUX5 = 0
 ADMUX = 0x20;  //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
 ACSR = 0x80;
 ADCSRA = 0x86;  //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}
//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{
 unsigned char a;
 if(Ch>7)
 {
  ADCSRB = 0x08;
 }
 Ch = Ch & 0x07;
 ADMUX= 0x20| Ch;
 ADCSRA = ADCSRA | 0x40;  //Set start conversion bit
 while((ADCSRA&0x10)==0); //Wait for conversion to complete
 a=ADCH;
 ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 ADCSRB = 0x00;
 return a;
}
/* 
 * Function Name: timer5_init()   
 * Input:    None(No input as it is used for initialization of timer)     
 * Output:    Does not returns anything .  
 * Logic:         Initializes all the registers which are required for proper operations of the timer including the mode of 
 *      operation(here PWM mode), counter value, compare value to which the counter will compare with to perform
 *      the interrupt.       
 * Example Call:  timer5_init() 
*/
void timer5_init()
{
 TCCR5B = 0x00;     //Stop
 TCNT5H = 0xFF;     //Counter higher 8-bit value to which OCR5xH value is compared with
 TCNT5L = 0x01;     //Counter lower 8-bit value to which OCR5xH value is compared with
 OCR5AH = 0x00;     //Output compare register high value for Left Motor
 OCR5AL = 0xFF;     //Output compare register low value for Left Motor
 OCR5BH = 0x00;     //Output compare register high value for Right Motor
 OCR5BL = 0xFF;     //Output compare register low value for Right Motor
 TCCR5A = 0xA9;     /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
          For Overriding normal port functionality to OCRnA outputs.
           {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
 
 TCCR5B = 0x0B;     //WGM52=1; CS52=0, CS51=1, CS50=1 (Prescaler=64)
}
/* 
 * Function Name: velocity()  
 * Input:    None     
 * Output:    Does not returns anything .  
 * Logic:         This is the function which gives the value to the two registers of the timers which are the values that are compared to the counter value
 *      of the  timers.Hence this decides the velocity of the motors as lower value in the register takes less time when compared with the counter
 *                value and hence gives less speed.
 * Example Call:  velocity (255, 255)
*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
 OCR5AL = left_motor;
 OCR5BL = right_motor;
}
/* 
 * Function Name: magnet_pin_config()   
 * Input:    None     
 * Output:    Does not returns anything .  
 * Logic:         Initializes the direction of pins to be used as(input or output) and giving the initial value to the port pins.      
 * Example Call:  magnet_pin_config() 
*/
void magnet_pin_config()
{
 DDRH = 0x01;    //Setting PH0 pin as output for magnet operation.
 PORTH = 0x00;    //Initially giving 0 to the pins so that magnet discharged initially.
}
/* 
 * Function Name: motor_pin_config()   
 * Input:    None     
 * Output:    Does not returns anything .  
 * Logic:         Initializes the direction of pins to be used as(input or output) and giving the initial value to the port pins.      
 * Example Call:  motor_pin_config() 
*/
void motor_pin_config()
{
 DDRA = 0x0F;     //Setting PA0, PA1, PA2, PA3 pins as output for direction control of motor
 PORTA = 0x00;     //Initially all pins are given logic 0. Hence both motors are at rest.
 DDRL = 0x18;     //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = 0x18;        //PL3 and PL4 pins are for velocity control using PWM.
}
/* 
 * Function Name: magnet_on()   
 * Input:    None     
 * Output:    Does not returns anything .  
 * Logic:         Magnet gets charged as current flows through it by giving HIGH logic to pin it is connected.      
 * Example Call:  magnet_on() 
*/
void magnet_on()
{
 PORTH = 0x01;    //Magnet charged as logic 1 is given to the pin where magnet is connected.
}
/* 
 * Function Name: magnet_off()  
 * Input:    None     
 * Output:    Does not returns anything .  
 * Logic:         Magnet gets discharged as current does not flows through it by giving LOW logic to pin it is connected.      
 * Example Call:  magnet_off() 
*/
void magnet_off()
{
 PORTH = 0x00;    //Magnet discharged as logic 0 is given to the pin where magnet is connected.
}
/* 
 * Function Name: forward()   
 * Input:    None(No input as it is used for initialization of timer)     
 * Output:    Does not returns anything .  
 * Logic:         Logic HIGH is given to PA2 pin to move the left motor forward and logic HIGH is given to pin PA1 to move right motor forward.      
 * Example Call:  forward() 
*/
void forward()    //both wheels forward
{
 PORTA = 0x06;
}
/* 
 * Function Name: backward()   
 * Input:    None     
 * Output:    Does not returns anything .  
 * Logic:         Logic HIGH is given to PA3 pin to move the left motor backward and logic HIGH is given to pin PA0 to move right motor backward.      
 * Example Call:  backward()
*/
void backward()   //both wheels backward
{
 PORTA = 0x09;
}
/* 
 * Function Name: left()   
 * Input:    None     
 * Output:    Does not returns anything .  
 * Logic:         Logic HIGH is given to PA3 pin to move the left motor backward and logic HIGH is given to pin PA1 to move right motor forward.      
 * Example Call:  left() 
*/
void left()       //Right wheel forward, Left wheel backward
{
 PORTA = 0x05;
}
/* 
 * Function Name: right()   
 * Input:    None     
 * Output:    Does not returns anything .  
 * Logic:         Logic HIGH is given to PA2 pin to move the left motor forward and logic HIGH is given to pin PA0 to move right motor backward.      
 * Example Call:  right() 
*/
void right()          //Left wheel forward, Right wheel backward
{
 PORTA = 0x0A;
}
 /* 
 * Function Name: stop()   
 * Input:    None     
 * Output:    Does not returns anything .  
 * Logic:         The potential difference to the motor pins has been made 0 as all the pins of port A has been made 0.Hence the motor stops.       
 * Example Call:  stop() 
*/ 
void stop()           //stop
{
 PORTA = 0x00;    
}
//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}
//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}
void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt
}
void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt
}
//ISR for right position encoder
ISR(INT5_vect)
{
 ShaftCountRight++;  //increment right shaft position count
}
//ISR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}
/*
Function_name: angle_rotate
Input:  Input to this function is amount of Degrees we want to rotate the bot.
        the degree should be in integer
Output: It returns nothing. The work of the function is to rotate the bot with specified degree and the stop the bot.
        It checks the following condition if((ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))
 and then any one of the condition becomes true it breaks from the while loop and then stop the bot.
Logic:  This function takes the angle in Degrees and then convert it to ReqdShaftCountInt and then it initializes two variables ShaftCountRight and ShaftCountLeft
        and then rotate until any one of the condition(ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt) becomes 
 true and then it breaks from the while loop and then stop the bot.
Example call: angle_rotate(Degrees);
*/
int angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;
 ReqdShaftCount = (float) Degrees/ 0.3; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0;
 ShaftCountLeft = 0;
 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))
  {
         break;
   }
 }
 stop(); //Stop robot
 return 0;
}
void left_degrees(unsigned int Degrees)
{
 left(); //Turn left
 angle_rotate(Degrees);
}
int right_degrees(unsigned int Degrees)
{
 right(); //Turn right
 angle_rotate(Degrees);
 return 0;
}
/*
Function_name: linear_distance_mm
Input:  the input to the function is amount of distance in millimeters to traverse the bot linearly 
Output: It returns nothing. The work of the function is to traverse the bot linearly with specified distance and then stop the bot.
        It checks the following condition if(ShaftCountRight > ReqdShaftCountInt)
        and then if the condition becomes true it breaks from the while loop and then stop the bot.
Logic:  This function takes the distance in millimeters and then convert it to ReqdShaftCountInt and then it initializes one variable ShaftCountRight 
        and then traverse linearly  until  the condition (ShaftCountRight > ReqdShaftCountInt) becomes
        true and then it breaks from the while loop and then stop the bot.
Example call: linear_distance_mm(DistanceInMM);
*/
void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;
 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
 ShaftCountRight = 0;
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt)
  {
   break;
  }
 }
 stop();  
}
/*
Function_name: forward_mm
Input:  the input to the function is amount of distance in millimeters to traverse the bot in forward direction 
Output: It moves the bot in forward direction with the specified velocity.
Logic:  This function takes the distance in millimeters and then call the linear_distance_mm(DistanceInMM) and move the bot in forward direction with specified velocity
        Every time it calls the linear_distance_mm(DistanceInMM) to check the condition to stop.
Example call: forward_mm(80);
*/
void forward_mm(unsigned int DistanceInMM)
{
 velocity(150,153.7);
 forward();
 linear_distance_mm(DistanceInMM);
}
/*
Function_name: back_mm
Input:  the input to the function is amount of distance in millimeters to traverse the bot in backward direction
Output: It moves the bot in backward direction with the specified velocity.
Logic:  This function takes the distance in millimeters and then call the linear_distance_mm(DistanceInMM) and move the bot in backward direction with specified velocity
        Every time it calls the linear_distance_mm(DistanceInMM) to check the condition to stop.
Example call: back_mm(back_value);
*/
void back_mm(unsigned int DistanceInMM)
{
 velocity(150,153.7);
 backward();
 linear_distance_mm(DistanceInMM);
}
/*
Function_name: follow(void)
Input:  The input is void
Output: It returns nothing.Just move the bot in the specific direction according to the specified conditions.
Logic:  It takes the value of the 3 line sensors and then according to the specified condition it will move.
        The threshold value on which the checking is done is 100.This funtion is used only when following the 
        bot is following straight line.
 The condition are following:
        1.if the sensor detects WBW(white,black,while) as left sensor,middle sensor, right sensor respectively
          it will move forward.
        2.if the sensor detects WWB(white,white,black) as left sensor,middle sensor, right sensor respectively
          it will move right to shift to to the line.
        3.if the sensor detects BWW(black,white,white) as left sensor,middle sensor, right sensor respectively
          it will move left to shift to the line.          
Example call: follow();
*/
void follow(void)
{ 
 if(Center_white_line>0x64 && Left_white_line<0x64 && Right_white_line<0x64)//WBW
 {
     velocity(130,117.53);               // 130,117.53
     forward();
 }
 
 if(Center_white_line<0x64 && Left_white_line<0x64 && Right_white_line>0x64)//WWB
 {
     right();
 }
 
 if(Center_white_line<0x64 && Left_white_line>0x64 && Right_white_line<0x64)//BWW
 {
     left();
 }
}
void init_devices (void)          //function to initialize all devices
{
 cli();
 magnet_pin_config();
 servo1_pin_config();
 uart0_init();
 buzzer_pin_config ();
 adc_pin_config ();  
 timer1_init();                    //disable all interrupts
 timer5_init();
 adc_init();
 left_encoder_pin_config();
 right_encoder_pin_config();
 left_position_encoder_interrupt_init ();
 right_position_encoder_interrupt_init ();
 sei();                                  //re-enable interrupts
}
/* 
 * Function Name: decode_into_index    
 * Input:   AR_cell[7] -> It is an charector array that recieves the concatenated string (eg: "16-1-1") and helps in identifying the index of the string.                
 * Output:   Returns index of the string found in 'decode_base_array[57][7]'.
 *                i->  index of the concatenated string eg."16-1-1" found , when searched in "decode_base_array[57][7]".                
 * Logic:         This function searches and compares the concatenated string from every string in "decode_base_array[57][7]" and returns the index of the string found.
 *                In the 'update_destination' function this index is used to get vertex numbers of the two vertices pointed by the concatenated string.
 *                Eg. If concatenated string is "16-1-1" , it returns 18 (index of "16-1-1" is 18 in "decode_base_array[57][7]").      
 * Example Call: decode_into_index(pebble_cell) 
*/ 
int decode_into_index(char AR_cell[7]){
for(int i = 0; i < 57; i++){
    if(strcmp(decode_base_array[i],AR_cell) == 0){
        return i;                                        // returning the index of the concatenated string 
    }
  }
  return 0;
}
/* 
 * Function Name: assign_two_vertices    
 * Input:   decode_index -> It is the index of the concatenated string returned by "decode_into_index(char AR_cell[7])" function and 
 *                is used to assign vertex 1 and vertex 2 (the two open sides of the AR_object ie. two vertex)
 *          decode_index_identifier -> It helps to identify that weather pitcher or pebble has to be assigned the vertices. 
 * Output:   int to inform the caller that the program exited correctly. It initializes vertex 1 and vertex 2 to respective pebble and pitcher.                  
 * Logic:    This function first checks the value of 'decode_index_identifier' and according to that assigns pitcher or pebble's two vertices.
 *           Every AR_object has two sides open so for each pebble and pitcher two vertex 1 and 2 are assigned based on 'decode_index'. The rules
 *           for 'decode_index_identifier' are :
 *           when, decode_index_identifier == 0 then pitcher_vertex_1 and pitcher_vertex_2
 *           when, decode_index_identifier == 1 then pebble_1_vertex_1 and pebble_1_vertex_2
 *           when, decode_index_identifier == 2 then pebble_2_vertex_1 and pebble_2_vertex_2
 *           when, decode_index_identifier == 3 then pebble_3_vertex_1 and pebble_3_vertex_2
 * Example Call: assign_two_vertices(decode_index_of_pitcher,0);
*/ 
 
int assign_two_vertices(int decode_index,int decode_index_identifier){
    if(decode_index_identifier == 0){                         // pitcher case
        pitcher_vertex_1 = vertices_array[decode_index][0];
        pitcher_vertex_2 = vertices_array[decode_index][1];
    } 
    else if(decode_index_identifier == 1){                  //pebble 1 case
    pebble_1_vertex_1 = vertices_array[decode_index][0];
    pebble_1_vertex_2 = vertices_array[decode_index][1];
    } 
    else if(decode_index_identifier == 2){                 //pebble 2 case
    pebble_2_vertex_1 = vertices_array[decode_index][0];
    pebble_2_vertex_2 = vertices_array[decode_index][1];
    }
    else if(decode_index_identifier == 3){                 //pebble 3 case
    pebble_3_vertex_1 = vertices_array[decode_index][0];
    pebble_3_vertex_2 = vertices_array[decode_index][1];
    }
    else{
    // Error : Error in ' decode_index_identifier 'value         
    }
        return 0;
}
/* 
 * Function Name: initialize_start_locations    
 * Input:   None
 * Output:   int to inform the caller that the program exited correctly. It only sets current location to 'vertex number' :  1 or 52.                  
 * Logic:         This function first checks whether 'robot_start' is equal to "START-1"/"START-2" and then the value of current_location is updated 
 *                to 'vertex number' :  1 or 52.           
 * Example Call: initialize_start_locations() 
*/
int initialize_start_locations()
{
 if(strcmp(robot_start , "START-1") == 0){
      current_location = 1;                             // START-1 ('vertex number' = 1)
   }
   else{
    current_location = 52;                            // START-2 ('vertex number' = 52)
   }
 return 0;
}
/* 
 * Function Name: minDistance    
 * Input:   dist[] -> This array stores the distance of a node(vertex) from source vertex. Eg , 'i' represent vertex then dist[i] represent distance of node from source/current vertex.   
 *                sptSet[] -> This array contains shortest path set for nodes from current vertex.
 *   Output: min_index -> It returns minimum index by calculating when distance of the particular node <= INT_MAX      
 * Logic:         It first assigns min as a sentinel by assigning a maximum value INT_MAX . After that a loop is used to traverse all vertices and checks that distance of node is less than equal to min
 *                and sptSet[] for that index/vertex is 0 then min is assigned distance from the current location and min_index is assigned the index at that moment.
 * Example Call:  minDistance(dist, sptSet) 
*/
int minDistance(int dist[],int sptSet[]) 
{ 
 int min = INT_MAX, min_index; 
 for (int v = 0; v < V; v++) 
 if ((sptSet[v] == 0) && (dist[v] <= min)) 
 {min = dist[v], min_index = v;} 
 
 return min_index; 
}
/* 
 * Function Name: printPath    
 * Input:   parent[] -> This array stores the parent of the element(according to index), ie. the index represent the vertex and the value at that index represent its parent.                   
 *                case_flag -> This variable helps the function to identify when it has to return the distance from one vertex to other and when to create
 *                j -> It contains the value of index (of all 54 vertices) ,one by one at a time.  
 * Output:   int to inform the caller that the program exited correctly . It calls 'printPath' recursively and creates 'vertex_path_array[vertex_path_array_counter]'.     
 * Logic:         First it checks that distance from parent node to itself and returns. Then it calls printPath function recursively. After that it checks that case_flag to be 2,
 *                then it uses a variable ' first_time_src_var ' and checks to be 1 , if 1 then it fills the 'vertex_path_array[]' with the source or starting index                                                             
 *                After that vertex_path_array[] is filled with the values of j that is extracted using parent array.  
 * Example Call:  printPath(parent, i, case_flag) 
*/ 
  
void printPath(int parent[], int j, int case_flag) 
{ 
  if(parent[j] == - 1)
 return; 
 
 printPath(parent, parent[j], case_flag);
 
    if(case_flag == 2)
    {
    if(first_time_src_var == 1){                       // adding the source node to the path
            vertex_path_array_counter++;
            vertex_path_array[vertex_path_array_counter] = current_location;
            first_time_src_var++;                     // increasing the value of 'first_time_src_var' so that next time this 'if' statement does not take place
    }
     vertex_path_array_counter++;
     vertex_path_array[vertex_path_array_counter] = j;
    }
}
/* 
 * Function Name: printSolution    
 * Input:   dist[] -> This array stores the distance of a node(vertex) from source vertex.
 *                n -> It is the total number of vertices in graph/map/arena ie.54
 *                parent[] -> This array stores the parent of the element(according to index), ie. the index represent the vertex and the value at that index represent its parent.                   
 *                finalNode -> It is the final vertex whether the robot has to go.
 *                case_flag -> This variable helps the function to identify when it has to return the distance from one vertex to other and when to create
 *   the 'vertex_path_array[]'. If case_flag = 1 then it returns the distance from src to finalNode ; when case_flag = 2 then
 *   it creates a vertex_path_array[] that contain sequence of vertices ('vertex numbers')from current location to final location.
 * Output:   It returns the distance from current location to final location.
 *                dist[i] -> This variable contains the distance of every vertex from source vertex , ie. if index 'i' represent the vertex then dist[i] represent its distance from source vertex. 
 * Logic:         This function traverses through every vertex and finds the finalNode and checks case_flag == 1 ie. distance has to be returned from function then dist[i] is returned.
 *                After this 'printPath(parent, i, case_flag)' is called repeatedly with each index values for creating the 'vertex_path_array[]' that stores sequence of vertices from source to
 *                finalNode.                                                   
 * Example Call:  printSolution(dist, V, parent,finalNode,case_flag) 
*/ 
    
int printSolution(int dist[], int n,int parent[],int finalNode, int case_flag) 
{ 
   for (int i = 0; i < V; i++) 
 {   
     if(i == finalNode)                
     {
      if(case_flag == 1){
         return dist[i];
      } 
        printPath(parent, i, case_flag);
     }
 } 
    return 0;
}
/* 
 * Function Name: dijkstra    
 * Input:   graph[V][V] -> It is the 54*54 adjancy matrix (graph representation) that stores nodes and how they are connected with each other.
 *                src -> This variable stores the current location of the robot.
 *                finalNode -> This variable stores the final vertex or the location where the robot has to go.
 *                case_flag -> This variable helps the function to identify when it has to return the distance from one vertex to other and when to create 
 * the 'vertex_path_array[]'. If case_flag = 1 then Dijsktra returns the distance from src to finalNode ; when case_flag = 2 then 
 * Dijkstra function creates a vertex_path_array[] that contain sequence of vertices ('vertex numbers')from current location to final location.                   
 * Output:        It returns the distance from current location to final location.
 *                distance_from_current_to_final -> This variable stores the distance from current location(src) to final location(finalNode) , which is further used to calculate 
 *                the closest vertex to current location(in 'select_shortest_path_and_move()').              
 * Logic:         It first declares set of variables and then runs a for loop ie. traverses through all the vertices and assign parent of all the nodes as -1
 *                It also assigns distance of all nodes from src node as INT_MAX (maximum value), and also assigns Shortest path set (sptSet[i]) for all the indexes(vertices)
 *                as 0. After that it assigns distance of src from itself as 0. After that it traverses from 0 to 52 vertex and for each index  'minDistance(dist, sptSet)'
 *                is called and stored in 'u' variable and also assigns sptSet[u] = 1. After that a nested for loop is run from 0 to 53 and it checks 
 *                that (sptSet[v] && graph[u][v] && dist[u] + graph[u][v] < dist[v]) is false or not , if it is false then dist[current index] is made equal to u (previous). After that  
 *                dist[v] is made equal to addition of dist[u] and graph[u][v] . After that 'printSolution' function is called further calls different functions to calculate distance or
 *                create vertex_path_array[].           
 * Example Call:  dijkstra(graph,current_location,v1,1) (The last argument(case_flag) can be 1 or 2 based on we want distance from 'dijkstra' function or we want to generate LR_path) 
*/ 
  
int dijkstra(int graph[V][V], int src, int finalNode, int case_flag) 
{   
    vertex_path_array_counter = -1; 
    first_time_src_var = 1;
 int distance_from_current_to_final;
 int dist[V]; 
 int sptSet[V]; 
 int parent[V]; 
 
 for (int i = 0; i < V; i++) 
 { 
 parent[src] = -1; 
 dist[i] = INT_MAX; 
 sptSet[i] = 0; 
 } 
  dist[src] = 0; 
  
  for (int count = 0; count < V - 1; count++) 
 { 
   int u = minDistance(dist, sptSet); 
      sptSet[u] = 1; 
  
  for (int v = 0; v < V; v++) 
 { if (!sptSet[v] && graph[u][v] && dist[u] + graph[u][v] < dist[v]) 
 { 
 parent[v] = u; 
 dist[v] = dist[u] + graph[u][v]; 
 } 
 }
 
 } 
 distance_from_current_to_final = printSolution(dist, V, parent,finalNode,case_flag); 
    return distance_from_current_to_final;
}
 /* 
 * Function Name: move_after_LR_path_formed    
 * Input:         None              
 * Output:        int to inform the caller that the program exited correctly. It runs a while loop and traverse through 'LR_path[]' array and moves according to left or right command given.             
 * Logic:         This function is called after LR path is formed for a run. This functions first checks if the LR_path[] is empty , this case will happen when the current_location  = final_vertex,
 *                ie. the robot is already at its final location. Based on its front_axis and destination_axis it rotates and updates its front_axis. The rules are : 
 *
 *                front_axis(old) destination_axis front_axis(new)  rotation direction
 *                ----------------------------------------------------------------------------  
 *                1                 3                   3                   right 
 *       1                 2                   2                   left
 *                2                 1                   1                   right    
 *      2                 3                   3                   left
 *                3                 1                   1                   left
 *                3                 2                  2                   right 
 *
 *                After that it runs an infinite while loop then takes the values of ADC_Conversion(3),ADC_Conversion(2),ADC_Conversion(1) and 
 *                then run a nested while loop then again takes the values of ADC_Conversion(3),ADC_Conversion(2),ADC_Conversion(1) as input because 
 *                the values of White line sensors has to be updated. Then call "follow()" function repeatdly to move the bot in forward direction till
 *                this (Black Black Black): Right_white_line>0x20 && Center_white_line>0x20 && Left_white_line>0x20 is achieved , that is the robot has reached 
 *                a node, after that the inner while loop is exited (because as node is found , there is no need of moving forward) and then the function fetches
 *                the value present at index i = 0 from the LR_array[] , then checks whether 'L' or 'R' and moves left or right according to that. Further the value of i is incremented 
 *                so as to traverse in the LR_array[] . If '\0' is received that means that the whole 'LR_path[]' is traversed and now function can exit.                                                           
 * Example Call:  move_after_LR_path_formed() 
*/ 
int move_after_LR_path_formed()
{
 int i = 0;
 // This case runs only when the robot is at the same location where it has to go ie. current_location = final_vertex. It then only rotates and updates its front axis.
 /*
 *                front_axis(old) destination_axis front_axis(new)  rotation direction
 *                ----------------------------------------------------------------------------  
 *                1                 3                   3                   right 
 *       1                 2                   2                   left
 *                2                 1                   1                   right    
 *      2                 3                   3                   left
 *                3                 1                   1                   left
 *                3                 2                  2                   right 
 *
 */
  if(LR_path[i] == '\0'){
   if((front_axis == 1) && (destination_axis == 3)){
     right_degrees(160);
     front_axis = 3;
     stop();                            // The bot stops for stablization 
     back_mm(back_value);               // The bot moves back to detect the BBB condition or the node so as to generate the new LR path
   }
   else if((front_axis == 1) && (destination_axis == 2)){
     
     left_degrees(160);
    front_axis = 2;
      stop();
     back_mm(back_value);
    
   }
   else if((front_axis == 2) && (destination_axis == 1)){
      right_degrees(160); 
     front_axis = 1;
     stop();
     back_mm(back_value);
   }
   else if((front_axis == 2) && (destination_axis == 3)){
 
    left_degrees(160); 
    front_axis = 3;
     stop();
    back_mm(back_value);
   }
   else if((front_axis == 3) && (destination_axis == 1)){ 
    left_degrees(160);
     front_axis = 1;
     stop();
     back_mm(back_value);
   }
   else if((front_axis == 3) && (destination_axis == 2)){
     right_degrees(160);
     front_axis = 2;
     stop();
     back_mm(back_value);
   }
   else{
    // It is already in the same destination_axis
   }
  }
 while (1)
 {
 Left_white_line = ADC_Conversion(3);   // Getting data of Left White Line Sensor
 Center_white_line = ADC_Conversion(2); // Getting data of Center White Line Sensor
 Right_white_line = ADC_Conversion(1);  // Getting data of Right White Line Sensor
 while(1){
 Left_white_line = ADC_Conversion(3);  
 Center_white_line = ADC_Conversion(2); 
 Right_white_line = ADC_Conversion(1);  
 follow();
 if(Right_white_line>0x20 && Center_white_line>0x20 && Left_white_line>0x20) //BBB
 { 
 stop ();
   _delay_ms(200);
   forward_mm(80);
   break;
 }
     }
 
     if(LR_path[i] == 'L'){
 left_degrees(80);
 forward_mm(200);
 i++;
 }
    else if(LR_path[i]=='R'){
 right_degrees(80);
 forward_mm(200);
 i++;
       }
  
    else if(LR_path[i] == '\0'){
 stop();
 break;
 }
    else{
          stop();
 } 
 }
 
 return 0;
}
/* 
 * Function Name: check_edge_case    
 * Input:         x_diff -> difference between two consecutive x-coordinates
 *                y_diff -> difference between two consecutive y-coordinates, these 'difference coordinates' and 'front_axis' are used to identify in which direction to move ('L' or 'R')              
 * Output:        int to inform the caller that the program exited correctly. It does 3 functions ie. check signs of x_diff and y_diff and front axis and rotate accordingly.
 *                After that it stops and move backwards for obtaining (Black Black Black) white line sensor condition because the bot leaves the BBB condition when it rotates.
 *                After that it updates the front axis according to x and y coordinate difference and current front axis.           
 * Logic:         This functions check the case when the robot is on a front_axis, on which the difference of coordinates (x2-x1 , y2-y1) (from current vertex to next vertex)
 *                are not defined in general traversal cases, ie. when the robot has to turn and move in 180 degrees angle direction.
 *                (x1 = x coordinate of current vertex)
 *                (x2 = x coordinate of final vertex)
 *                (y1 = y coordinate of current vertex)
 *                (y2 = y coordinate of final vertex)
 *               When the robot is pointing towards a particular axis on which the movement defined by difference of axis and current front axis is not defined then a series 
 *               of if else statements that check that whether if its that case then it will first rotate 120 degrees right and then stop and move backwards
 *               so that ( Black Black Black ) white line sensor condition is achieved , because the BBB condition is not present  after 120 degrees rotation.
 *               After that the front axis is updated according to the following rules :
 *               
 *               x_diff     y_diff             front_axis(old)     front_axis(new) 
 *               -----------------------------------------------------------------
 *               positive   positive                3                   2 
 *               positive   constant    1                   3
 *               positive   negative                2                   1
 *               negative   negative                3                   2 
 *               negative   constant                1                   3
 *               negative   positive                2                   1
 *                                                         
 * Example Call:  check_edge_case(x_coordinate_difference , y_coordinate_difference) 
*/
int check_edge_case(int x_diff , int y_diff)
{
      if((x_diff > 0 && y_diff > 0) && (front_axis == 3))       // x positive, y positive
      {
   forward_mm(300);
   right_degrees(160);                                // Rotate by 120 degrees right
      stop(); 
   _delay_ms(500);                                                // Stop the robot
      back_mm(800);                                     // Move backwards by 'back_value'( = 800 ) for obtaining BBB condition on white line sensors and continue path traversal
      front_axis = 2;                                          // Update the value of front axis according to rotation
      }
      
      if((x_diff > 0 && y_diff == 0) && (front_axis == 1))       // x positive, y constant
      {
   forward_mm(300);
      right_degrees(160);                                 // rotate by 120 degrees right
      stop();
   _delay_ms(500);
      back_mm(800);
      front_axis = 3;
      }
      
      if((x_diff > 0 && y_diff < 0) && (front_axis == 2)){      // x positive, y negative
      //rotate 120 R
   forward_mm(300);
   right_degrees(160);                              // rotate by 120 degrees right
      stop();
   _delay_ms(500);
      back_mm(800);
      front_axis = 1;
     }
     // x - ,y -
     if((x_diff < 0 && y_diff < 0) && (front_axis == 3)){
      // rotate by 120 R
   forward_mm(300);
      right_degrees(160);                                     // rotate by 120 degrees right
      stop();
   _delay_ms(500);
      back_mm(800);
      front_axis = 2;
     }
     // x- , y = k
     if((x_diff < 0 && y_diff == 0) && (front_axis == 1)){
      // rotate by 120 R
   forward_mm(300); 
   right_degrees(160);                                // rotate by 120 degrees right
      stop();
   _delay_ms(500);
      back_mm(800);
      front_axis = 3;
     }
     
     //x - ,y +
     if((x_diff < 0 && y_diff > 0) && (front_axis == 2)){
      forward_mm(300);
    
   right_degrees(160);                                    // rotate by 120 degrees right
      stop();
      _delay_ms(500);
   back_mm(800);
      front_axis = 1;
  }     
 return 0;
} 
/* 
 * Function Name: update_axis    
 * Input:   LR_path_counter -> It is the counter for 'LR_path[]' array that at the time of call points to a index of 'LR_path[]'              
 * Output:  int to inform the caller that the program exited correctly. Does one function ie. updates the front axis according to current front axis and the rotation command provided.       
 * Logic:   This function first checks the value of front axis and the movement command provided ('L'/'R')and updates the value of front axis according to following rules :
 * 
 * front_axis (OLD)  movement_command ('L'/'R')  front_axis (NEW)
 * --------------------------------------------------------------
 * 2                 L                           1
 * 2                 R                           3
 * 1                 L                           3
 * 1                 R                           2
 * 3                 L                           2
 * 3                 R                           1
 *    
 * Example Call: update_axis(LR_path_counter) 
*/  
int update_axis(int LR_path_counter)
{
 
 if(front_axis == 2 && LR_path[LR_path_counter] == 'L'){
 front_axis = 1;
 }
 else if(front_axis == 2 && LR_path[LR_path_counter] == 'R'){
 front_axis = 3;
 }
 else if(front_axis == 1 && LR_path[LR_path_counter] == 'L'){
 front_axis = 3;
 }
 else if(front_axis == 1 && LR_path[LR_path_counter] == 'R'){
 front_axis = 2;
 }
 else if(front_axis == 3 && LR_path[LR_path_counter] == 'L'){
 front_axis = 2;
 }
 else if(front_axis == 3 && LR_path[LR_path_counter] == 'R'){
 front_axis = 1;
 }
 else{
 //  Error: Cannot update axis 
 }
 return 0;
}
/* 
 * Function Name: create_LR_path    
 * Input:         graph[V][V] -> It is the 54*54 adjancy matrix (graph representation) that stores nodes and how they are connected with each other.
 *                initial_vertex -> It is the current / initial location (vertex number) of the robot , it is later used by 'dijsktra' to calculate path and distance to final vertex.
 *                final_vertex -> It is the final location (vertex number) of the robot , it is later used by 'dijsktra' to calculate path and distance from initial vertex. 
 * Output:        int to inform the caller that the program exited correctly. Does five functions : Calls 'dijkstra' with case_flag = 2 (the last parameter) that creates index_path_array[] that
 *                stores sequence of vertex numbers (the number assigned to each node in the arena/map/graph). After that (x1,x2) and (y1,y2) are calculated and 
 *                'check_edge_case(x_coordinate_difference, y_coordinate_difference)' is called to check edge (backward 180 motion case) and then 'L'/'R' path is stored ,axis is updated 
 *                and 'move_after_LR_path_formed()' is called to move the robot according to LR path formed.        
 * Logic:         This function first initializes few variables and then calls 'dijkstra(graph,initial_vertex,final_vertex,2)' with last parameter as '2' (case_flag) which is used by dijkstra to
 *                identify that it has to create a vertex_path_array[V] according to initial and final vertex provided to it. After that 'vertex_path_array[V]' is traversed in consecutive pair 
 *                and according to vertex number(that is stored in vertex_path_array[V]) the two consecutive vertex numbers in the array are provided with coordinates (x1,y1) and (x2,y2) on 
 *                the basis of the following rule :
 *
 *                vertex number   x-coordinate  y-coordinate
 *               ---------------------------------------------
     0               0              2
     1               0              0
     2               0             -2
     3               1              3 
     4               1              1
     5               1             -1 
     6               1             -3
     7               2              3
     8               2              1
     9               2             -1
     10              2             -3
     11              3              4
     12              3              2
     13              3              0
     14              3             -2
     15              3             -4
     16              4              4
     17              4              2
     18              4              0
     19              4             -2
     20              4             -4
     21              5              5 
     22              5              3
     23              5              1
     24              5             -1
     25              5             -3
     26              5             -5
     27              6              5
     28              6              3
     29              6              1
     30              6             -1
     31              6             -3
     32              6             -5
     33              7              4           
     34              7              2
     35              7              0
     36              7             -2 
     37              7             -4  
     38              8              4
     39              8              2
     40              8              0
     41              8             -2
     42              8             -4
     43              9              3
     44              9              1                    
     45              9             -1
     46              9             -3
     47              10             3
     48              10             1
     49              10            -1
     50              10            -3
     51              11             2
     52              11             0
     53              11            -2 
 * After assigning the x1,y1 and x2,y2 to consecutive coordinates their difference is calculated and stored in x_coordinate_difference and y_coordinate_difference. On the basis of this difference
 * of x and y coordinate the bot decide whether to move left or right. It calculates the difference between the coordinates and then according to following rules
 * fills the 'LR_path[]' array with 'L' or 'R' :
 *
 *     x_coordinate_difference  y_coordinate_difference    front_axis  Movement (or assign values to 'LR_path[]' array) 
 * --------------------------------------------------------------------------------------------------------------- 
 *      positive    positive      1           Left
 *      positive    positive      2   Right
 *      positive    constant      3           Right
 *      positive    constant      2           Left
 *      positive    negative      3           Left
 *      positive    negative      1   Right
 *      negative    negative      2           Right                               {  fa : the axis in which robot's nose is pointing / or in which direction("1-1"/"2-2"/"3-3")
 *      negative    negative      1   Left                                   Left : 'L'  
 *      negative    constant      2           Left                                   Right : 'R'  
 *      negative    constant      3           Right                               } 
 *      negative    positive      3           Left 
 *      negative    positive         1           Right  
 *
 * After this 'check_edge_case' is called that checks initially whether the robot can move or not in the direction provided to it , if not it rotates the bot
 * 120 degrees right so that it can follow 'L' or 'R' command given to it. After that using above rules 'LR_path' array is formed ,and after each LR assigning 'update_axis(LR_path_counter)'
 * is called that updates the front axis of the robot because after each left or right the robot has changed its orientation from "1-1" to "2-2" and such. After the whole 'vertex_path_array[]'      
 * has been traversed and 'LR_path_array[]' has been populated with sequence of 'L' and 'R' then 'LR_path[LR_path_counter + 1] = '\0'' is done to mark a sentinel so that we can traverse in
 * 'LR_path[]' till we get a '\0' , this is explicitly done because during two paths of different lengths , due to global scope '\0' is misplaced and the smaller path takes extra charectors(L/R).
 * After that 'move_after_LR_formed()' is called that traverses through the 'L' 'R' sequence and moves the robot according to that.    
 * Example Call:  create_LR_path(graph,current_location,final_vertex) 
*/
 
 
int create_LR_path(int graph[V][V],int initial_vertex,int final_vertex){
    int x1, x2, y1, y2, x, y, k, j,LR_path_counter = -1;
 int x_coordinate_difference;
 int y_coordinate_difference;
 
    dijkstra(graph,initial_vertex,final_vertex,2);
     for(int i = 0; i < vertex_path_array_counter; i++){
         j = 1;
         while(j<=2)
         {  if(j == 1){
              k = i;                
            }
            else{
              k = i+1;  
            }
            switch(vertex_path_array[k])
    {
        case 0 : 
                x = 0; 
                y = 2;
                break;
        case 1 : 
                 x = 0;
                 y = 0;
                 break;
        case 2 : 
                        x = 0;
                 y = -2;
                break;
        case 3 :
                 x = 1;
                 y = 3;
                break;
        case 4 : 
                 x = 1;
                 y = 1;
                 break;
        case 5 : 
                 x = 1;
                 y = -1;
                break;
        case 6 : 
                 x = 1;
                 y = -3;
                break;
        case 7 : 
                 x = 2;
                 y = 3;
                break;
        case 8 : 
                 x = 2;
                 y = 1;
                break;
        case 9 : 
                 x = 2;
                 y = -1;
                break;
        case 10 : 
                 x = 2;
                 y = -3;
                break;
        case 11 : 
                 x = 3;
                 y = 4;
                break;
        case 12 : 
                 x = 3;
                 y = 2;
                break;
        case 13 : 
                 x = 3;
                 y = 0;
                 break;
        case 14 : 
                 x = 3;
                 y = -2;
                break;
        case 15 : 
                 x = 3;
                 y = -4;
                 break;
        case 16 : 
                 x = 4;
                 y = 4;
                 break;
        case 17 : 
                 x = 4;
                 y = 2;
                break;
        case 18 : 
                 x = 4;
                 y = 0;
                break;
        case 19 : 
                 x = 4;
                 y = -2;
                break;
        case 20 : 
                 x = 4;
                 y = -4;
                 break;
        case 21 : 
                 x = 5;
                 y = 5;
                break;
        case 22 : 
                 x = 5;
                 y = 3;
                break;
        case 23 : 
                 x = 5;
                 y = 1;
                break;
        case 24 : 
                 x = 5;
                 y = -1;
                break;
        case 25 : 
                 x = 5;
                 y = -3;
                break;
        case 26 : 
                 x = 5;
                 y = -5;
                break;
        case 27 : 
                 x = 6;
                 y = 5;
                break;
        case 28 : 
                 x = 6;
                 y = 3;
                break;
        case 29 : 
                 x = 6;
                 y = 1;
                break;
        case 30 : 
                 x = 6;
                 y = -1;
                break;
        case 31 : 
                 x = 6;
                 y = -3;
                break;
        case 32 : 
                 x = 6;
                 y = -5;
                break;
        case 33 : 
                 x = 7;
                 y = 4;
                break;
        case 34 : 
                 x = 7;
                 y = 2;
                break;
        case 35 : 
                 x = 7;
                 y = 0;
                break;
        case 36 : 
                 x = 7;
                 y = -2;
                break;
        case 37 : 
                 x = 7;
                 y = -4;
                break;
        case 38 : 
                 x = 8;
                 y = 4;
                break;
        case 39 : 
                 x = 8;
                 y = 2;
                break;
        case 40 : 
                 x = 8;
                 y = 0;
                break;
        case 41 : 
                 x = 8;
                 y = -2;
                break;
        case 42 : 
                 x = 8;
                 y = -4;
                break;
        case 43 : 
                 x = 9;
                 y = 3;
                break;
        case 44 : 
                 x = 9;
                 y = 1;
                break;
        case 45 : 
                 x = 9;
                 y = -1;
                break;
        case 46 : 
                 x = 9;
                 y = -3;
                break;
        case 47 : 
                 x = 10;
                 y = 3;
                break;
        case 48 : 
                 x = 10;
                 y = 1;
                break;
        case 49 : 
                 x = 10;
                 y = -1;
                 break;
        case 50 : 
                 x = 10;
                 y = -3;
                break;
        case 51 : 
                 x = 11;
                 y = 2;
                break;
        case 52 : 
                 x = 11;
                 y = 0;
                 break;
        case 53 : 
                 x = 11;
                 y = -2;
                break;
       default : // Error: Not generating coordinates 
                 break;
    }
    if(k == i ){
              x1 = x;
              y1 = y;
    }
    if(k == (i+1)){
              x2 = x;
              y2 = y;
    }
    j++;
        }
        
        x_coordinate_difference = x2 - x1;
        y_coordinate_difference = y2 - y1;                             
      
        check_edge_case(x_coordinate_difference, y_coordinate_difference);
 
 // x+ , y+
        if((x_coordinate_difference > 0 && y_coordinate_difference > 0) && (front_axis == 1)){
            LR_path_counter++;
            LR_path[LR_path_counter] = 'L';
        }
        else if((x_coordinate_difference > 0 && y_coordinate_difference > 0) && (front_axis == 2)){
            LR_path_counter++;
            LR_path[LR_path_counter] = 'R';
        }
        //x+ , y = k
        else if((x_coordinate_difference > 0 && y_coordinate_difference == 0) && (front_axis == 3)){
            LR_path_counter++;
            LR_path[LR_path_counter] = 'R';
        }
        else if((x_coordinate_difference > 0 && y_coordinate_difference == 0) && (front_axis == 2)){
            LR_path_counter++;
            LR_path[LR_path_counter] = 'L';
        }
        //x+ , y -
        else if((x_coordinate_difference > 0 && y_coordinate_difference < 0) && (front_axis == 3)){
            LR_path_counter++;
            LR_path[LR_path_counter] = 'L';
         }
        else if((x_coordinate_difference > 0 && y_coordinate_difference < 0) && (front_axis == 1)){
            LR_path_counter++;
            LR_path[LR_path_counter] = 'R';
        }
        
        // x - ,y - 
        else if((x_coordinate_difference < 0 && y_coordinate_difference < 0) && (front_axis == 2)){
            LR_path_counter++;
            LR_path[LR_path_counter] = 'R';
        }
        else if((x_coordinate_difference < 0 && y_coordinate_difference < 0) && (front_axis == 1)){
            LR_path_counter++;
            LR_path[LR_path_counter] = 'L';
        }
        // x- , y = k 
        else if((x_coordinate_difference < 0 && y_coordinate_difference == 0) && (front_axis == 2)){
            LR_path_counter++;
            LR_path[LR_path_counter] = 'L';
        }
        else if((x_coordinate_difference < 0 && y_coordinate_difference == 0) && (front_axis == 3)){
            LR_path_counter++;
            LR_path[LR_path_counter] = 'R';
        }
        //x - ,y +
        else if((x_coordinate_difference < 0 && y_coordinate_difference > 0) && (front_axis == 3)){
            LR_path_counter++;
            LR_path[LR_path_counter] = 'L';
        }
        else if((x_coordinate_difference < 0 && y_coordinate_difference > 0) && (front_axis == 1)){
            LR_path_counter++;
            LR_path[LR_path_counter] = 'R';
        }
        else{
             // Error: Difference calculate , axis and assigning LR 
        }
        
       update_axis(LR_path_counter);
        
    }
 
    LR_path[LR_path_counter + 1] = '\0';
 move_after_LR_path_formed();
 return 0;
}

/* 
 * Function Name: decode_and_assign_vertices    
 * Input:         None  
 * Output:        int to inform the caller that the program exited correctly. It concatenates cell number and axis and assign the two vertices to pebble or pitcher.      
 * Logic:         This functions first assigns the distances array (the array used to calculate the shortest among all the distances from current_location) to -1. After that 
 *                it one by one concatenates pebble cell and pebble axis (1/2/3) and pitcher cell and pitcher axis and calls 'decode_into_index(pebble_cell1)' to calculate the index of 
 *                the concatenated string eg . "16-1-1" ie. 18 , after that it calls 'assign_two_vertices(decode_index_of_pebble,1)' to assign two vertices ('vertex number') to the respective 
 *                pebbles and the pitcher.                                   
 * Example Call:  decode_and_assign_vertices() 
*/
int decode_and_assign_vertices(){
 int decode_index_of_pebble_1;
 int decode_index_of_pebble_2;
 int decode_index_of_pebble_3;
 int decode_index_of_pitcher;
 
 for(int i = 0;i < 8; i++){
  distances[i] = -1;
 }
 
 strcat(pebble_cell1,"-");
 strcat(pebble_cell1,pebble_axis1);
 decode_index_of_pebble_1 = decode_into_index(pebble_cell1);
 assign_two_vertices(decode_index_of_pebble_1,1);
 
 strcat(pebble_cell2,"-");
 strcat(pebble_cell2,pebble_axis2);
 decode_index_of_pebble_2 = decode_into_index(pebble_cell2);
 assign_two_vertices(decode_index_of_pebble_2,2);
 
 strcat(pebble_cell3,"-");
 strcat(pebble_cell3,pebble_axis3);
 decode_index_of_pebble_3 = decode_into_index(pebble_cell3);
 assign_two_vertices(decode_index_of_pebble_3,3);
 strcat(pitcher_cell,"-");
 strcat(pitcher_cell,pitcher_axis);
 decode_index_of_pitcher = decode_into_index(pitcher_cell);
 assign_two_vertices(decode_index_of_pitcher,0);
 
 return 0;
}
/* 
 * Function Name: update_destination    
 * Input:         odd_even_value -> This variable takes an integer value that is odd or even and then according to that alternately decide wheter to go to pebble or pitcher. 
 *                When 'odd_even_value' is odd then the 'final_vertex' is set from one of the pitcher locations and When 'odd_even_value' is even then the 'final_vertex' is set from one of the pebble locations 
 *                graph[V][V] -> Adjancy matrix of 54*54 that stores the relation between the vertices
 * Output:        int to inform the caller that the program exited correctly. It calculate the shortest AR object from the current location. It also changes the destination name and axis.      
 * Logic:         This function checks weather the value of 'odd_even_value' is odd or even , because the robot has to follow alternate
 *                sequence ie. :  
 *                START -> Pebble
 *                Pebble -> Pitcher (till here in current task)
 *                Pitcher -> Pebble (in next task) and similar for 3 pebbles in total           
 *                So to follow alternate sequence if 'odd_even_value' is even robot moves towards Pebble and if even then the robot moves 
 *                towards Water Pitcher. The robot initially (when current_location is START-1/2) have 6 locations('vertex number') to choose from or go to : 
 *                pebble_1_vertex_1 , pebble_1_vertex_2,pebble_2_vertex_1 , pebble_2_vertex_2,pebble_3_vertex_1 , pebble_3_vertex_2
 *                The function first calculate the distance between all the above vertices and the current_location and then find the shortest path distance and according to the shortest path
 *                the final_vertex is assigned.
 *                The flow  that the robot will follow ((below) from left to right):                   
 *                pebble_1_vertex_1                                                                                                                                  pebble_2_vertex_1                                                                                  (if pebble_2_vertex_1 is smallest(closest))                                      (after choosing shortest from left ones and traversing path)
 *                pebble_1_vertex_2     (after choosing shortest from left ones and traversing path)  pitcher_vertex_1  (if pebble_1_vertex_1 is smallest(closest))  pebble_2_vertex_2  (after choosing shortest from left ones and traversing path)  pitcher_vertex_1 -------------------------------------------> pebble_3_vertex_1,pebble_3_vertex_2  -------------------------------------------------------------> pitcher_vertex_1,pitcher_vertex_2---------------> END
 *                pebble_2_vertex_1    ------------------------------------------------------------>                    ------------------------------------------>  pebble_3_vertex_1   ---------------------------------------------------------->  pitcher_vertex_2
 *                pebble_2_vertex_2                                                                   pitcher_vertex_2                                               pebble_3_vertex_2    
 *                pebble_3_vertex_1
 *                pebble_3_vertex_2                                                             
 *                When the bot is at any current_location for executing in shortest time the bot should be able to find shortest path among number of available vertices.
 *                Example when the robot is at current_location == START-1/2 , then it has 6 locations to choose from pebble_1_vertex_1,pebble_1_vertex_2,pebble_2_vertex_1,pebble_2_vertex_2,pebble_3_vertex_1,pebble_3_vertex_2.
 *                The bot first calculates shortest path from current_location to all available vertices and then chooses the shortest one.
 *                distances[] array is used to store to distances from 'current_location' to all available vertices. 'shortest_index' stores the index of that shortest distance node.
 *                When the odd_even_value == even , ie. the bot has to go to pebble , it calculates all distances from current_location and then assigns final_vertex to appropriate pebble.
 *                It also assigns destination_axis to that pebble's axis.
 *                'shortest_index' is assigned and used as follows : 
 *               if , shortest_index == 2 then final_vertex = pebble_1_vertex_1
*                if , shortest_index == 3 then final_vertex = pebble_1_vertex_2
*                if , shortest_index == 4 then final_vertex = pebble_2_vertex_1
*                if , shortest_index == 5 then final_vertex = pebble_2_vertex_2
*                if , shortest_index == 6 then final_vertex = pebble_3_vertex_1
*                if , shortest_index == 7 then final_vertex = pebble_3_vertex_2
*                After calculating shortest_index or shortest vertex the cells other vertex and itself the vertex is assigned V = 54 ,that helps to ignore it from next turn(because the bot has already been there)
*                'update_animation' is also assigned to respective pebble's that later message python script to change the repective ARuco Id'.
*                'update_animation' rules :(the variable's are defined as 'a','b','c','d' because storing names like 'p1','p2' requires strcpy() , which is generating lag due to input buffer)
*                when update_animation == 'a' for changing pebble_1_vertex_1/pebble_1_vertex_2
*                when update_animation == 'b' for changing pebble_2_vertex_1/pebble_2_vertex_2
*                when update_animation == 'c' for changing pebble_3_vertex_1/pebble_3_vertex_2
*                when update_animation == 'd' for changing pitcher_vertex_1/pitcher_vertex_2
* Example Call:  update_destination(graph,update_destination_counter) 
*/
int update_destination(int graph[V][V] , int odd_even_value){
 int shortest;
 int shortest_index;
    
 if((odd_even_value % 2) == 0)
 {
  if(distances[2] != V)
  {distances[2] = dijkstra(graph,current_location,pebble_1_vertex_1,1);}
  if(distances[3] != V)
  {distances[3] = dijkstra(graph,current_location,pebble_1_vertex_2,1);}
  if(distances[4] != V)
  {distances[4] = dijkstra(graph,current_location,pebble_2_vertex_1,1);}
  if(distances[5] != V)
  {distances[5] = dijkstra(graph,current_location,pebble_2_vertex_2,1);}
  if(distances[6] != V)
  {distances[6] = dijkstra(graph,current_location,pebble_3_vertex_1,1);}
  if(distances[7] != V)
  {distances[7] = dijkstra(graph,current_location,pebble_3_vertex_2,1);}
 
 shortest = distances[2];
 shortest_index = 2;
      for (int i = 2; i <= 7; i++) 
   {
       if (distances[i] <= shortest) 
    {
         shortest = distances[i];
         shortest_index = i;
    }
      }
  
 if(shortest_index == 2)
 { 
   final_vertex = pebble_1_vertex_1;
   destination_axis = pebble_axis1[0] - '0';
   distances[2] = V;
   distances[3] = V;
   update_animation = 'a';
 }
 if(shortest_index == 3)
 {
   final_vertex = pebble_1_vertex_2;
   destination_axis = pebble_axis1[0] - '0';
   distances[3] = V;
   distances[2] = V;
   update_animation = 'a'; 
 }
 if(shortest_index == 4)
 {
   final_vertex = pebble_2_vertex_1;
   destination_axis = pebble_axis2[0] - '0';
   distances[4] =  V;
   distances[5] = V;
 update_animation = 'b'; 
 }
 if(shortest_index == 5)
 {
   destination_axis = pebble_axis2[0] - '0';
   final_vertex = pebble_2_vertex_2;
   distances[5] = V;
   distances[4] = V;
  update_animation = 'b';  
 } 
 if(shortest_index == 6)
 {
   final_vertex = pebble_3_vertex_1;
   destination_axis = pebble_axis3[0] - '0';
   distances[6] = V;
   distances[7] = V;
   update_animation = 'c';
  }
 if(shortest_index == 7)
 {
   final_vertex = pebble_3_vertex_2;
   destination_axis = pebble_axis3[0] - '0';
   distances[6] = V;
   distances[7] = V;
   update_animation = 'c';
  }
  
         destination_name = 'P'; 
 }
    
 
else{
  
  distances[0] = dijkstra(graph,current_location,pitcher_vertex_1,1);
  distances[1] = dijkstra(graph,current_location,pitcher_vertex_2,1);
 
  if(distances[0] <= distances[1])
  { 
   final_vertex = pitcher_vertex_1;
  }
  else
  { 
   final_vertex = pitcher_vertex_2;
  } 
  
        destination_axis = pitcher_axis[0] - '0';
        destination_name = 'W'; 
        update_animation = 'd';
    }
    return 0;
}
/* 
 * Function Name: select_shortest_path_and_move    
 * Input:         graph[V][V] -> It is the 54*54 adjancy matrix ( graph representation ) that stores nodes and how they are connected with each other.
 *                current_location -> It stores the current location of the robot , initially after 'initialize_start_locations()' call in 'main' function
 *                it gets the value 1 or 52 based on its start location (START-1/START-2) and after a run , its value is updated back in 'main' as final vertex,
 *                ie. the final node it has reached.        
 * Output:        int to inform the caller that the program exited correctly. Does three functions : call 'update_destination(update_destination_counter)' to update the 
 *                destination ( where next to go ), checks weather destination_name == 'W' and updates the value of 'update_destination_counter' and then calls 'create_LR_path(graph,current_location,final_vertex)' for creating LR path (ie. a path containing sequence of L R (left and right)).      
 * Logic:         This function first calls 'update_destination(update_destination_counter)' to update the destination name , axis and also assign vertex 1 (v1) and 2 (v2) their
 *                respective vertex number values. 'update_destination_counter', a global variable is provided as input that is later on increased by 1 , so as to provide a odd-even sequence
 *                in even cases : the robot has to go to Pebble and in odd cases : the robot has to go to Water pitcher. Then 'destination_name' is checked if it is equal to 'W' ie. 
 *                Water Pitcher then 'water_pitcher_visited_count', a global variable is increased , this acts as a sentinel, as whole program and robot will stop when 'water_pitcher_visited_count' 
 *                becomes equal to 3. After that create_LR_path() is called to create a LR path according to which the bot has to move.
 * Example Call:  select_shortest_path_and_move(graph,current_location) 
*/ 
int select_shortest_path_and_move(int graph[V][V],int current_location){
   update_destination(graph,update_destination_counter);
    update_destination_counter++;
    
    if(destination_name == 'W'){
        water_pitcher_visited_count++;
    }
   create_LR_path(graph,current_location,final_vertex);
    return 0;
}
/* 
 * Function Name: rotate_servo_and_transmit_after_run   
 * Input:   None       
 * Output:   int to inform the caller that the program exited correctly, and does mainly three functions after a successful run : rotate in direction of destination_axis , beeps the buzzer and
 *                transmit a charector to python script on laptop indicating to change the Blender model projected.     
 * Logic:         This function checks that after a successful run (from 'start' to 'pebble' or 'pebble' to 'pitcher' or 'pitcher' to 'pebble'),
 *      that whether the front axis is equal to destination axis, if it is not then according to the following rules the robot is rotated 
 *                left or right so as to align in open direction (destination_axis) of Aruco marker : 
 *                (fa = front_axis , da = destination_axis)
 *      Also the function checks the the bot is perfectly aligned with the axis so that it does not collide with the aruco marker.The bot after reaching the node of the destination the bot  
 *      moves a fixed distance forward using encoder and moves back until a case:
 *      a) The left sensor gets a black and the centre and the right gets white, in that case the bot adjust itself by moving a right by 15 degree, because the bot has reached this situation 
 *      when it is alligned slightly leftwards.
 *      b) The right sensor gets black and the centre and left sensor gets white, in that case the bot adjust itself by moving a left by 15 degree, because the bot has reached this situation
 *      when it is alligned slightly rightwards
 *      c) All the sensor gets black at the same time, this  means that the bot is straight and there is no need of adjustment required in this case.. 
 * 
 *                  if fa = 1 and da = 3 then rotate 120 degrees right
 *     if fa = 1 and da = 2 then rotate 120 degrees left
 *     if fa = 2 and da = 1 then rotate 120 degrees right
 *     if fa = 2 and da = 3 then rotate 120 degrees left
 *     if fa = 3 and da = 1 then rotate 120 degrees left
 *     if fa = 3 and da = 2 then rotate 120 degrees right,
 *     if none of above , means that robot is already in open direction(Only 2 sides of AR_object are open) (destination_axis) of Aruco marker.
 *                
 *                After that the bot checks whether it has reached to the pitcher, if reached it sends 'd' to python eg: uart_tx('d'). Which means that the anomation of crow dropping the pebble will start.
 *                Otherwise it will send 'u' which means that it has reached the pebble and the animation of crow picking the pebble will start in the openGL scene.Also it will on the magnet as it has reached the pebble.
 *                Then the bot will pick the pebble by streaching the Arm which will move by rotating the servo..
 *                Later it will send the characters to python by according to whether it has reached pebble or pitcher. 
 *                Pitcher: a) OFF the magnet(drop the pebble)
 *         b) Checks the water_pitcher_animation_count counter value and send 'm','h' and 'f' to make the pitcher make its water level up by medium,high and full respectively. After that it will send 'n'
 *         which means that after dropping the stone in the pitcher the crow will fly without having stone in the mouth which means normal fly.      
 *                Pebble:  If it has reached the pebble the it will check which pebble it has reached:
 *                         a) If the update_animation contains 'a' it means it has reached first pebble and it will send 'p' to the python so that it can change the animation at that aruco id.
 *         b) If the update_animation contains 'b' it means it has reached second pebble and it will send 'q' to the python so that it can change the animation at that aruco id.
 *         c) If the update_animation contains 'c' it means it has reached first pebble and it will send 'r' to the python so that it can change the animation at that aruco id.
 *         d) After that it will send 'w' to the python so that after picking the pebble the crow do flying animation with stone in the beak.
 *     After that it will move back by fixed distance so that it reaches node.   
 * Example Call:  rotate_servo_and_transmit_after_run() 
*/
int rotate_servo_and_transmit_after_run()
{
 if((front_axis == 1) && (destination_axis == 3)){
 forward_mm(250);                          //move the bot forward from the node and after that move the bot backwards to adjust
 int adjust = 0;                           //adjust variable used to adjust the bot at the node
 while (1)
 {
     
  velocity(125, 130);   
  backward();                              //move the bot backwards until it reaches the following conditions
  Left_white_line = ADC_Conversion(3);
  Center_white_line = ADC_Conversion(2);
  Right_white_line = ADC_Conversion(1);
  if(Center_white_line<line_thres && Left_white_line>line_thres && Right_white_line<line_thres)//BWW
  {
   adjust = 1;                            //the value counter is adjusted to 1 is BWW condition has occured
   break;
  }
  
  else if(Center_white_line<line_thres && Left_white_line<line_thres && Right_white_line>line_thres)//WWB
  {
   adjust = 2;                            //the value counter is adjusted to 2 is WWB condition has occured
   break;
  }
  
  else if(Center_white_line>line_thres && Left_white_line>line_thres && Right_white_line>line_thres)//BBB
  {
   adjust = 3;                            //the value counter is adjusted to 3 is BBB condition has occured
   break;
  }
 }
 
 if (adjust == 1){right_degrees(15);}     //if the value in adjust variable is 1 move slightly right
 if (adjust == 2){left_degrees(15);}      //if the value in adjust variable is 2 move slightly left
 forward_mm(500);
 right_degrees(rotation_value);
 stop(); 
if(update_animation = 'd'){
 uart_tx('d');
}
else{
 uart_tx('u');
 magnet_on();
}
   
for (int i = 180; i > 60; i--)
{
 servo_1(i);
 _delay_ms(10);
}
servo_1_free();
if(update_animation == 'd'){
 magnet_off();
 if(water_pitcher_animation_count == 1)
{uart_tx('m'); water_pitcher_animation_count++;}
 else if(water_pitcher_animation_count == 2)
{uart_tx('h');water_pitcher_animation_count++;}
 else if(water_pitcher_animation_count == 3)
{uart_tx('f');water_pitcher_animation_count++;}

uart_tx('n');
}
else{
 if(update_animation == 'a'){
  uart_tx('p');
}
if(update_animation == 'b'){
 uart_tx('q');
}
if(update_animation == 'c'){
 uart_tx('r');
}
    uart_tx('w');
}
for (int i = 60; i < 180; i++)
{
 servo_1(i);
 _delay_ms(10);
}
servo_1_free(); 

 }
 
 else if((front_axis == 1) && (destination_axis == 2)){
 forward_mm(250);
 int adjust = 0;
 while (1)
 {
  velocity(125, 130);   //131,109.53
  backward();
  Left_white_line = ADC_Conversion(3);
  Center_white_line = ADC_Conversion(2);
  Right_white_line = ADC_Conversion(1);
  if(Center_white_line<line_thres && Left_white_line>line_thres && Right_white_line<line_thres)//BWW
  {
   adjust = 1;
   break;
  }
  
  else if(Center_white_line<line_thres && Left_white_line<line_thres && Right_white_line>line_thres)//WWB
  {
   adjust = 2;
   break;
  }
  
  else if(Center_white_line>line_thres && Left_white_line>line_thres && Right_white_line>line_thres)
  {
   adjust = 3;
   break;
  }
 }
 
 if (adjust == 1){right_degrees(15);}
 if (adjust == 2){left_degrees(15);}  
 forward_mm(500);
 left_degrees(rotation_value);
 stop();
 
 
 if(update_animation == 'd'){
  uart_tx('d');
 }
 else{
  uart_tx('u');
  magnet_on();
 }
 
     for (int i = 180; i > 60; i--)
     {
      servo_1(i);
      _delay_ms(10);
     }
     servo_1_free();
 
 if(update_animation == 'd'){
  magnet_off();
  if(water_pitcher_animation_count == 1)
 {uart_tx('m'); water_pitcher_animation_count++;}
  else if(water_pitcher_animation_count == 2)
 {uart_tx('h');water_pitcher_animation_count++;}
  else if(water_pitcher_animation_count == 3)
 {uart_tx('f');water_pitcher_animation_count++;}

 uart_tx('n');
}
else{
 if(update_animation == 'a'){
  uart_tx('p');
}
if(update_animation == 'b'){
 uart_tx('q');
}
if(update_animation == 'c'){
 uart_tx('r');
}
    uart_tx('w');
}
 
     for (int i = 60; i < 180; i++)
     {
      servo_1(i);
      _delay_ms(10);
     }
     servo_1_free(); 
  
 }
 
 else if((front_axis == 2) && (destination_axis == 1)){
 forward_mm(250);
 int adjust = 0;
 while (1)
 {
  velocity(125, 130);   //131,109.53
  backward();
  Left_white_line = ADC_Conversion(3);
  Center_white_line = ADC_Conversion(2);
  Right_white_line = ADC_Conversion(1);
  if(Center_white_line<line_thres && Left_white_line>line_thres && Right_white_line<line_thres)//BWW
  {
   adjust = 1;
   break;
  }
  
  else if(Center_white_line<line_thres && Left_white_line<line_thres && Right_white_line>line_thres)//WWB
  {
   adjust = 2;
   break;
  }
  
  else if(Center_white_line>line_thres && Left_white_line>line_thres && Right_white_line>line_thres)
  {
   adjust = 3;
   break;
  }
 }
 
 if (adjust == 1){right_degrees(15);}
 if (adjust == 2){left_degrees(15);}  
 forward_mm(500);
 right_degrees(rotation_value);
 stop(); 

if(update_animation == 'd'){
 uart_tx('d');
}
else{
 uart_tx('u');
 magnet_on();
}
      for (int i = 180; i > 60; i--)
      {
       servo_1(i);
       _delay_ms(10);
      }
      servo_1_free();
   
   if(update_animation == 'd'){
    magnet_off();
    if(water_pitcher_animation_count == 1)
   {uart_tx('m'); water_pitcher_animation_count++;}
    else if(water_pitcher_animation_count == 2)
   {uart_tx('h');water_pitcher_animation_count++;}
    else if(water_pitcher_animation_count == 3)
   {uart_tx('f');water_pitcher_animation_count++;}

   uart_tx('n');
}
else{
 if(update_animation == 'a'){
  uart_tx('p');
}
if(update_animation == 'b'){
 uart_tx('q');
}
if(update_animation == 'c'){
 uart_tx('r');
}
    uart_tx('w');
}
     
      for (int i = 60; i < 180; i++)
      {
       servo_1(i);
       _delay_ms(10);
      }
      servo_1_free();
   
 }
 else if((front_axis == 2) && (destination_axis == 3)){
 forward_mm(250);
 int adjust = 0;
 while (1)
 {
  velocity(125, 130);   //131,109.53
  backward();
  Left_white_line = ADC_Conversion(3);
  Center_white_line = ADC_Conversion(2);
  Right_white_line = ADC_Conversion(1);
  if(Center_white_line<line_thres && Left_white_line>line_thres && Right_white_line<line_thres)//BWW
  {
   adjust = 1;
   break;
  }
  
  else if(Center_white_line<line_thres && Left_white_line<line_thres && Right_white_line>line_thres)//WWB
  {
   adjust = 2;
   break;
  }
  
  else if(Center_white_line>line_thres && Left_white_line>line_thres && Right_white_line>line_thres)
  {
   adjust = 3;
   break;
  }
 }
 
 if (adjust == 1){right_degrees(15);}
 if (adjust == 2){left_degrees(15);}  
 forward_mm(500);
 left_degrees(rotation_value);
 stop(); 
 
 
 if(update_animation == 'd'){
  uart_tx('d');
 }
 else{
  uart_tx('u');
  magnet_on();
 }
    for (int i = 180; i > 60; i--)
      {
       servo_1(i);
       _delay_ms(10);
      }
      servo_1_free();
   
   if(update_animation == 'd'){
    magnet_off();
    if(water_pitcher_animation_count == 1)
   {uart_tx('m'); water_pitcher_animation_count++;}
    else if(water_pitcher_animation_count == 2)
   {uart_tx('h');water_pitcher_animation_count++;}
    else if(water_pitcher_animation_count == 3)
   {uart_tx('f');water_pitcher_animation_count++;}

   uart_tx('n');
}
else{
 if(update_animation == 'a'){
  uart_tx('p');
}
if(update_animation == 'b'){
 uart_tx('q');
}
if(update_animation == 'c'){
 uart_tx('r');
}
    uart_tx('w');
}
     
      for (int i = 60; i < 180; i++)
      {
       servo_1(i);
       _delay_ms(10);
      }
      servo_1_free();
   
 }
 else if((front_axis == 3) && (destination_axis == 1)){
 forward_mm(250);
 int adjust = 0;
 while (1)
 {
  velocity(125, 130);   //131,109.53
  backward();
  Left_white_line = ADC_Conversion(3);
  Center_white_line = ADC_Conversion(2);
  Right_white_line = ADC_Conversion(1);
  if(Center_white_line<line_thres && Left_white_line>line_thres && Right_white_line<line_thres)//BWW
  {
   adjust = 1;
   break;
  }
  
  else if(Center_white_line<line_thres && Left_white_line<line_thres && Right_white_line>line_thres)//WWB
  {
   adjust = 2;
   break;
  }
  
  else if(Center_white_line>line_thres && Left_white_line>line_thres && Right_white_line>line_thres)
  {
   adjust = 3;
   break;
  }
 }
 
 if (adjust == 1){right_degrees(15);}
 if (adjust == 2){left_degrees(15);}  
 forward_mm(500);	 
 left_degrees(rotation_value);
 stop(); 
 
 
 if(update_animation == 'd'){
  uart_tx('d');
 }
 else{
  uart_tx('u');
  magnet_on();
 }
   for (int i = 180; i > 60; i--)
      {
       servo_1(i);
       _delay_ms(10);
      }
      servo_1_free();
 
   if(update_animation == 'd'){
    magnet_off();
    if(water_pitcher_animation_count == 1)
   {uart_tx('m'); water_pitcher_animation_count++;}
    else if(water_pitcher_animation_count == 2)
   {uart_tx('h');water_pitcher_animation_count++;}
    else if(water_pitcher_animation_count == 3)
   {uart_tx('f');water_pitcher_animation_count++;}

   uart_tx('n');
}
else{
 if(update_animation == 'a'){
  uart_tx('p');
}
if(update_animation == 'b'){
 uart_tx('q');
}
if(update_animation == 'c'){
 uart_tx('r');
}
    uart_tx('w');
}
     
      for (int i = 60; i < 180; i++)
      {
       servo_1(i);
       _delay_ms(10);
      }
      servo_1_free();
   
 }
 else if((front_axis == 3) && (destination_axis == 2)){
 forward_mm(250);
 int adjust = 0;
 while (1)
 {
  velocity(125, 130);   //131,109.53
  backward();
  Left_white_line = ADC_Conversion(3);
  Center_white_line = ADC_Conversion(2);
  Right_white_line = ADC_Conversion(1);
  if(Center_white_line<line_thres && Left_white_line>line_thres && Right_white_line<line_thres)//BWW
  {
   adjust = 1;
   break;
  }
  
  else if(Center_white_line<line_thres && Left_white_line<line_thres && Right_white_line>line_thres)//WWB
  {
   adjust = 2;
   break;
  }
  
  else if(Center_white_line>line_thres && Left_white_line>line_thres && Right_white_line>line_thres)
  {
   adjust = 3;
   break;
  }
 }
 
 if (adjust == 1){right_degrees(15);}
 if (adjust == 2){left_degrees(15);}  
 forward_mm(500);
 stop();
 _delay_ms(1000);
 right_degrees(rotation_value);
 stop(); 
 
 
 if(update_animation == 'd'){
  uart_tx('d');
 }
 else{
  uart_tx('u');
  magnet_on();
 }
   for (int i = 180; i > 60; i--)
      {
       servo_1(i);
       _delay_ms(10);
      }
      servo_1_free();
 
   if(update_animation == 'd'){
    magnet_off();
    if(water_pitcher_animation_count == 1)
   {uart_tx('m'); water_pitcher_animation_count++;}
    else if(water_pitcher_animation_count == 2)
   {uart_tx('h');water_pitcher_animation_count++;}
    else if(water_pitcher_animation_count == 3)
   {uart_tx('f');water_pitcher_animation_count++;}

   uart_tx('n');
}
else{
 if(update_animation == 'a'){
  uart_tx('p');
}
if(update_animation == 'b'){
 uart_tx('q');
}
if(update_animation == 'c'){
 uart_tx('r');
}
    uart_tx('w');
}
      for (int i = 60; i < 180; i++)
      {
       servo_1(i);
       _delay_ms(10);
      }
      servo_1_free();
   
 }
 
 else{
 forward_mm(250);
 int adjust = 0;
 while (1)
 {
  velocity(125, 130);   //131,109.53
  backward();
  Left_white_line = ADC_Conversion(3);
  Center_white_line = ADC_Conversion(2);
  Right_white_line = ADC_Conversion(1);
  if(Center_white_line<line_thres && Left_white_line>line_thres && Right_white_line<line_thres)//BWW
  {
   adjust = 1;  
   break;
  }
  
  else if(Center_white_line<line_thres && Left_white_line<line_thres && Right_white_line>line_thres)//WWB
  {
   adjust = 2;
   break;
  }
  
  else if(Center_white_line>line_thres && Left_white_line>line_thres && Right_white_line>line_thres)
  {
   adjust = 3;
   break;
  }
 }
 
 if (adjust == 1){right_degrees(15);}
 if (adjust == 2){left_degrees(15);}  
 stop();
 forward_mm(150);
 
 
 if(update_animation == 'd'){
  uart_tx('d');
 }
 else{
  uart_tx('u');
  magnet_on();
 }
    for (int i = 180; i > 60; i--)
      {
       servo_1(i);
       _delay_ms(10);
      }
      servo_1_free();
 
   if(update_animation == 'd'){
    magnet_off();
    if(water_pitcher_animation_count == 1)
   {uart_tx('m'); water_pitcher_animation_count++;}
    else if(water_pitcher_animation_count == 2)
   {uart_tx('h');water_pitcher_animation_count++;}
    else if(water_pitcher_animation_count == 3)
   {uart_tx('f');water_pitcher_animation_count++;}
   uart_tx('n');
}
else{
 if(update_animation == 'a'){
  uart_tx('p');
}
if(update_animation == 'b'){
 uart_tx('q');
}
if(update_animation == 'c'){
 uart_tx('r');
}
    uart_tx('w');
}
     
      for (int i = 60; i < 180; i++)
      {
       servo_1(i);
       _delay_ms(10);
      }
      servo_1_free();
   
 back_mm(300);
 
 }
 return 0;
}
 
//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}
//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
//This TIMER is used in funtioning of servo motor as with the help of timer we can vary the speed of the motor 
//Here we have only used one servo 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01; //Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03; //Output compare Register high value for servo 1
 OCR1AL = 0xFF; //Output Compare Register low Value For servo 1
 //OCR1BH = 0x03; //Output compare Register high value for servo 2
 //OCR1BL = 0xFF; //Output Compare Register low Value For servo 2
 //OCR1CH = 0x03; //Output compare Register high value for servo 3
 //OCR1CL = 0xFF; //Output Compare Register low Value For servo 3
 ICR1H  = 0x03; 
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
      For Overriding normal port functionality to OCRnA outputs.
      {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}
//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}
//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.
void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}
/* 
 * Function Name: main   
 * Input:         None  
 * Output:        int to inform the caller that the program exited correctly   
 * Logic:         First the Atmega sends '@' to the python script to message python that the robot is ready to take locations and axis (ie. 16,1-1)as input from xbee.
 *                Then the function takes input into robot_start,pitcher_cell,axis and pebble_cell,axis and other using uart_rx() functions and registers. 
 *                Then calls 'init_devices()' for initializing all the connected devices and pin configrations then call 'motor_pin_config()' for
 *                configuring motor pins. After that it calls 'initialize_start_locations()' for initializing 'robot_start' with'START-1' or 'START-2'. 
 *                After that it initializes 'graph[V][V]' ,a 54*54 adjancy matrix for storing the map/graph of the arena, after that decode_and_assign_vertices() is called to 
 *                initaliaze AR locations and then runs a infinite loop and calls 'select_shortest_path_and_move(graph,current_location)' which in turn
 *                calls many function for creating an array containing L R (left , right) sequence and then move according to that. After that 
 *                'rotate_servo_and_transmit_after_run()' is called to align the robot according to destination orientation.
 *                After that front axis is made equal to destination axis because the bot has aligned in the destination orientation from the previous
 *                function call. The infinite loop breaks when 'water_pitcher_visited_count' becomes equal to 3 ie. bot has moved to water pitcher thrice
 *                and then 'z' is transmitted to python script to project no movement in wing crow and buzzer is beeped for 5 seconds ie. END, if
 *                that is not true then the current location is updated by final vertex location.              
 * Example Call:  Called automatically by the Operating System   
*/
int main() 
{  
    uart0_init();
    uart_tx('@');
 
    for (int i = 0 ; i < 7 ; i++) 
    {
    robot_start[i] = uart_rx();
    }
    
 for (int i = 0 ; i < 13 ; i++)
 {
   water_pitcher_string[i] =uart_rx();   
 }
 
 for (int i = 0 ; i < 2 ; i++)
 {
  pitcher_cell[i] = uart_rx();
 }
 for (int i = 0 ; i < 3 ; i++)
 {
  pitcher_axis[i] = uart_rx();
 }
    
    for (int  i = 0 ; i < 7 ; i++)
 {
   pebble1_string[i] = uart_rx();  
 }
 
 for (int i = 0 ; i < 2  ; i++)
    {
     pebble_cell1[i] = uart_rx();
    }    
 for (int i = 0 ; i < 3 ; i++)
    {
    pebble_axis1[i] = uart_rx();
    }
 
     for (int  i = 0 ; i < 7 ; i++)
     {
      pebble2_string[i] = uart_rx();
     }
    
  for (int i = 0 ; i < 2 ; i++)
  {
   pebble_cell2[i] = uart_rx();
  }  
  for (int i = 0 ; i < 3 ; i++)
  {
   pebble_axis2[i] = uart_rx();
  }
    
  
         for (int  i = 0 ; i < 7 ; i++)
         {
          pebble3_string[i] = uart_rx();
         }
   
   for (int i = 0 ; i < 2 ; i++)
   {
    pebble_cell3[i] = uart_rx();
   }   
   for (int i = 0 ; i < 3 ; i++)
   {
    pebble_axis3[i] = uart_rx();
   }
        
if (pebble_cell1[0] == '0'){pebble_cell1[0] = pebble_cell1[1];pebble_cell1[1] = '\0';}
if (pebble_cell2[0] == '0'){pebble_cell2[0] = pebble_cell2[1];pebble_cell2[1] = '\0';}
if (pebble_cell3[0] == '0'){pebble_cell3[0] = pebble_cell3[1];pebble_cell3[1] = '\0';}
if (pitcher_cell[0] == '0'){pitcher_cell[0] = pitcher_cell[1];pitcher_cell[1] = '\0';}      
   
 init_devices();
 motor_pin_config(); 
initialize_start_locations();  
int graph[V][V] = {         
{0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0}
}; 
 
decode_and_assign_vertices(); 
while(1)
{
     select_shortest_path_and_move(graph,current_location);
     rotate_servo_and_transmit_after_run();
     front_axis = destination_axis;   
     
  if(water_pitcher_visited_count == 3)
  {
   uart_tx('z'); 
   buzzer_on();
   _delay_ms(5000);
   buzzer_off();
         break;
     }  
     else
  { 
           current_location = final_vertex;
     }
 }
 return 0; 
}