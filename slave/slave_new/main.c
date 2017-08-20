
/*
* Team ID:             eYRC-BV#2
*
* Author List :        Shivam Waghela, Akshdeep Rungta, Anuj Singh, Siddharth Vyas
*
* Filename:            main.c
*
* Theme:               Bothoven
*
* Functions:           int main(void)
*                      int check_path(int, int, int)
*				        void identify_neighbors_of_MNP()
*				        void find_shortest_path()
*				        void bot_traversal();
*                      void turn_left(unsigned int);
*                      void turn_right(unsigned int);
*                      void find_optimal_path_for_MNP_with_several_neighbor_nodes(int[], int);
*
* Global Variables:  	int MNP_list[]
*                      int MNP_list_length
*                      int source_node
*                      int destination_node
*                      int current_bot_angle
*                      int path_angle = 0
*                      short rotation_angles[48][48]
*/

#define F_CPU 14745600


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#include "lcd.c"
#include "path_calculator.c"

int obs_flag;
int flag_tx; // transmit data
int flag_rx; // receive data


int MNP_list[33]; // list of MNPs the robot has to visit
int MNP_list_length = 0;
int slave_mnp_list[20];
int master_mnp_list[20];
int master_mnp_list_length = 0;
int slave_mnp_list_length = 0;

int source_node; // start node, indexed from 0
int destination_node; // stores the node near to the destination MNP
int current_bot_angle = 360; // stores the current angle of bot
int path_angle = 0; //stores the angle of the next path
int track_current_mnp_index; // holds the mnp corresponding to the next node to be played

int x = 0, y = 0;

// stores the angles between nodes by taking North direction as 0 angle
short rotation_angles[48][48] = {
	{0,   15,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   165, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{180, 0,   30,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   195, 0,   45,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   90,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   150, 0,   0,   0,   0,   0,   0},
	{0,   0,   210, 0,   60,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   225, 0,   75,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   240, 0,   90,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   255, 0,   105, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   210, 150, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   270, 0,   120, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   285, 0,   135, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   300, 0,   150, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   315, 0,   165, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   270, 210, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   330, 0,   180, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   345, 0,   195, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   360, 0,   210, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   15,  0,   225, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   330, 270, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   30,  0,   240, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   45,  0,   255, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   60,  0,   270, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   75,  0,   285, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   30,  330, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   90,  0,   300, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   105, 0,   315, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   120, 0,   330, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   135, 0,   345, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   90,  30,  0,   0,   0,   0,   0,   0,   0,   0},
	{360, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   150, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   270, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   90,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   210, 0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   270, 0,   30,  0,   90,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   150, 0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   30,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   210, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   330, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   150, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   270, 0,   330, 0,   90,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   210, 0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   90,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   270, 0,   150, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   30,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   330, 0,   210, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   30,  0,   150, 0,   210, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   270, 0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   150, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   330, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   90,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   270, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   30,  0,   90,  0,   210, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   330, 0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   210, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   30,  0,   270, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   150, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   90,  0,   330, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   150, 0,   270, 0,   330, 0,   0,   0,   0,   0,   0,   30},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   270, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   90,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   210, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   30,  0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   150, 0,   210, 0,   330, 90,  0,   0,   0,   0,   0},
	{0,   0,   330, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   30,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   150, 0,   0,   0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   270, 0,   0,   30,  0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   330, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   210, 0,   0,   0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   30,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   150, 0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   90,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   330, 0,   0,   0},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   150, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   270},
	{0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   210, 0,   0,   0,   0,   0,   0,   0,   0,   90,  0}
};

int check_path(int, int, int);

void identify_neighbors_of_MNP();

void find_shortest_path();

void traverse(int master_index);

void bot_traversal();

void turn_left(unsigned int);

void turn_right(unsigned int);

void find_optimal_path_for_MNP_with_several_neighbor_nodes(int[], int);

int cal_striking_angle(int mnp, int node);

void play_note(int mnp);
void slave_com();

////////////////////////////////////////////////////////
//line following
///////////////////////////////////////////////////////

/*
* Team ID:              eYRC-BV#2
*
* Author List :         Shivam Waghela, Akshdeep Rungta, Anuj Singh, Siddharth Vyas
*
* Filename:             line_following.c
*
* Theme:                Bothoven
*
* Functions:            void follow(void)
*                       void port_init();
*						 void init_devices();
*						 void timer1_init();
*						 void timer5_init();
*						 void motion_pin_config (void);
*						 void motion_set (unsigned char);
*						 void forward (void);
*						 void stop (void);
*						 void back (void);
*						 void left (void);
*						 void right (void);
*						 void velocity(unsigned char, unsigned char);
*						 void motors_delay();
*						 void left_encoder_pin_config();
*						 void right_encoder_pin_config();
*						 void left_position_encoder_interrupt_init(void);
*						 void right_position_encoder_interrupt_init(void);
*						 void buzzer_pin_config(void);
*						 void buzzer_on();
*						 void buzzer_off();
*						 void adc_init();
*						 void adc_pin_config (void);
* 						 void lcd_port_config (void);
*						 void print_sensor(char row, char coloumn,unsigned char channel);
*						 unsigned char ADC_Conversion(unsigned char Ch);
* 						 void angle_rotate(unsigned int Degrees);
*						 void left_degrees(unsigned int Degrees);
*						 void right_degrees(unsigned int Degrees);
*						 void linear_distance_mm(unsigned int DistanceInMM);
*						 void forward_mm(unsigned int DistanceInMM);
*						 void back_mm(unsigned int DistanceInMM);
*						 void follow();
*
* Global Variables:  	volatile unsigned long int ShaftCountRight = 0
*						volatile unsigned long int ShaftCountLeft = 0
*						int has_turned = 0
*						int last_turned = 0
*/

void port_init();

void init_devices();

void timer1_init();

void timer5_init();

void motion_pin_config(void);

void motion_set(unsigned char);

void forward(void);

void stop(void);

void back(void);

void left(void);

void right(void);

void velocity(unsigned char, unsigned char);

void motors_delay();

void left_encoder_pin_config();

void right_encoder_pin_config();

void left_position_encoder_interrupt_init(void);

void right_position_encoder_interrupt_init(void);

void buzzer_pin_config(void);

void buzzer_on();

void buzzer_off();

void adc_init();

void adc_pin_config(void);

void lcd_port_config(void);

void print_sensor(char row, char coloumn, unsigned char channel);

unsigned char ADC_Conversion(unsigned char Ch);

void angle_rotate(unsigned int Degrees);

void left_degrees(unsigned int Degrees);

void right_degrees(unsigned int Degrees);

void linear_distance_mm(unsigned int DistanceInMM);

void forward_mm(unsigned int DistanceInMM);

void back_mm(unsigned int DistanceInMM);

void follow();

volatile unsigned long int ShaftCountRight = 0;
volatile unsigned long int ShaftCountLeft = 0;


/*
*
* Function Name:	follow
* Input:			void
* Output:		    void
* Logic:			Uses PID line following algorithm to follow line from one node to another
* Example Call:	follow();
*
*/
void follow(void)
{

	_delay_ms(500);
	forward_mm(30);
	unsigned char center_sensor = 0; //stores count from center white line sensor
	unsigned char left_sensor = 0; //stores count from center white line sensor
	unsigned char right_sensor = 0; //stores count from center white line sensor
	unsigned int black_threshold = 140;


	int lastproportional = 0, integral = 0;
	int error = 0;
	float kp = 1.8, ki = 0, kd = 4.75;
	forward();
	while (1) {
		center_sensor = ADC_Conversion(2);
		left_sensor = ADC_Conversion(3);
		right_sensor = ADC_Conversion(1);

		//print_sensor(1,1,3);		//Prints value of White Line Sensor Left
		//print_sensor(1,5,4);		//Prints value of White Line Sensor Center
		//print_sensor(1,9,5);

		unsigned int sum = (center_sensor + left_sensor + right_sensor);

		// lcd_print(1, 13, sum, 3);
		if (sum >= black_threshold) {
			stop();
			_delay_ms(10);
			forward();
			velocity(252, 255);
			stop();
			break;
			} else {
			if (right_sensor <= left_sensor) {
				int position = (((right_sensor * 2) + center_sensor) * 250) / sum;
				int set_point = 250;
				int proportional = position - set_point;
				integral += proportional;
				int derivative = proportional - lastproportional;
				lastproportional = proportional;
				error = (proportional * kp + integral * ki + derivative * kd);
				int left_speed, right_speed;
				//Restricting the error value between +256.
				if (error < -255)
				error = -255;

				if (error > 255)
				error = 255;
				if (error < 0) {
					right_speed = 255 + error;
					left_speed = 255;
				}
				// If error_value is greater than zero calculate left turn values
				else {
					right_speed = 255;
					left_speed = 255 - error;
				}
				forward();
				velocity(right_speed, left_speed);
				} else {
				int position = (((left_sensor * 2) + center_sensor) * 250) / sum;
				int set_point = 250;
				int proportional = position - set_point;
				integral += proportional;
				int derivative = proportional - lastproportional;
				lastproportional = proportional;
				error = (proportional * kp + integral * ki + derivative * kd);
				int left_speed, right_speed;
				//Restricting the error value between +256.
				if (error < -255)
				error = -255;
				if (error > 255)
				error = 255;
				if (error < 0) {
					right_speed = 255;
					left_speed = 255 + error;
				}
				// If error_value is greater than zero calculate left turn values
				else {
					right_speed = 255 - error;
					left_speed = 255;
				}
				forward();
				velocity(right_speed, left_speed);
			}
		}
	}

}
//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; //set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}

void init_devices(void) {
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer1_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	// uart2_init(); // serial communication from PC to robot
	uart0_init(); // serial communication using Zigbee between robots
	sei();   //Enables the global interrupts
}

//Function to Initialize PORTS
void port_init() {
	adc_pin_config();
	motion_pin_config();
	left_encoder_pin_config();
	right_encoder_pin_config();
	buzzer_pin_config();
	lcd_port_config();
	// interrupt_switch_config();
	right_servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	left_servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
	striking_arm_servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation
}

//Function to configure LCD port
void lcd_port_config(void) {
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Column Location.
void print_sensor(char row, char coloumn, unsigned char channel) {
	// lcd_print(row, coloumn, ADC_Conversion(channel), 3);
}

//ADC pin configuration
void adc_pin_config(void) {
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
}


void adc_init() {
	ADCSRA = 0x00;
	ADCSRB = 0x00;        //MUX5 = 0
	ADMUX = 0x20;        //Vref = 5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;        //ADEN = 1 --- ADIE = 1 --- ADPS2:0 = 1 1 0
}

//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch) {
	unsigned char a;
	if (Ch > 7) {
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX = 0x20 | Ch;
	ADCSRA = ADCSRA | 0x40;        //Set start conversion bit
	while ((ADCSRA & 0x10) == 0);    //Wait for ADC conversion to complete
	a = ADCH;
	ADCSRA = ADCSRA | 0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function to configure ports to enable robot's motion
void motion_pin_config(void) {
	DDRA = DDRA | 0x0F;   //set direction of the PORTA 3 to PORTA 0 pins as output
	PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}


// Timer 5 initialized in PWM mode for velocity control
void timer5_init() {
	TCCR5B = 0x00;    //Stop
	TCNT5H = 0xFF;    //Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;    //Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;    //Output compare register high value for Left Motor
	OCR5AL = 0xFF;    //Output compare register low value for Left Motor
	OCR5BH = 0x00;    //Output compare register high value for Right Motor
	OCR5BL = 0xFF;    //Output compare register low value for Right Motor
	OCR5CH = 0x00;    //Output compare register high value for Motor C1
	OCR5CL = 0xFF;    //Output compare register low value for Motor C1
	TCCR5A = 0xA9;    /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
	For Overriding normal port functionality to OCRnA outputs.
	{WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/

	TCCR5B = 0x0B;    //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

//Function for velocity control
void velocity(unsigned char left_motor, unsigned char right_motor) {
	OCR5AL = (unsigned char) left_motor;
	OCR5BL = (unsigned char) right_motor;
}

//Function used for setting motor's direction
void motion_set(unsigned char Direction) {
	unsigned char PortARestore = 0;

	Direction &= 0x0F;        // removing upper nibble for the protection
	PortARestore = PORTA;        // reading the PORTA original status
	PortARestore &= 0xF0;        // making lower direction nibble to 0
	PortARestore |= Direction; // adding lower nibble for forward command and restoring the PORTA status
	PORTA = PortARestore;        // executing the command
}


void forward(void) {
	motion_set(0x06);
}

void stop(void) {
	motion_set(0x00);
}

void back(void) //both wheels backward
{
	motion_set(0x09);
}

void left(void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right(void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}


void left_encoder_pin_config(void) {
	DDRE = DDRE & 0xEF;
	PORTE = PORTE | 0x10;
}

void right_encoder_pin_config(void) {
	DDRE = DDRE & 0xDF;
	PORTE = PORTE | 0x20;
}

void left_position_encoder_interrupt_init(void) {
	cli();
	EICRB = EICRB | 0x02;
	EIMSK = EIMSK | 0x10;
	sei();
}

void right_position_encoder_interrupt_init() {
	cli();
	EICRB = EICRB | 0x08;
	EIMSK = EIMSK | 0x20;
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++; //increment right shaft position count


}

//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++; //increment left shaft position count
}

void angle_rotate(unsigned int Degrees) {
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = (float) Degrees / 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	while (1) {
		if ((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

void left_degrees(unsigned int Degrees) {
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	//velocity(220, 223);
	forward_mm(30);
	left(); //Turn left
	angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees) {
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	//velocity(220, 223);
	forward_mm(30);
	right(); //Turn right
	angle_rotate(Degrees);
}


// buzzer code
void buzzer_pin_config(void) {
	DDRC = DDRC | 0x08;        //Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;        //Setting PORTC 3 logic low to turnoff buzzer
}

void buzzer_on(void) {
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off(void) {
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}


void linear_distance_mm(unsigned int DistanceInMM) {
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	ShaftCountRight = 0;
	while (1) {
		if (ShaftCountRight > ReqdShaftCountInt) {
			break;
		}
	}
	stop();
}

void forward_mm(unsigned int DistanceInMM) {
	forward();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM) {
	back();
	linear_distance_mm(DistanceInMM);
}

void timer1_init(void) {
	TCCR1B = 0x00; //stop
	TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
	TCNT1L = 0x01; //Counter low value to which OCR1xH value is to be compared with
	OCR1AH = 0x03; //Output compare Register high value for servo 1
	OCR1AL = 0xFF; //Output Compare Register low Value For servo 1
	OCR1BH = 0x03; //Output compare Register high value for servo 2
	OCR1BL = 0xFF; //Output Compare Register low Value For servo 2
	OCR1CH = 0x03; //Output compare Register high value for servo 3
	OCR1CL = 0xFF; //Output Compare Register low Value For servo 3
	ICR1H = 0x03;
	ICR1L = 0xFF;
	TCCR1A = 0xAB;
	/*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
	For Overriding normal port functionality to OCRnA outputs.
	{WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
	TCCR1C = 0x00;
	TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor.
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading) {
	float distance;
	unsigned int distanceInt;
	distance = (int) (10.00 * (2799.6 * (1.00 / (pow(adc_reading, 1.1546)))));
	distanceInt = (int) distance;
	if (distanceInt > 800) {
		distanceInt = 800;
	}
	return distanceInt;
}

///////////////////////////////////////////////
// servo
//////////////////////////////////////////

//Configure PORTB 5 pin for servo motor 1 operation
void right_servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void left_servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void striking_arm_servo3_pin_config (void)
{
	DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
	PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void right_servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void left_servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void striking_arm_servo_3(unsigned char degrees)
{
	float PositionServo = 0;
	PositionServo = ((float)degrees / 1.86) + 35.0;
	OCR1CH = 0x00;
	OCR1CL = (unsigned char) PositionServo;
}

//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.

void right_servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

void left_servo_2_free (void) //makes servo 2 free rotating
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}

void striking_arm_servo_3_free (void) //makes servo 3 free rotating
{
	OCR1CH = 0x03;
	OCR1CL = 0xFF; //Servo 3 off
}


//////////////////////////////////////////////





ISR(USART0_RX_vect) // receive mnp list
{
	int current_node, successor_node, data;
	data = UDR0;
	if (data >= 251 && data <= 254)
	{
		flag_rx = data;
	}
	else
	{
		if(flag_rx == 251) // receive mnp list
		{
			if (x == 0)
			{
				MNP_list_length = data;
				x++;
			}
			else
			{
				MNP_list[x-1] = data;
				x++;
			}
		}
		/*if(flag_rx == 252) // receive slave mnp list
		{
		if (y == 0)
		{
		slave_mnp_list_length = data;
		y++;
		}
		else
		{
		MNP_list[y-1] = data;
		y++;
		}
		}*/
		if(flag_rx == 253)
		{
			track_current_mnp_index = data;
			_delay_ms(100);
			// lcd_print(2, 1, track_current_mnp_index, 1);
		}
		if(flag_rx == 254)
		{
			if(data == 100)
			{
				obs_flag = 210;
				_delay_ms(100);
			}
			else if(data == 200)
			{
				obs_flag = 220;
				_delay_ms(100);
			}
			else
			{
				if(obs_flag == 210)
				{
					current_node = data;
					_delay_ms(100);
				}
				else if(obs_flag == 220)
				{
					successor_node = data;
					_delay_ms(100);
					connected_nodes[current_node][successor_node] = '0';
					connected_nodes[successor_node][current_node] = '0';
				}
			}
		}
	}
}


void mnp_assigner(void) {
	// source nodes
	int master_source_node = 0;
	int slave_source_node = 12;

	int master_current_path_length = 0, slave_current_path_length = 0;

	// accumulate previous length of path covered
	int master_total_path_length = 0;
	int slave_total_path_length = 0;
	int i, j = 0, k = 0;
	for (i = 0; i < MNP_list_length; i++) {
		// bot 1: Master
		source_node = master_source_node;
		identify_neighbors_of_MNP(MNP_list[i]);
		master_current_path_length = path_length;
		master_total_path_length += path_length;

		// bot 2: Slave
		source_node = slave_source_node;
		identify_neighbors_of_MNP(MNP_list[i]);
		slave_current_path_length = path_length;
		slave_total_path_length += path_length;

		// compare the TOTAL path lengths
		if (master_total_path_length < slave_total_path_length) {
			// bot 1 is selected for current MNP
			master_mnp_list[j] = MNP_list[i];
			slave_total_path_length -= slave_current_path_length;

			// update source nodes
			master_source_node = destination_node;

			// counter
			master_mnp_list_length++;
			j++;
			} else {
			slave_mnp_list[k] = MNP_list[i];
			master_total_path_length -= master_current_path_length;

			// update source nodes
			slave_source_node = destination_node;

			// counter
			slave_mnp_list_length++;
			k++;
		}

	}

	//printf("master total path length: %d\n", master_total_path_length);
	//printf("slave total path length: %d\n", slave_total_path_length);
}


void slave_com()
{
	buzzer_on();
	_delay_ms(2000);
	buzzer_off();
	int slave_index = 0;
	track_current_mnp_index = 0;
	// go to master_mnp_list[j]

	for(slave_index = 0; slave_index < slave_mnp_list_length; slave_index ++)
	{
		lcd_print(2, 1 , slave_mnp_list[slave_index], 2 );
		identify_neighbors_of_MNP(slave_mnp_list[slave_index]);
		bot_traversal();
		while (1)
		{			
			if(slave_mnp_list[slave_index] == MNP_list[track_current_mnp_index])
			{
				//play_note(slave_mnp_list[slave_index]);
				buzzer_on();
				_delay_ms(2000);
				buzzer_off();
				track_current_mnp_index++;  // now select next mnp from MNP_list
				flag_tx = 253;
				UDR0 = 253;
				_delay_ms(100);
				UDR0 = track_current_mnp_index; // send current mnp to slave robot
				_delay_ms(100);
				break;
			}
			// lcd_print(1,5, track_current_mnp_index, 2);
			_delay_ms(100);
		}
		
	}
}
/*
* Serial communication from PC to robot
*
*/
/*
//Function To Initialize UART2
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart2_init(void)
{
UCSR2B = 0x00; //disable while setting baud rate
UCSR2A = 0x00;
UCSR2C = 0x06;
UBRR2L = 0x5F; //set baud rate lo
UBRR2H = 0x00; //set baud rate hi
UCSR2B = 0x98;
}

int counter = 0;

ISR(USART2_RX_vect)
{
int data = UDR2; // local
if (counter == 0)
{
MNP_list_length = data; // get MNP_list_length
int MNP_list[MNP_list_length]; // declare the MNP_list array
}
else
{
MNP_list[counter - 1] = data; // load (MNP)data one at a time
}
counter++;
}
*/
/************************************************************************/
/*        Boot key                                                      */
/************************************************************************/
/*
//Function to configure Interrupt switch
void interrupt_switch_config(void)
{
DDRE = DDRE & 0x7F;  //PORTE 7 pin set as input
PORTE = PORTE | 0x80; //PORTE7 internal pull-up enabled
}
*/
/*
* Function Name:	main
* Input:			void
* Output:			returns 0 if main is successfully executed without any errors
* Logic:			1) Loops through the MNP_list
*                  2) calculate the shortest path to the MNP
*					3) traverse to the node nearest to MNP
*					4) Turns on the buzzer at the MNP
*                  5) Turns on the continuous buzzer to indicate end of the task
* Example Call:	automatically called by the micro-controller
*
*/
int main(void)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	
	while (1)
	{
		if((PINE & 0x80) != 0x80) //switch is pressed
		{
			buzzer_on();
			_delay_ms(500);
			buzzer_off();

			break;
		}
	}
	
	// initialize slave mnp list
	for (int i = 0; i < 20; i++)
	{
		slave_mnp_list[i] = 0;
	}
	slave_mnp_list_length = 0;

	// initialize mnp list
	for (int i = 0; i < 30; i++)
	{
		MNP_list[i] = 0;
	}
	MNP_list_length = 0;
	
	// check that reception is proper
	// Check for MNP list reception
	
	
	
	while(1)
	{
		if (flag_rx == 251)
		{
			break;
		}
		_delay_ms(500);
	}
	
	_delay_ms(2000);
	while (1)
	{
		if (MNP_list_length != 0)
		{
			break;
		}
		
		_delay_ms(500);
		
	}
	
	// lcd_print(2, 1, MNP_list_length, 1);

	_delay_ms(2000);

	while(1)
	{
		if (x > MNP_list_length)
		{
			break;
		}
		_delay_ms(500);
	}
	/*for (int i = 0; i < MNP_list_length; i++)
	{
	lcd_print(2, 1+i, MNP_list[i], 1);
	_delay_ms(1000);
	}*/
	
	
	lcd_set_4bit();
	lcd_init();
	

	// Check for slave MNP list reception
	/*
	while(1)
	{
	if (flag_rx == 252)
	{
	break;
	}
	_delay_ms(500);
	}

	while (1)
	{
	if (slave_mnp_list_length != 0)
	{
	break;
	}
	else
	_delay_ms(500);
	
	}

	lcd_print(1, 1, slave_mnp_list_length, 2);
	
	_delay_ms(2000);

	while(1)
	{
	if (y > slave_mnp_list_length)
	{
	break;
	}
	_delay_ms(500);
	}
	for (int i = 0; i < slave_mnp_list_length; i++)
	{
	lcd_print(2, 1+i, slave_mnp_list[i], 1);
	_delay_ms(1000);
	}
	
	*/
	
	mnp_assigner();
	for(int i = 0; i < slave_mnp_list_length; i++)
	{
		lcd_print(1, 1+i*2, slave_mnp_list[i], 2);
	}
	source_node = 12; // The bot initially starts from Start 2(i.e. node 12)
	right_servo_1(5);
	_delay_ms(1000);
	left_servo_2(185);
	_delay_ms(1000);
	slave_com();
	_delay_ms(2000);
	buzzer_on();
	_delay_ms(5000);
	buzzer_off();
	return 0;

}




/*
* Function Name:	find_shortest_path
* Input:			None
* Output:			void
* Logic:			finds the shortest path between 2 nodes and stores the path and path length in the global variable
*
* Example Call:	find_shortest_path()
*
*/

void find_shortest_path() {
	// initialize path array
	for (int i = 0; i < MAX; i++) {
		path[i] = 0;
	}
	path_length = 0; // initialize path length
	dijkstra(source_node);
	find_path(source_node, destination_node);
}


/*
* Function Name:	bot_traversal
* Input:			None
* Output:			void
* Logic:			contains logic for the bot to follow the path according to the angles specified
*					1) Read first node from the path array
*					2) Calculate angle required by the bot to reach the node and rotate accordingly
*					3) After rotation check for the obstacle
*					4)		If obstacle found then re-calculate the path and goto step 1
*					5) Traverse to the next node
*
* Example Call:	bot_traversal()
{
*
*/
void bot_traversal() {
	HERE:
	for (int k = 0, j = path_length; k < path_length; k++, j--) {
		path_angle = rotation_angles[path[j]][path[j - 1]]; // Read the angle of the path from rotation_angles matrix
		int bot_rotate_angle = abs(path_angle - current_bot_angle); // calculate the rotation angle
		lcd_print(2,6,bot_rotate_angle,3);
		if (bot_rotate_angle > 15) // if angle is not greater than 15 degrees then don't rotate
		{
			if (current_bot_angle > path_angle)
			{
				// rotate sharp sensor left
				
				
				if(bot_rotate_angle > 180)
				{
					right_servo_1(360 - bot_rotate_angle);
					_delay_ms(1500);
					if (check_path(path[j], path[j - 1], j)) // check for obstacle between the current and next nodes
					{
						right_servo_1(5);
						_delay_ms(1500);
						goto HERE;
					}
					right_servo_1(5);
					_delay_ms(1500);
					turn_right(360 - (bot_rotate_angle - 30));
				}
				
				else
				{
					left_servo_2(180 - bot_rotate_angle);
					_delay_ms(1500);
					if (check_path(path[j], path[j - 1], j)) // check for obstacle between the current and next nodes
					{
						left_servo_2(185);
						_delay_ms(1500);
						goto HERE;
					}
					left_servo_2(185);
					_delay_ms(1500);
					turn_left(bot_rotate_angle - 30);
				}
				
			}
			else
			{
				// rotate sharp sensor right

				if(bot_rotate_angle > 180)
				{
					left_servo_2(360 - (180 - bot_rotate_angle));
					_delay_ms(1500);
					if (check_path(path[j], path[j - 1], j)) // check for obstacle between the current and next nodes
					{
						left_servo_2(185);
						_delay_ms(1500);
						goto HERE;
					}
					left_servo_2(185);
					_delay_ms(1500);
					turn_left(360 - (bot_rotate_angle - 30));
				}
				
				else
				{
					right_servo_1(bot_rotate_angle);
					_delay_ms(1500);
					if (check_path(path[j], path[j - 1], j)) // check for obstacle between the current and next nodes
					{
						right_servo_1(5);
						_delay_ms(1500);
						goto HERE;
					}
					right_servo_1(5);
					_delay_ms(1500);
					turn_right(bot_rotate_angle - 30);
				}
			}
		}
		else
		{
			if (check_path(path[j], path[j - 1], j)) // check for obstacle between the current and next nodes
			{
				goto HERE;
			}
		}
		current_bot_angle = path_angle; // set the current angle of robot to the path angle
		follow(); // traverse to the next node
		_delay_ms(500);
	}

	source_node = path[0];

	return;
}


/*
* Function Name:	check_path
* Input:			current node, successor node
* Output:			returns 1 if obstacle if present, 0 if obstacle is absent
* Logic:			checks if there is any obstacle present in the path of the bot (i.e between current node & next node)
*
* Example Call:	check_path(current node, successor node)
*
*/

int check_path(int current_node, int successor_node, int j) {
	unsigned char sharp;
	unsigned int distance_mm = 0; // stores the distance obtained from the sharp IR sensor in millimeters
	int k = 0, sharp_threshold;
	while (k < 5) {
		sharp = ADC_Conversion(11);
		distance_mm += Sharp_GP2D12_estimation(sharp);
		k++;
		_delay_ms(50);
	}
	distance_mm /= 5;
	lcd_print(2, 13, distance_mm, 3);

	// Since the path between nodes 37 and 40, 25 and 28, 31 and 34 is twice the length of other path the distance between the bot and the object
	// will be greater. Hence, setting higher sharp threshold  for these paths
	if ((current_node == 37 || current_node == 40 || current_node == 25 || current_node == 28 || current_node == 31 || current_node == 34)
	&& (successor_node == 37 || successor_node == 40 || successor_node == 25 || successor_node == 28 || successor_node == 31 || successor_node == 34)) {
		sharp_threshold = 600;
		} else {
		sharp_threshold = 100;
	}

	if (distance_mm <= sharp_threshold) {
		// removing path between the nodes with obstacle
		flag_tx = 254; // denotes obstacle
		UDR0 = flag_tx;
		_delay_ms(100);
		connected_nodes[current_node][successor_node] = '0';
		connected_nodes[successor_node][current_node] = '0';
		UDR0 = 100; // flag to send current node
		_delay_ms(100);
		UDR0 = current_node;
		_delay_ms(100);
		UDR0 = 200;	// flag to send successor nod
		_delay_ms(100);
		UDR0 = successor_node;
		_delay_ms(100);
		// setting current node as the source node and re-calculating the path to the destination MNP
		source_node = path[j];
		find_shortest_path();

		back_mm(30); // go back to avoid crashing with the obstacle
		return 1;
	}
	return 0;
}


/*
* Function Name:	identify_neighbors_of_MNP
* Input:			index of current MNP
* Output:			void
* Logic:			Finds the list of the closest nodes(neighbors) of the MNP.
*
* Example Call:	identify_neighbors_MNP(index of current MNP)
*
*/

void identify_neighbors_of_MNP(int mnp) {
	int a[6];
	switch (mnp) {
		case 25:
		a[0] = 0;
		a[1] = 39;
		a[2] = 40;
		a[3] = 41;
		find_optimal_path_for_MNP_with_several_neighbor_nodes(a, 4);
		break;
		case 26:
		a[0] = 24;
		a[1] = 25;
		a[2] = 40;
		a[3] = 41;
		a[4] = 42;
		a[5] = 43;
		find_optimal_path_for_MNP_with_several_neighbor_nodes(a, 6);
		break;
		case 27:
		a[0] = 4;
		a[1] = 24;
		a[2] = 25;
		a[3] = 26;
		find_optimal_path_for_MNP_with_several_neighbor_nodes(a, 4);
		break;
		case 28:
		a[0] = 8;
		a[1] = 27;
		a[2] = 28;
		a[3] = 29;
		find_optimal_path_for_MNP_with_several_neighbor_nodes(a, 4);
		break;
		case 29:
		a[0] = 28;
		a[1] = 29;
		a[2] = 30;
		a[3] = 31;
		a[4] = 44;
		a[5] = 45;
		find_optimal_path_for_MNP_with_several_neighbor_nodes(a, 6);
		break;
		case 30:
		a[0] = 12;
		a[1] = 30;
		a[2] = 31;
		a[3] = 32;
		find_optimal_path_for_MNP_with_several_neighbor_nodes(a, 4);
		break;
		case 31:
		a[0] = 16;
		a[1] = 33;
		a[2] = 34;
		a[3] = 35;
		find_optimal_path_for_MNP_with_several_neighbor_nodes(a, 4);
		break;
		case 32:
		a[0] = 34;
		a[1] = 35;
		a[2] = 36;
		a[3] = 37;
		a[4] = 46;
		a[5] = 47;
		find_optimal_path_for_MNP_with_several_neighbor_nodes(a, 6);
		break;
		case 33:
		a[0] = 20;
		a[1] = 36;
		a[2] = 37;
		a[3] = 38;
		find_optimal_path_for_MNP_with_several_neighbor_nodes(a, 4);
		break;
		default: // For MNPs on the circular path there is only one nearest node
		destination_node = mnp - 1; // Since the node is indexed from 0, the node corresponding to MNP on the circular path will be one less than the MNP number
		find_shortest_path();
	}

}

/*
* Function Name:	find_optimal_path_for_MNP_with_several_neighbor_nodes
* Input:			array of nearest neighbors of MNP, length of the array
* Output:			void
* Logic:			Finds the shortest path from amongst the closest neighbors of MNP
*
* Example Call:	find_optimal_path(array of nearest neighbors of MNP, length of array)
*
*/

void find_optimal_path_for_MNP_with_several_neighbor_nodes(int neighbor_nodes[], int neighbor_nodes_array_length) {
	int index_of_shortest_path = 0;
	// calculate path for the first nearest node
	destination_node = neighbor_nodes[0];
	find_shortest_path(); // calculate shortest path
	int current_minimum_length = path_length;

	// calculate path starting from second nearest node
	for (int i = 1; i < neighbor_nodes_array_length; i++) {
		destination_node = neighbor_nodes[i];
		find_shortest_path();
		if (path_length < current_minimum_length) {
			current_minimum_length = path_length;
			index_of_shortest_path = i; // if the current path is shorter than previous one then store its index
		}
	}
	// re-calculate the path using the index_of_shortest_path
	destination_node = neighbor_nodes[index_of_shortest_path];
	find_shortest_path();
	// now path array stores shortest path among all its neighbors
}


/*
* Function Name:  turn_left
* Input: 		  degree at which robot wants to rotate left
* Output: 		  void
* Logic: 		  1) The robot will turn left by the angle passed as the parameter
*				  2) After turning it will check if center white line sensor is on black line or not
*				  3) If the center white line is not present on the black line then rotate robot left until center white line
*				     comes on the black line
* Example Call:	  turn_left()
*
*/
void turn_left(unsigned int degrees) {
	forward_mm(35);
	unsigned char center = 0;
	velocity(230, 230);
	left_degrees(degrees); // rotate left by degree specified
	center = ADC_Conversion(2); // read of white line sensor
	if (center < 30) //check if center white line sensor is on black line or not
	{
		left(); //if center white line is not on black line then rotate left
		center = ADC_Conversion(2); //read values white line sensor
		while (center < 30) // keep turning left while center white line does not come on black line
		{
			center = ADC_Conversion(2); // while turning keep checking the white line sensor value
		}
	}
	_delay_ms(80);
	right_degrees(5);
	stop(); //stop after turning left
	_delay_ms(500);
}

/*
* Function Name:  turn_right
* Input: 		  degree at which robot wants to rotate right
* Output: 		  void
* Logic: 		  1) The robot will turn left by the angle passed as the parameter
*				  2) After turning it will check if center white line sensor is on black line or not
*				  3) If the center white line is not present on the black line then rotate robot left until center white line
*				     comes on the black line
* Example Call:	  turn_right()
*
*/
void turn_right(unsigned int degrees) {
	forward_mm(35);
	unsigned char center = 0;
	velocity(230, 230);
	right_degrees(degrees); // rotate right by degree specified
	center = ADC_Conversion(2); // read of white line sensor
	if (center < 30) //check if center white line sensor is on black line or not
	{
		right(); //if center white line is not on black line then rotate right
		center = ADC_Conversion(2); //read values white line sensor
		while (center < 30) // keep turning right while center white line does not come on black line
		{
			center = ADC_Conversion(2); // while turning keep checking the white line sensor value
		}
	}
	_delay_ms(80);
	left_degrees(5);
	stop(); //stop after turning left
	_delay_ms(500);
}


void play_note(int mnp)
{
	int rotation_angle = cal_striking_angle(mnp, source_node);
	int sharp_threshold = 150;
	unsigned char sharp;
	unsigned int distance_mm;
	int servo1_moved = 0, servo2_moved = 0;
	rotation_angle = abs(rotation_angle - current_bot_angle); // calculate the rotation angle
	if (current_bot_angle > rotation_angle)
	{
		// rotate servo motor left by rotation_angle
		left_servo_2(rotation_angle);
		servo2_moved = 1;
	}
	else
	{
		// rotate servo motor right by rotation_angle
		right_servo_1(rotation_angle);
		servo1_moved = 1;
	}
	sharp = ADC_Conversion(11);
	distance_mm = Sharp_GP2D12_estimation(sharp);
	if(distance_mm <= sharp_threshold)
	{
		striking_arm_servo_3(10); // hit the pipe
	}
	else
	{
		//rotate 10 degree to right
		if(servo1_moved)
		{
			right_servo_1(rotation_angle + 5); //rotate 5 degree to right
			sharp = ADC_Conversion(11);
			distance_mm = Sharp_GP2D12_estimation(sharp);

			if(sharp_threshold < distance_mm)
			striking_arm_servo_3(10);// hit
			else
			{
				right_servo_1(rotation_angle - 5);// rotate 10 degree to left
				striking_arm_servo_3(10);// hit
			}
		}
		
		if(servo2_moved)
		{
			left_servo_2(rotation_angle + 5); //rotate 5 degree to right
			sharp = ADC_Conversion(11);
			distance_mm = Sharp_GP2D12_estimation(sharp);

			if(sharp_threshold < distance_mm)
			striking_arm_servo_3(10);// hit
			else
			{
				left_servo_2(rotation_angle - 5);// rotate 10 degree to left
				striking_arm_servo_3(10);// hit
			}
			
		}
	}
	striking_arm_servo_3(100); // move servo motor 3 to initial position
}
int cal_striking_angle(int mnp, int node)
{
	switch (mnp)
	{
		case 25:
		switch(node)
		{
			case 0: return 90;
			case 39:return 330;
			case 40:return 270;
			case 41:return 210;
		}
		case 26:
		switch(node)
		{
			case 24:return 150;
			case 25:return 210;
			case 40:return 30;
			case 41:return 90;
		}
		case 27:
		switch(node)
		{
			case 4:return 150;
			case 24:return 30;
			case 25:return 330;
			case 26:return 270;
		}
		case 28:
		switch(node)
		{
			case 8:return 210;
			case 27:return 90;
			case 28:return 30;
			case 29:return 330;
		}
		case 29:
		switch(node)
		{
			case 28:return 150;
			case 29:return 210;
			case 30:return 270;
			case 31:return 330;
		}
		
		case 30:
		switch(node)
		{
			case 12:return 270;
			case 30:return 150;
			case 31:return 90;
			case 32:return 30;
		}

		case 31:
		switch(node)
		{
			case 16:return 330;
			case 33:return 210;
			case 34:return 150;
			case 35:return 90;
		}
		
		case 32:
		switch(node)
		{
			case 34:return 270;
			case 35:return 330;
			case 36:return 30;
			case 37:return 90;
		}
		
		case 33:
		switch(node)
		{
			case 20:return 30;
			break;
			case 36:return 270;
			break;
			case 37:return 210;
			break;
			case 38:return 150;
			break;
		}
		
		case 7:
		return 0;
		
		case 11:
		return 60;
		
		case 15:
		return 120;

		case 19:
		return 180;

		case 23:
		return 240;
		
		case 3:
		return 300;
		
		default: return 90;  //TO DO
		
	}
}
