
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


#include "servo.c"
#include "lcd.c"
#include "line_following.c"
#include "path_calculator.c"

int obs_flag;


int MNP_list[] = {7, 29, 26, 18, 24, 13, 30, 21}; // list of MNPs the robot has to visit
#define MNP_list_length 8 // total number of MNPs in MNP_list
int master_mnp_list[MNP_list_length];
int slave_mnp_list[MNP_list_length];

int master_mnp_list_length = 0, slave_mnp_list_length = 0;

int source_node; // start node, indexed from 0
int destination_node; // stores the node near to the destination MNP
int current_bot_angle = 0; // stores the current angle of bot
int path_angle = 0; //stores the angle of the next path
int track_current_mnp_index; // holds the mnp corresponding to the next node to be played
int flag_tx; // used to transmit data b/w master and slave robots
int flag_rx; // used to receive data
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
void master_com();

ISR(USART0_RX_vect)
{
	int current_node, successor_node;
	int data = UDR0;
	if (data >= 251 && data <= 254)
	{
		flag_rx = data;
	}
	else
	{
		if(flag_rx == 253)
		{
			track_current_mnp_index = data;
			
			// lcd_print(2, 1, track_current_mnp_index, 1);
		}
		if(flag_rx == 254)
		{
			if(data == 100)
			obs_flag = 210;
			else if(data == 200)
			obs_flag = 220;
			else
			{
				if(obs_flag == 210)
				{
					current_node = data;
				}
				else if(obs_flag == 220)
				{
					successor_node = data;
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





void master_com() {

	flag_tx = 251;
	UDR0 = 251;
	_delay_ms(100);
	UDR0 = MNP_list_length;
	_delay_ms(100);
	for (int iter = 0; iter < MNP_list_length; iter++) // send mnp_list to master robot
	{
		UDR0 = MNP_list[iter];
		_delay_ms(100);
	}
	_delay_ms(500);

	/*
	flag_tx = 252;
	UDR0 = 252;
	_delay_ms(100);
	UDR0 = slave_mnp_list_length;
	_delay_ms(100);
	for (int iter = 0; iter < slave_mnp_list_length; iter++) 	// send the slave_mnp_list to slave robot
	{
	UDR0 = slave_mnp_list[iter];
	_delay_ms(100);
	}
	*/
	
	int master_index = 0;
	track_current_mnp_index = 0;
	// go to master_mnp_list[j]
	//master_mnp_list_length = MNP_list_length; // REMOVE THIS LATER DURING FINAL RUN
	for (master_index = 0; master_index < master_mnp_list_length; master_index++)
	{
		lcd_print(2, 1, master_mnp_list[master_index], 2);
		identify_neighbors_of_MNP(master_mnp_list[master_index]);
		bot_traversal();
		while (1)
		{
			if (master_mnp_list[master_index] == MNP_list[track_current_mnp_index])
			{
				//play_note(track_current_mnp);//TO Do
				buzzer_on();
				_delay_ms(2000);
				buzzer_off();
				track_current_mnp_index++; // now select next mnp from MNP_list
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
	/*
	// get the MNP_list from the PC
	while (1)
	{
	if (counter > MNP_list_length)
	break;
	}
	*/
	// start the robot when boot key is pressed by akshdeep
	/*
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
	*/

	// _delay_ms(1000); // don't start abruptly, you will injure akshdeep

	// initialize servo positions
	/*
	right_servo_1(5);
	_delay_ms(2000);
	
	left_servo_2(185);
	_delay_ms(2000);

	striking_arm_servo_3(100);
	_delay_ms(2000);
	*/
	// assign MNPs to both the bots
	/*
	mnp_assigner();
	for (int i = 0; i < master_mnp_list_length; i++)
	{
		lcd_print(1, 1+i*2, master_mnp_list[i], 2);
		_delay_ms(1000);
	}
	source_node = 0; // The bot initially starts from Start 1(i.e. node 0)
	master_com();
	buzzer_on();
	_delay_ms(5000);
	buzzer_off();
	*/
	_delay_ms(2000);
	left_servo_2(180);
	_delay_ms(1000);
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
*
*/
void bot_traversal() {
	HERE:
	for (int k = 0, j = path_length; k < path_length; k++, j--)
	{
		path_angle = rotation_angles[path[j]][path[j - 1]]; // Read the angle of the path from rotation_angles matrix
		int bot_rotate_angle = abs(path_angle - current_bot_angle); // calculate the rotation angle
		lcd_print(2,6,bot_rotate_angle,3);
		if (bot_rotate_angle > 15) // if angle is not greater than 15 degrees then don't rotate
		{
			if (current_bot_angle > path_angle)
			{
				// rotate sharp sensor left
				//if (check_path(path[j], path[j - 1], j)) // check for obstacle between the current and next nodes
				//{
				//goto HERE;
				//}
				if(bot_rotate_angle > 180)
				{
					turn_right(360 - (bot_rotate_angle - 30));
				}
				else
				{
					turn_left(bot_rotate_angle - 30);
					
				}
			}
			else
			{
				// rotate sharp sensor right
				//if (check_path(path[j], path[j - 1], j)) // check for obstacle between the current and next nodes
				//{
				//goto HERE;
				//}
				if(bot_rotate_angle > 180)
				{
					turn_left(360 - (bot_rotate_angle - 30));
				}
				else
				{
					turn_right(bot_rotate_angle - 30);
					
				}
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
		UDR0 = 200;	// flag to send successor node
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
		
		default:return 90;
				  //TO DO
		
	}
}

void play_note(int mnp)
{
	int rotation_angle = cal_striking_angle(mnp, source_node);
	int sharp_threshold = 150;
	unsigned char sharp;
	unsigned int distance_mm;
	int servo1_moved = 0, servo2_moved = 0;
	
	if((mnp >= 1 && mnp <= 24) && (mnp != 3 || mnp != 7 || mnp != 11 || mnp != 15 || mnp != 19 || mnp != 23))
	{
		if(((path[0] - path[1]) > 0) && path[0] == 0)
			left_servo_2(90); // SERVO ADJUSTMENT TO DO
		if(((path[0] - path[1]) < 0) && path[0] == 23)
			right_servo_1(90);// SERVO ADJUSTMENT TO DO
	}	
	
	else
	{
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
	}
	striking_arm_servo_3(100); // move servo motor 3 to initial position
}