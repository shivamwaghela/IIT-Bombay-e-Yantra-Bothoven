/*
 * slave_communication.c
 *
 * Created: 15-02-2017 12:35:25
 *  Author: Swaminarayan
 */ 
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

int x = 0, y = 0;
int MNP_list_length;
char obs_flag;

ISR(USART0_RX_vect) // receive mnp list
{
	int current_node, successor_node;
	if (UDR0 >= 97 && UDR0 <= 122)
	{
		flag = UDR0;
	}
	else
	{
		if(flag == 'a') // receive mnp list
		{
			data = UDR0;
			if (x == 0)
			{
				MNP_list_length = data;
				int MNP_list[MNP_list_length];
				x++;
			}
			else
			{
				MNP_list[--x] = data;
				x++;
			}
		}
		if(flag == 'b') // receive slave mnp list
		{
			data = UDR0;
			if (y == 0)
			{
				slave_mnp_list_length = data;
				int slave_mnp_list[slave_mnp_list_length];
				y++;
			}
			else
			{
				MNP_list[--y] = data;
				y++;
			}
		}
		if(flag == 'c')
		{
			track_current_mnp = UDR0;
		}
		if(flag == 'd')
		{
			if(UDR0 == 100)
			obs_flag = 'C';
			else if(UDR0 == 200)
			obs_flag = 'S';
			else
			{
				if(obs_flag == 'C')
				{
					current_node = UDR0;
				}
				else if(obs_flag == 'S')
				{
					successor_node = UDR0;
					connected_nodes[current_node][successor_node] = '0';
					connected_nodes[successor_node][current_node] = '0';
				}
			}
		}
	}
}
}

void slave_com()
{
	int i = 0, slave_index = 0;
	track_current_mnp = MNP_list[i];
	// go to master_mnp_list[j]

	for(slave_index = 0; slave_index < slave__mnp_list_length; slave_index ++)
	{
		traverse(slave_index);
		while (1)
		{
			if(slave_mnp_list[slave_index] == track_current_mnp)
			{
				play_note(slave_mnp_list[slave_index]);
				slave_index++;
				track_current_mnp = MNP_list[++i] // now select next mnp from MNP_list
				flag = 'c';
				UDR0 = flag;
				UDR0 = track_current_mnp; // send current mnp to slave robot
				break;
			}
		}
	}

	


}
