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


ISR(USART0_RX_vect) // receive mnp list
{
	int current_node, successor_node;
	if (UDR0 >= 97 && UDR0 <= 122)
	{
		flag = UDR0;
	}
	else
	{
		if(flag == 'c')
		{
			data = UDR0;
			track_current_mnp = data;
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

void master_com()
{
	// assign MNPs to both the bots
	mnp_assigner();
	
	for (int iter = 0; iter < MNP_list_length; iter++) // send mnp_list to master robot
	{
		if(!iter)
		{
			flag = 'a';
			UDR0 = flag;
		}
		UDR0 = MNP_list[iter];
		_delay_ms(100);
	}
	for (int iter = 0; iter < master_mnp_list_length; iter++) 	// send the master_mnp_list to slave robot
	{
		if(!iter)
		{
			flag = 'b';
			UDR0 = flag;
		}
		UDR0 = master_mnp_list[iter];
		_delay_ms(100);
	}
	int i = 0, master_index = 0;
	track_current_mnp = MNP_list[i];
	// go to master_mnp_list[j]
	
	for(master_index = 0; master_index < master_mnp_list_length; master_index ++)
	{
		traverse(master_index);
		while (1)
		{
			if(master_mnp_list[master_index] == track_current_mnp)
			{
				play_note(master_mnp_list[master_index]);
				master_index++;
				track_current_mnp = MNP_list[++i]; // now select next mnp from MNP_list
				flag = 'c';
				UDR0 = flag;
				UDR0 = track_current_mnp; // send current mnp to slave robot
				break;
			}
		}
	}


}