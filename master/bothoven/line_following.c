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

        lcd_print(1, 13, sum, 3);
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
    lcd_print(row, coloumn, ADC_Conversion(channel), 3);
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