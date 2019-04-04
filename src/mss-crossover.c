#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>

#define CHANNEL0_ON_DELAY     0x00
#define CHANNEL0_OFF_DELAY    0x01
#define CHANNEL0_THRESHOLD_H  0x02
#define CHANNEL0_THRESHOLD_L  0x03

volatile uint8_t eventFlags = 0;
#define EVENT_DO_BD_READ        0x01
#define EVENT_DO_ADC_RUN        0x02
#define EVENT_1HZ_BLINK         0x04
#define EVENT_CALIBRATION_MODE  0x08
#define EVENT_TICK              0x10

#define CALIBRATION_CYCLES        16
#define CALIBRATION_BUTTON_DELAY  20

uint8_t detectorOnDelayCount = 0;
uint8_t detectorOffDelayCount = 0;
uint8_t detectorOnDelay = 4;
uint8_t detectorOffDelay = 25;

volatile uint16_t adcValue[2];
#define ADC_CHANNEL_DETECTOR_0  0
#define ADC_CHANNEL_SETPOINT_0  1

void initializeADC()
{
	for(uint8_t i=0; i<sizeof(adcValue)/sizeof(adcValue[0]); i++)
		adcValue[i] = 0;

	// Setup ADC for bus voltage monitoring
	ADMUX  = 0x40;  // AVCC reference, ADC0 starting channel
	ADCSRA = _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 64 prescaler, ~8.3kconv / s
	ADCSRB = 0x00; // Free running mode
	DIDR0  = 0x01;  // Turn ADC pins 0 into analog input
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
}

uint16_t readThresholdCalibration()
{
	return ((uint16_t)eeprom_read_byte((uint8_t*)(CHANNEL0_THRESHOLD_H)))<<8 | (uint16_t)(eeprom_read_byte((uint8_t*)(CHANNEL0_THRESHOLD_L)));
}

void writeThresholdCalibration(uint16_t threshold)
{
	eeprom_write_byte((uint8_t*)(CHANNEL0_THRESHOLD_H), 0x03 & (threshold>>8));
	eeprom_write_byte((uint8_t*)(CHANNEL0_THRESHOLD_L), 0xFF & threshold);
}

void initializeDelays()
{
	detectorOnDelay = eeprom_read_byte((uint8_t*)(CHANNEL0_ON_DELAY));
	if (0xFF == detectorOnDelay || 0x00 == detectorOnDelay)
	{
		eeprom_write_byte((uint8_t*)(CHANNEL0_ON_DELAY), 4);
		detectorOnDelay = eeprom_read_byte((uint8_t*)(CHANNEL0_ON_DELAY));
	}	


	detectorOffDelay = eeprom_read_byte((uint8_t*)(CHANNEL0_OFF_DELAY));
	if (0xFF == detectorOffDelay || 0x00 == detectorOffDelay)
	{
		eeprom_write_byte((uint8_t*)(CHANNEL0_OFF_DELAY), 25);
		detectorOffDelay = eeprom_read_byte((uint8_t*)(CHANNEL0_OFF_DELAY));
	}

	adcValue[ADC_CHANNEL_SETPOINT_0] = readThresholdCalibration();
	if (adcValue[ADC_CHANNEL_SETPOINT_0] > 1023 || 0x00 == adcValue[ADC_CHANNEL_SETPOINT_0])
	{
		writeThresholdCalibration(0x001D); // Arbitrary - roughly a 10k resistor on my test track
		adcValue[ADC_CHANNEL_SETPOINT_0] = readThresholdCalibration();
	}
}

ISR(ADC_vect)
{
	static uint8_t workingChannel = 0;
	static uint16_t accumulator = 0;
	static uint8_t count = 0;

	eventFlags &= ~EVENT_DO_ADC_RUN;

	accumulator += ADC;
	if (++count >= 64)
	{
		adcValue[workingChannel] = accumulator / 64;
		accumulator = 0;
		count = 0;
		//ADMUX = (ADMUX & 0xF0) | (workingChannel & 0x01);
		eventFlags |= EVENT_DO_BD_READ;
	}

	if (0 == (eventFlags & EVENT_DO_BD_READ))
	{
		// Trigger the next conversion.  Not using auto-trigger so that we can safely change channels
		ADCSRA |= _BV(ADSC);
	}
}


void initialize10HzTimer()
{
	// Set up timer 1 for 10Hz interrupts
	TCNT1 = 0;
	OCR1A = 0x0C34;
	TCCR1A = 0;
	TCCR1B = _BV(WGM12) | _BV(CS12);
	TCCR1C = 0;
	TIFR1 |= _BV(OCF1A);
	TIMSK1 |= _BV(OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
	static uint8_t cntr = 0;
	
	eventFlags |= EVENT_DO_ADC_RUN;

	if (++cntr >= 5)
	{
		eventFlags ^= EVENT_1HZ_BLINK;
		cntr = 0;
	}
}

uint8_t debounceInputs(uint8_t* ioState, uint8_t rawInput)
{
	static uint8_t clock_A=0, clock_B=0;
	uint8_t delta = rawInput ^ *ioState;
	uint8_t changes;

	clock_A ^= clock_B;                     //Increment the counters
	clock_B  = ~clock_B;
	clock_A &= delta;                       //Reset the counters if no changes
	clock_B &= delta;                       //were detected.
	changes = ~((~delta) | clock_A | clock_B);
	*ioState ^= changes;	

	return(changes);
}

uint8_t detectorStatus = 0;

void processDetector(void)
{	
	uint16_t threshold = adcValue[ADC_CHANNEL_SETPOINT_0];

	// Introduce a bit of hysteresis. 
	// If the channel is currently "on", lower the threshold by 5 counts
	// If the channel is currently "off", raise the threshold by 5
	if ((0 != detectorStatus) && (threshold >= 5))
	{
		threshold -= 5;
	} 
	else if ((0 == detectorStatus) && (threshold <= (1023-5)) )
	{
		threshold += 5;
	}
	
	if (adcValue[ADC_CHANNEL_DETECTOR_0] > threshold)
	{
		// Current for the channel has exceeded threshold
		if (0 == detectorStatus)
		{
			// Channel is currently in a non-detecting state
			// Wait for the turnon delay before actually indicating on
			if (detectorOnDelayCount < detectorOnDelay)
				detectorOnDelayCount++;
			else
			{
				detectorStatus = 1;
				detectorOffDelayCount = 0;
			}
		} else {
			// Channel is currently in a detecting state
			detectorOffDelayCount = 0;
		}
	} else {
		// Current for the channel is under the threshold value
		if (0 != detectorStatus)
		{
			// Channel is currently in a non-detecting state
			if (detectorOffDelayCount < detectorOffDelay)
				detectorOffDelayCount++;
			else
			{
				detectorStatus = 0;
				detectorOnDelayCount = 0;
			}
		} else {
			// Channel is currently in a detecting state
			detectorOnDelayCount = 0;
		}
	}
}

void setOccupancyOn()
{
	PORTA |= _BV(PA3);
}

void setOccupancyOff()
{
	PORTA &= ~_BV(PA3);
}

void setOccupancyLEDOn()
{
	PORTD |= _BV(PD3);
}

void setOccupancyLEDOff()
{
	PORTD &= ~_BV(PD3);
}

void setAuxLEDOn()
{
	PORTD |= _BV(PD4);
}

void setAuxLEDOff()
{
	PORTD &= ~_BV(PD4);
}


int main(void)
{
	uint8_t auxDetectInputState = 0;
	uint8_t debouncedInputs = 0;
	uint8_t rawInputs = 0;
	uint8_t calibrationSwitchCountdown = CALIBRATION_BUTTON_DELAY;
	uint8_t calibrationCycles = CALIBRATION_CYCLES;
	uint32_t calibrationAccumulator = 0;
	// Deal with watchdog first thing
	
	MCUSR = 0; // Clear reset status
	wdt_enable(WDTO_1S);

	// Initialize ports 
	// Pin Assignments for PORTA/DDRA
	//  PA0 - Auxilliary occupancy input (inverted)
	//  PA1 - Not used
	//  PA2 - Not used
	//  PA3 - Occupancy transistor enable
	//  PA4 - N/A (doesn't exist)
	//  PA5 - N/A (doesn't exist)
	//  PA6 - N/A (doesn't exist)
	//  PA7 - N/A (doesn't exist)
	DDRA  = 0b00001110;
	PORTA = 0b00000001;

	// Pin Assignments for PORTB/DDRB
	//  PB0 - Not used
	//  PB1 - Not used
	//  PB2 - Not used
	//  PB3 - MOSI (in,pulled up)
	//  PB4 - MISO (in)
	//  PB5 - SCK (in)
	//  PB6 - Not used
	//  PB7 - Not used
	DDRB  = 0b11000111;
	PORTB = 0b00111000;

	// Pin Assignments for PORTC/DDRC
	//  PC0 - Analog - Detector 0 current sense
	//  PC1 - Digital - Detector 0 set button (pullup on)
	//  PC2 - Not used
	//  PC3 - Not used
	//  PC4 - Not used
	//  PC5 - Not used
	//  PC6 - N/A (/RESET pin)
	//  PC7 - Not used
	DDRC  = 0b11111100;
	PORTC = 0b00000010;

	
	// Pin Assignments for PORTD/DDRD
	//  PD0 - Not used
	//  PD1 - Not used
	//  PD2 - Not used
	//  PD3 - Occupancy Detect LED (red, out)
	//  PD4 - Aux onboard LED (amber, out)
	//  PD5 - N/A (doesn't exist)
	//  PD6 - N/A (doesn't exist)
	//  PD7 - N/A (doesn't exist)
	DDRD  = 0b00011111;
	PORTD = 0b00000000;

	setOccupancyOff();
	setOccupancyLEDOff();
	setAuxLEDOff();

	initialize10HzTimer();
	initializeADC();
	initializeDelays();

	sei();

	while(1)
	{
		wdt_reset();
		
		// If the ISR tells us it's time to run the ADC again and we've handled the last read, 
		// start the ADC again
		if ((eventFlags & EVENT_DO_ADC_RUN) && !(eventFlags & EVENT_DO_BD_READ))
		{
			ADCSRA |= _BV(ADSC);
		}

		// If we're in cal mode, do special stuff
		if (eventFlags & EVENT_CALIBRATION_MODE)
		{
			setOccupancyOff();
			
			if (calibrationCycles & 0x02)
			{
				setAuxLEDOff();
				setOccupancyLEDOn();
			} else {
				setAuxLEDOn();
				setOccupancyLEDOff();
			}
			
			if (eventFlags & EVENT_DO_BD_READ)
			{
				if (calibrationCycles)
				{
					calibrationCycles--;
					calibrationAccumulator += adcValue[ADC_CHANNEL_DETECTOR_0];
				} else {
					// last calibration cycle, store values
					uint32_t threshold = (calibrationAccumulator) / (CALIBRATION_CYCLES);
					uint32_t ninetyPercentThreshold = (9 * calibrationAccumulator) / (10 * CALIBRATION_CYCLES);
					
					if (threshold - ninetyPercentThreshold < 8 && threshold >= 9) // 5 is the hysteresis threshold
						threshold -= 8;
					else 
						threshold = ninetyPercentThreshold;

					writeThresholdCalibration(threshold);
					adcValue[ADC_CHANNEL_SETPOINT_0] = readThresholdCalibration();
					eventFlags &= ~(EVENT_CALIBRATION_MODE);
					setAuxLEDOff();
					setOccupancyLEDOff();
				}
				eventFlags &= ~(EVENT_DO_BD_READ);
			}
			continue;
		}


		// If all the analog inputs have been read, the flag will be set and we
		// can then process the analog detector inputs
		if (eventFlags & EVENT_DO_BD_READ)
		{
			// Read the auxilliary occupancy input and the calibration switch
			// It's inverted, so flip the bit around so that auxDetectInputState is
			// positive logic 

			// Clear the flag and start the next chain of conversions
			eventFlags &= ~(EVENT_DO_BD_READ);

			rawInputs = ((~PINA) & 0x01) | ((~PINC) & 0x02);
			debounceInputs(&debouncedInputs, rawInputs);

			if (debouncedInputs & 0x02)
			{
				// If the calibration switch is down
				if (0 != calibrationSwitchCountdown)
				{
					calibrationSwitchCountdown--;
					setAuxLEDOff();
				} else {
					setAuxLEDOn();
				}
			} else {
				// Button's up, 
				if (0 == calibrationSwitchCountdown)
				{
					eventFlags |= EVENT_CALIBRATION_MODE;
					calibrationAccumulator = 0;
					setAuxLEDOff();
				}
				calibrationSwitchCountdown = CALIBRATION_BUTTON_DELAY;
				calibrationCycles = CALIBRATION_CYCLES;
			}

			// Normal system behaviour - get the detector, do the analog
			// hysteresis, set output driver

			// Do all the analog magic
			processDetector();
			auxDetectInputState = debouncedInputs & 0x01;

			if (detectorStatus || auxDetectInputState)
				setOccupancyOn();
			else
				setOccupancyOff();
		}

		// This should provide roughly a 1Hz blink rate
		// It provides a nice proof of life that the ADC is actually doing its thing and is alive.

		if (eventFlags & EVENT_1HZ_BLINK)
		{
			if (CALIBRATION_BUTTON_DELAY == calibrationSwitchCountdown)
				setAuxLEDOff();

			if (detectorStatus)
				setOccupancyLEDOn();
			else if (auxDetectInputState)
				setOccupancyLEDOff();				
			else
				setOccupancyLEDOff();
		}
		else
		{
			if (CALIBRATION_BUTTON_DELAY == calibrationSwitchCountdown)
				setAuxLEDOn();

			if (detectorStatus)
				setOccupancyLEDOn();
			else if (auxDetectInputState)
				setOccupancyLEDOn();
			else
				setOccupancyLEDOff();
		}
	}
}

