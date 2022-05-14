/*
 *              AtTiny 84 Development 
 *              Tiny Slope Detector
 *              Vernon Billingsley 2022
 *              
 *              A gate generator that reads an incoming
 *            CV and set one gate high on rising cv and one
 *            gate high on falling cv and one gate high on 
 *            a steady state..A sense pot sets the time of 
 *            the state change...
 *          
 *              
 *                    ----U--- 
 *              VCC -| 1    14|- GND
 *              PB0 -| 2    13|- PA0
 *              PB1 -| 3    12|- PA1
 *              PB3 -| 4    11|- PA2
 *              PB2 -| 5    10|- PA3
 *              PA7 -| 6     9|- PA4
 *              PA6 -| 7     8|- PA5
 *                    --------
 *                    
 *            Pin       Function        
 *            1         Vcc
 *            2         Rising CV
 *            3         Steady CV
 *            4         Rest
 *            5         Falling
 *            6
 *            7
 *            8
 *            9
 *            10
 *            11        Sense Pot
 *            12
 *            13        CV Input
 *            14        GND
 *            
 *  Based on Ken Stone's Slope Detector Module CSG13...          
 *            
 */

/************************* Defines ********************************/

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define ADC_MAX 3

/************************** Variables *****************************/
byte service_adc;
unsigned int old_adc[ADC_MAX];

int sense = 100;
int sense_count;

boolean new_adc = false;

/*Hold the adc values */
unsigned long adc_array[ADC_MAX];
/*The ADC channel to read */
uint8_t adc_count = 0;

/*About the filter  */
/* .125 * 256 = 32 */
unsigned long alpha = 32;
/*Store the filtered sample */
unsigned long adc_filtered[ADC_MAX];
/*Store the previous filtered sample */
unsigned long f_v[ADC_MAX][3];

/**************************  Functions ****************************/
/* An integer math low pass filter for smooting the ADC reads */
void filter(byte ary_num) {
  /*Calculate the new value */
  //f_v[ary_num][1] = (float)alpha * adc_array[ary_num] + (1 - alpha) * f_v[ary_num][0];
  f_v[ary_num][1] = (((adc_array[ary_num] << 8) * alpha) + ((256 - alpha) * ( f_v[ary_num][0] << 8))) >> 16;
  /*Store the old value */
  f_v[ary_num][0] = f_v[ary_num][1];

  /*Store the filtered value */
  adc_filtered[ary_num] = f_v[ary_num][1];
}

void chage_adc(byte this_adc){
  ADMUX = 0x00 + this_adc;
  /*Start the conversion */
  sbi(ADCSRA, ADSC); 
}

/******************************************************************/
/*************************** Setup ********************************/
/******************************************************************/
void setup() {
 
  /************************* Setup Pins ***************************/
  /* Chip Pin 2 as Output for Rising */
  DDRB |= _BV (0); 
  /* Chip Pin 3 as Output for Steady */
  DDRB |= _BV (1); 
  /* Chip Pin 5 as Output for Falling */
  DDRB |= _BV (2); 

 /*************************  Setup ADC ***************************/
  /*Set to Right Adjust for 1024 precision */
  cbi(ADMUX, ADLAR);

  /*Set to VRef to AVCC */
  cbi(ADMUX, REFS1);
  cbi(ADMUX, REFS0);

  /*Set to ADC0 to start */
  cbi(ADMUX, MUX5);
  cbi(ADMUX, MUX4);
  cbi(ADMUX, MUX3);
  cbi(ADMUX, MUX2);
  cbi(ADMUX, MUX1);
  cbi(ADMUX, MUX0);

  /*Set prescaler to 32 */
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  /* Diable digital buffer */
  sbi(DIDR0, ADC0D);
  sbi(DIDR0, ADC1D);
  sbi(DIDR0, ADC2D);  

  /*Turn off Auto Trigger */
  cbi(ADCSRA, ADATE);

  /*Turn the ADC ON  */
  sbi(ADCSRA, ADEN);

  /*Start the first conversion */
  sbi(ADCSRA, ADSC);  


}/**************************  End Setup **************************/


/******************************************************************/
/**************************** Loop ********************************/
/******************************************************************/
void loop() {
  /*Check to see if ADC has finished */
  if (!(bitRead(ADCSRA, ADSC))) {
    /*Read and store the results  */
    uint8_t temp_adcl = ADCL;
    uint16_t temp_adc = (ADCH << 8) + temp_adcl;
    /*Keep a running average */
    adc_array[adc_count] = (adc_array[adc_count] + temp_adc) / 2;
    /*Filter the results using an integer math low pass filter */
    filter(adc_count);
    /*Send the results to the Plotter */
    //Serial.println(adc_filtered[adc_count]);
    service_adc = adc_count;

    /* Change ADC */
    /* Increment the adc count */
    adc_count += 2;
    if(adc_count > ADC_MAX){
      adc_count = 0;
    }
    /* Change the ADC */
    chage_adc(adc_count);
    new_adc = true;

    /*Start the next conversion */
    //sbi(ADCSRA, ADSC);
  }
  if(new_adc){
      switch(service_adc){
        case 0:
    /* Rising */
        if(adc_filtered[0] > old_adc[0]){
          PORTB |= _BV (0); // digitalWrite (8, HIGH);
          PORTB &= ~_BV (1); // digitalWrite (9, LOW);
          PORTB &= ~_BV (2); // digitalWrite (10, LOW); 
          sense_count = 0; 
        }
   /* Falling */
        if(adc_filtered[0] < old_adc[0]){
          PORTB &= ~_BV (0); // digitalWrite (8, LOW);
          PORTB &= ~_BV (1); // digitalWrite (9, LOW);
          PORTB |= _BV (2); // digitalWrite (10, HIGH);
          sense_count = 0;
        }  
  /* Steady */
        if(adc_filtered[0] == old_adc[0]){
          sense_count ++;
          if(sense_count > sense){
            PORTB &= ~_BV (0); // digitalWrite (8, LOW);
            PORTB |= _BV (1); // digitalWrite (9, HIGH);
            PORTB &= ~_BV (2); // digitalWrite (10, LOW);
            sense_count = sense;
          }
        } 
          old_adc[0] = adc_filtered[0];
          break;
        case 2:
          sense = 25 + adc_filtered[2];
          break;  
      }
    
    new_adc = false;
  }

}/*************************** End Loop *****************************/
