#include <enc28j60.h>


#define ETH_DST_MAC 0
#define ETH_SRC_MAC 6
#define ETH_LEN_H_P 12
#define ETH_LEN_L_P 13
#define ETH_DATA_P 0xe
#define DEBAG printf
#define RX 0
#define TX 1
#define STATUS TX


#define ADC_PIN_    0


// RAN =============


int diode = 0;
int triger = 800;
int LlastLight;                // variable to store last button state
int RlastLight; 
long LstartTime ;                    // start time for stop watch
long RstartTime ;
long elapsedTime ;                  // elapsed time for stop watch
int LVx = 0;
int RVx = 0;
int Llight = 0;
int Rlight = 0;

// RAN ==============


typedef union _group_16 {
  int u16;
  uint8_t u8[2];
} group_16;



typedef struct _data_frame{
  unsigned char sens_id;
  int sens_data;
}data_frame;


typedef struct _sens_node_frame {

  int BGR_hadder;
  unsigned char node_id;
  unsigned char number_of_sens;
  data_frame sens_array[4];
}sens_node_frame;

typedef union _sens_pakage
{
  sens_node_frame frame;
  unsigned char seggment[26];

}sens_pakage;


int FlipEindian(int num )
{
  group_16 ans;
  unsigned char temp1,temp2;

  temp1 = 0xff & num;
  temp2 =  num>>8;

  ans.u8[0] = temp2;
  ans.u8[1] = temp1;

  return ans.u16;
}

const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
sens_pakage sensor_frame;
int i;
enc28j60 enc;

uint8_t data[200] = {0};
uint8_t buf[200];


void setup() {
  
// RAN ======
  
   pinMode(A0, INPUT);
   pinMode(A1, INPUT);
  
// RAN ======
  
  
  // put your setup code here, to run once:
Serial.begin(9600);





// init

// set address
uint8_t macaddr[6]    = {0x27,0x28,0x29,0x30,0x31,0x32};
uint8_t gwmacaddr[6]  = {0xB8,0x27,0xEB,0x85,0xBB,0xAE};
//group_16 datalen;



enc.enc28j60Init(macaddr);


//set adress into the buffer buf
int i =0;
while(i<6)

{
  buf[ETH_DST_MAC +i]=gwmacaddr[i]; // gw mac in local lan or host mac
  buf[ETH_SRC_MAC +i]=macaddr[i];
i++;
}

//set Ether type
buf[ETH_LEN_H_P] = 0x08;
buf[ETH_LEN_L_P] = 0x00;

//init frame

for(i = 0 ;i<10 ; i++)
  data[i] = 0xBB;


while(i<10)
{
  buf[ETH_DATA_P+i]=data[i];
  i++;
}

enc.enc28j60PacketSend(40,buf);



}

void loop() {
  // put your main code here, to run repeatedly:

while(1){



sensor_frame.frame.BGR_hadder = 0xBBBB;
sensor_frame.frame.node_id = 1;
sensor_frame.frame.number_of_sens = 1;

sensor_frame.frame.sens_array[0].sens_id = 0xF0;

  
// RAN ================
  
  
  //**************************************** code for left *************************************************** 
   Llight = analogRead(A0); 
   if (Llight > triger && LlastLight < triger)          // check if get to hole

   {     

      elapsedTime =   millis() - LstartTime;
      LstartTime = millis();                                   // store the start time
     LVx =  3.6*44*PI*elapsedTime/1000 ;                            //elapsedTime/1000 its make time in sec multiply by 2pi multiply by R eff
   
   }
      LlastLight = Llight;                     // store buttonState in lastButtonState, to compare next time
      Serial.println(LVx); 
//********************************************* code for right ****************************************************
  Rlight = analogRead(A1);
   if (Rlight > triger && RlastLight < triger)          // check if get to hole

   {     

      elapsedTime =   millis() - RstartTime;
      RstartTime = millis();                                   // store the start time
     RVx =  3.6*44*PI*elapsedTime/1000 ;                            //elapsedTime/1000 its make time in sec multiply by 2pi multiply by R eff 
   }
        RlastLight = Rlight;                     // store buttonState in lastButtonState, to compare next time

        Serial.println(RVx);   

sensor_frame.frame.sens_array[0].sens_data = FlipEindian(LVx);
  
sensor_frame.frame.sens_array[1].sens_id = 0xF1;
sensor_frame.frame.sens_array[1].sens_data = FlipEindian(RVx);
// RAN ==================  

_delay_ms(20);
sensor_frame.frame.sens_array[2].sens_id = 0xF2;
sensor_frame.frame.sens_array[2].sens_data = FlipEindian(analogRead(A2));;
_delay_ms(20);
sensor_frame.frame.sens_array[3].sens_id = 0xF3;
sensor_frame.frame.sens_array[3].sens_data = FlipEindian(analogRead(A3));;
_delay_ms(20);


i =0;
while(i<26)
{
  buf[ETH_DATA_P+i]=sensor_frame.seggment[i];
  i++;
}
enc.enc28j60PacketSend(26,buf);

}//while(1)





}//loop
