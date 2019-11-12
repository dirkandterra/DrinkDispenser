#include <HX711_ADC.h>
#include <SPI.h>
#include <SFE_MicroOLED.h>

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell(21, 22);
//HardwareSerial Zerial2(2); // Enable 3rd serial port on ESP32

long t;

#define PIN_RESET 19  // Connect RST to pin 9 (req. for SPI and I2C)
#define PIN_DC    15  // Connect DC to pin 8 (required for SPI)
#define PIN_CS    5 // Connect CS to pin 10 (required for SPI)
#define DC_JUMPER 0 // Set to either 0 (default) or 1 based on jumper, matching the value of the DC Jumper

#define mtr1_Pin 17
#define mtr2_Pin 16
#define mtr3_Pin 12
#define mtr4_Pin 13
#define mtrEnable_Pin 14

#define onboardBtn 0
#define sizeBtn 34
#define brewBtn 33
#define brewLed 25
#define heatLed 26
#define mainLed 27

#define NUM_RECIPIES 3
#define NUM_PRODS 2
#define NUM_CAL_AVG 5

const int freq = 5000;
const int resolution = 8;
int dutycycle = 255;

typedef struct{
	float cal;
	float calArray[NUM_CAL_AVG];
	int calArrayPtr;
	char name[6];
  int mtr_Pin;
}ProdStruct;

typedef struct{
	float prod_Amt[NUM_PRODS];
}RecipeStruct;

RecipeStruct Recipe[NUM_RECIPIES]={{4.0,4.0},{2.5,5.5},{3.0,0.0}};
ProdStruct Prod[NUM_PRODS]={{0.3,{0.3,0.3,0.3,0.3,0.3},0,"WHSKY",17},{0.3,{0.3,0.3,0.3,0.3,0.3},0,"WATER",16}};

float startingWeight=0.0;
float targetWeight=0;
float currentWeight=0.0;
int currentProd=0;
int fillTimeout=0;      //Seconds

int batchState=0;
int buttonPushed=0;
int brewBtnOld=0;
int sizeBtnOld=0;
int currentRecipe=0;
int batchDelay=0;
int batchError=0;

void tareScale(void);
void brewBtnPushed(void);
void sizeBtnPushed(void);
void updateScreen(void);
MicroOLED oled(PIN_RESET, PIN_DC, PIN_CS); //Example SPI declara

void setup() {
    int ii=0;
    Serial.begin(115200);
    //Zerial2.begin(9600);
    Serial.println("Wait...");
    LoadCell.begin();
    long stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
    LoadCell.start(stabilisingtime);
    LoadCell.setCalFactor(12737.0); // user set calibration factor (float)
    Serial.println("Startup + tare is complete");
    delay(100);
    oled.begin();
    //oled.invert(true);  
    oled.clear(ALL);
    pinMode(mtrEnable_Pin, OUTPUT);
    pinMode(mainLed, OUTPUT);
    pinMode(heatLed, OUTPUT);
    pinMode(brewLed, OUTPUT);
    pinMode(sizeBtn, INPUT);
    pinMode(brewBtn, INPUT);
    pinMode(onboardBtn, INPUT);
    for(ii=0; ii<NUM_PRODS; ii++){
        pinMode(Prod[ii].mtr_Pin, OUTPUT);
        ledcSetup(ii, freq, resolution);
        ledcAttachPin(Prod[ii].mtr_Pin, ii);
    }
}

void loop() {
	//update() should be called at least as often as HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS
	//longer delay in sketch will reduce effective sample rate (be carefull with delay() in loop)
	int ii=0;
	LoadCell.update();
	//get smoothed value from data set + current calibration factor
	if (millis() > t + 250) {
		if(!digitalRead(onboardBtn)){
		  buttonPushed=1;
		}
		else{
		  buttonPushed=0;
		}
		if(!digitalRead(brewBtn)&&brewBtnOld){
		  Serial.println("BrewBtn");
		  brewBtnOld=0;
		  brewBtnPushed();
		}
		else{
		  brewBtnOld=1;
		}
		if(!digitalRead(sizeBtn)&&sizeBtnOld){
		  Serial.println("SizeBtn");
		  sizeBtnOld=0;
		  sizeBtnPushed();
		}
		else{
		  sizeBtnOld=1;
		}

		currentWeight = (LoadCell.getData());
		float v = LoadCell.getCalFactor();
		//Serial.print("Load_cell output val: ");
		//Serial.print(currentWeight);
		//Serial.print("      Load_cell calFactor: ");
		//Serial.println(v);
		  updateScreen();

		// Create a string which is the integer value of the weight times 10,
		//  to remove the decimal point.
		String weight = String(int(currentWeight*10));
		//Zerial2.write(0x76); // Clear the display
		//Zerial2.print(weight); // Write out the weight value to the display

		// Identify which decimal point to set, and set it.
		int shiftBy = 5-weight.length();
		int decimalPoint = 0x08>>(shiftBy);
		//Zerial2.write(0x77);
		//Zerial2.write(decimalPoint & 0x0F);

		if(buttonPushed){
		  if(batchState>0){
		  //batchState=0;   TODO: Make a cancel
		  }else{
			batchState=1;
		  }
		}
		if(batchDelay<1){
			switch (batchState){
				case 1:
					digitalWrite(mtrEnable_Pin,1);
					Serial.println("Batch Beginning.....");
					digitalWrite(mainLed,0);
					tareScale();
					delay(2000);
					startingWeight=0;
					currentProd=0;
					batchError=0;
					batchState++;
					break;
				case 2:
					//Do we have products to service?
					if(currentProd<NUM_PRODS){
						//Does the recipe call for this product?
						if(Recipe[currentRecipe].prod_Amt[currentProd]>0){
							targetWeight=startingWeight+Recipe[currentRecipe].prod_Amt[currentProd];
							fillTimeout=45*4;
							ledcWrite(currentProd, dutycycle);		//Start the Motor
              batchState++;
						}
						//No, skip to next
						else{
							Serial.print("PROD");
							Serial.print(currentProd);
							Serial.println(" Not Needed");
							currentProd++;
						}
					}
					//Done with products
					else{
						batchState=5;
						batchDelay=20;
					}
					break;				
				case 3:
					//Are we to target minus cal value?
					if((currentWeight<(targetWeight-Prod[currentProd].cal)) && fillTimeout>0){          
						digitalWrite(heatLed, 0);
						Serial.print("MTR ");
						Serial.print(currentProd);
						Serial.print(" ON - ");
						Serial.print(currentWeight);
						Serial.print(" < ");
						Serial.println(targetWeight);
						fillTimeout--;          
					}
					//We have timed out
					else if(fillTimeout==0){
						Serial.print("PROD");
						Serial.print(currentProd);
						Serial.println(" Timeout");
						ledcWrite(currentProd, 0);
						batchState=0;
						batchError=currentProd+1;
					}
					//We are at target minus cal, shut down motor
					else{
						Serial.print("MTR ");
						Serial.print(currentProd);
						Serial.println(" OFF");
						digitalWrite(heatLed, 1);
						ledcWrite(currentProd, 0);
						batchState++;
						batchDelay=20;          //let weight settle
					}
					break;
				case 4: //update Cal
					Prod[currentProd].calArray[Prod[currentProd].calArrayPtr]=(currentWeight-targetWeight)+Prod[currentProd].cal;
					Prod[currentProd].cal=0;
					for (ii=0;ii<NUM_CAL_AVG;ii++){
						Prod[currentProd].cal+=Prod[currentProd].calArray[ii];
					}
					Prod[currentProd].cal=Prod[currentProd].cal/NUM_CAL_AVG;	//Calculate new cal value					
					Prod[currentProd].calArrayPtr++	;							//Index calArrayPtr and check if over max
					if(Prod[currentProd].calArrayPtr>=NUM_CAL_AVG){
						Prod[currentProd].calArrayPtr=0;
					}
          Serial.print("New Cal: ");
          Serial.println(Prod[currentProd].cal);
         
					currentProd++;				//Inc current product and go to the next
					batchState=2;
					break;
				case 5:
					Serial.println("Batch Complete");
					batchState=0;
					batchDelay=20;				//Linger on this message for 2 seconds
					break;
				case 0:
				default:
					digitalWrite(brewLed, 1);
					digitalWrite(heatLed, 1);
					digitalWrite(mainLed,1);
					digitalWrite(mtrEnable_Pin,0);
					for(ii=0;ii<NUM_PRODS;ii++){
						ledcWrite(ii, 0);
					}
					batchState=0;			//Just in case batchState is off somewhere weird
					break;
			}
		}
		//Dec batchDelay if greater than 0
		if (batchDelay>0){
			batchDelay--;
		}
		
		t = millis();	//set current milliseconds as t
	}

	//receive from serial terminal
	if (Serial.available() > 0) {
		float i;
		char inByte = Serial.read();
		if      (inByte == 'l') i = -1.0;
		else if (inByte == 'L') i = -10.0;
		else if (inByte == 'h') i = 1.0;
		else if (inByte == 'H') i = 10.0;
		else if (inByte == 't') tareScale;
		else if (inByte == '1') dutycycle = 0;
		else if (inByte == '2') dutycycle = 127;
		else if (inByte == '3') dutycycle = 191;
		else if (inByte == '4') dutycycle = 255;
		if (i != 't') {
			float v = LoadCell.getCalFactor() + i;
			LoadCell.setCalFactor(v);
		}
	}
}

void tareScale(){
	int d = 20;
	LoadCell.tareNoDelay();
	while(LoadCell.getTareStatus()==false && d>0){
		delay(100);   
		d--;
	}
	if(d==0){
		Serial.println("Tare failed");
	}
	else{ 
		Serial.println("Tare complete");
	}
}

void brewBtnPushed() {
	if (batchState==0){
		batchState=1;
	}
}

void sizeBtnPushed() {
	if (batchState>0){
		Serial.println("Manual Batch Stop....");
		batchState=0;
	}
	else{
		if(batchError){
			batchError=0;
		}
		else{ 
			if(currentRecipe<NUM_RECIPIES){
				currentRecipe++;
			}
			else{
				currentRecipe=0;
			}
		}
	}
}

void updateScreen(){
	int jj=0;
	oled.clear(PAGE);            // Clear the display
	//Batching
	switch(batchState){
		case 1:
		case 2:
		case 3:
		case 4:
			oled.setCursor(0, 0);        // Set cursor to top-left
			//oled.setFontType(0);         // Smallest font
			//oled.print("Oz: ");          // Print "Oz"
			oled.setFontType(2);         // 7-segment font
			oled.print(currentWeight);   // Print weight reading	
			oled.setCursor(0, 32);       
			oled.print(targetWeight);
			break;
		case 5:
			oled.setCursor(0, 0);        // Set cursor to top-left
			oled.setFontType(0);         // Smallest font
			oled.print("Drink");
			oled.setCursor(0, 16);
			oled.print("Complete");
			break;
		default:
		case 0:
			if(!batchError){
				for(jj=0;jj<NUM_PRODS;jj++){
					oled.setFontType(0);         // Smallest font
					oled.setCursor(0, jj*8);     // Set cursor
					oled.print(Prod[jj].name);
					oled.print(":");       
					oled.print(Recipe[currentRecipe].prod_Amt[jj]);   // Print setpoint
				}
			}
			else{
				if(batchError<=NUM_PRODS){
					oled.setCursor(0, 0);        // Set cursor to top-left
					oled.setFontType(0);         // Smallest font
					oled.print(Prod[batchError-1].name); 
					oled.setCursor(0, 8); 
					oled.print("Timeout"); 
				}
				else{
					batchError=0;
				}					
			}
			break;			
	}
	oled.display();							//Make it so
}
