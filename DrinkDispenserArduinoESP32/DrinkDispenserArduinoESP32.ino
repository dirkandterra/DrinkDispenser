#include <HX711_ADC.h>
#include <SPI.h>
#include <EEPROM.h>
#include <SFE_MicroOLED.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "CustomInfo.h"

#define VERSION "1.3"

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell(21, 22);
//HardwareSerial Zerial2(2); // Enable 3rd serial port on ESP32

long t;

#define INITVAL 0xEC

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
#define brewBtn 35
#define brewLed 25
#define heatLed 26
#define mainLed 27

#define NUM_RECIPIES 5
#define NUM_PRODS 2
#define NUM_CAL_AVG 5

const int freq = 5000;
const int resolution = 8;
int dutycycle = 255;
char IPAddr[17]="0.0.0.0";
IPAddress IP;
String SSIDName="";

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

typedef struct{
  uint16_t calProd[NUM_PRODS];
  uint16_t prodLeft[NUM_PRODS];
  uint8_t initialized;
  char ssid[50];
  char pwd[30];
  uint16_t customRecipe[NUM_PRODS];
}EEVars;

RecipeStruct Recipe[NUM_RECIPIES]={{2.0,6.0},{4.0,4.0},{2.5,5.5},{3.0,0.0},{0.0,4.0}};
ProdStruct Prod[NUM_PRODS]={{0.3,{0.3,0.3,0.3,0.3,0.3},0,"WHSKY",17},{.7,{.7,.7,.7,.7,.7},0,"WATER",16}};
EEVars EE;

uint8_t *EE_START=(uint8_t *)&EE.calProd[0];

float startingWeight=0;
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

typedef enum{
  SCRN_NORMAL=0,
  SCRN_ERROR,
  SCRN_IP,
}screenMode;

screenMode scrnMode = SCRN_NORMAL;
void tareScale(void);
void brewBtnPushed(void);
void sizeBtnPushed(void);
void updateScreen(void);
void EEPROM_Read(uint8_t *data, uint8_t bytes);
void EEPROM_Write(uint8_t *data, uint8_t bytes);
void backupWifiVars();
void batchingLoop();
void refreshStats();

MicroOLED oled(PIN_RESET, PIN_DC, PIN_CS); //Example SPI declara

//*WIFI STUFF*
// Replace with desired credentials in CustomInfo.h
const char* ssidAP     = SSID_AP;
const char* passwordAP = PWD_AP;
AsyncWebServer server(80);
String header;
String brewState = "Off";
String stats="";
String stats_html="";
int ii=0;
/**/

// HTML web page to handle 3 input fields (input1, input2, input3)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
              <link rel=\"icon\" href=\"data:,\">
              <style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
              .submital { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;
              text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
              .button2 { background-color: #555555; border: none; color: white; padding: 16px 40px;
              text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}</style></head>
          <body><h1>DR.Ink Controls</h1>
          <p><a href="/brew"><button class="submital">Brew</button></a></p>
          <p><a href="/size"><button class="button2">Size</button></a></p>
          <br>
          <a href="/stats">Stats<br>
          <a href="/recipe">Recipe<br>
          <a href="/wifi">Wifi<br>
          
</body></html>)rawliteral";
const char statsbefore_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
              <link rel=\"icon\" href=\"data:,\">
              <style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
              .submital { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;
              text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
              .button2 { background-color: #555555; border: none; color: white; padding: 16px 40px;
              text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}</style></head>
          <body><h1>DR.Ink Stats</h1>)rawliteral";
const char statsafter_html[] PROGMEM = R"rawliteral(
          <br>
          <h1>Set Stats</h1>
          <form action="/get">
            Whiskey Remaining: <input type="float" name="prod1Left">oz<br>
            Water Remaining: <input type="float" name="prod2Left">oz<br><br>
            <input type="submit" class = "submital" value="Submit"><br><br>
          <a href="/">Return to Home Page</a>
</body></html>)rawliteral";
const char recipe_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
                <link rel=\"icon\" href=\"data:,\">
                <style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
                .submital { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;
                text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
                .button2 {background-color: #555555;}</style></head>
                <body><h1>DR.Ink Recipe</h1>
  <form action="/get">
    Whiskey: <input type="float" name="customProd1">oz<br>
    Water: <input type="float" name="customProd2">oz<br><br>
    <input type="submit" class = "submital" value="Submit">
  </form><br>
</body></html>)rawliteral";

const char wifi_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
                <link rel=\"icon\" href=\"data:,\">
                <style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
                .submital { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;
                text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
                .button2 {background-color: #555555;}</style></head>
                <body><h1>ESP32 Web Server</h1>
  <form action="/get">
    SSID: <input type="text" name="ssid"><br>
    Pwd: <input type="text" name="pwd"><br><br>
    <input type="submit" class = "submital" value="Submit">
  </form><br>
</body></html>)rawliteral";
 
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void setup() {
    int jj=0;
    int storedWifiRetry = 20; //Times to try stored wifi before jumping to AP
    Serial.begin(115200);
    //Zerial2.begin(9600);
    EEPROM.begin(sizeof(EE));
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

    //initialize eevars
    EEPROM_Read(&EE.initialized,1);
          Serial.print("Initialized: ");
          Serial.println(EE.initialized);
    if(EE.initialized==INITVAL){
      for(ii=0;ii<NUM_PRODS;ii++){
          EEPROM_Read((uint8_t *)&EE.calProd[ii],2);
          for(jj=0;jj<NUM_CAL_AVG;jj++){
              Prod[ii].calArray[jj]=(float)EE.calProd[ii]/100;
          }
          Prod[ii].cal=(float)EE.calProd[ii]/100;
          Serial.print("Saved Prod Cal: ");
          Serial.println(Prod[ii].cal);
          EEPROM_Read((uint8_t *)&EE.prodLeft[ii],2);

          EEPROM_Read((uint8_t *)&EE.customRecipe[ii],2);
          Recipe[0].prod_Amt[ii]=(float)EE.customRecipe[ii]/10;
      }
      
      for(ii=0;ii<49;ii++){
          EEPROM_Read((uint8_t*)&EE.ssid[ii],1);
          if(EE.ssid[ii]==0){ii=50;}
      }
      
      for(ii=0;ii<29;ii++){
          EEPROM_Read((uint8_t*)&EE.pwd[ii],1);
          if(EE.pwd[ii]==0){ii=30;}
      }  
      
    }
    else{
      EE.initialized=INITVAL;
      for(ii=0;ii<NUM_PRODS;ii++){
        EE.calProd[ii]=(uint8_t)(Prod[ii].cal*100);
        EEPROM_Write((uint8_t *)&EE.calProd[ii],2);
      }      
      EEPROM_Write(&EE.initialized,1);
      // Replace with desired credentials in CustomInfo.h
      strcpy(EE.ssid,SSID_EXTERNAL);
      strcpy(EE.pwd,PWD_EXTERNAL);
      backupWifiVars();
    }
    for(ii=0;ii<NUM_PRODS;ii++){
      Serial.print("Prod Cal: ");
      Serial.println(Prod[ii].cal);
    }
   //*WEBSTUFF
  Serial.print("Connecting to ");
  Serial.println(EE.ssid);
  WiFi.begin(EE.ssid, EE.pwd);
  while ((WiFi.status() != WL_CONNECTED) && (storedWifiRetry>0)) {
    delay(500);
    Serial.print(".");
    storedWifiRetry--;
  }
  if(storedWifiRetry){
      Serial.println("");
      Serial.println("WiFi connected.");
      Serial.println("IP address: ");
      SSIDName=EE.ssid;
      IP = WiFi.localIP();
      Serial.println(IP);
  }
  else{
    Serial.println("WiFi Failed on " + String(EE.ssid) + "!");
    WiFi.softAP(ssidAP, passwordAP);
    IP = WiFi.softAPIP();
    SSIDName=ssidAP;
    Serial.print("AP IP address: ");
    Serial.println(IP);
  } 
  scrnMode=SCRN_IP;
  batchDelay=40;    //Display IP for 8 seconds
// Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", wifi_html);
  });
  server.on("/recipe", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", recipe_html);
  });
  server.on("/brew", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
    brewBtnPushed();
  });
  server.on("/size", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
    sizeBtnPushed();
  });
  server.on("/stats", HTTP_GET, [](AsyncWebServerRequest *request){
    refreshStats();
    request->send(200, "text/html", stats_html);
  });

  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage, inputMessage2;
    float inputNumber;
    String inputParam;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam("ssid")) {
      inputMessage = request->getParam("ssid")->value();
      inputMessage2 = "Saved\r\nWIFI SSID: " + inputMessage;
      inputMessage.toCharArray(EE.ssid,inputMessage.length()+1);
      EEPROM_Write((uint8_t *)&EE.ssid[0], strlen(EE.ssid)+1);
      // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
      if (request->hasParam("pwd")) {
        inputMessage = request->getParam("pwd")->value();
        inputMessage2 += "\r\nPWD: " + inputMessage;
        inputMessage.toCharArray(EE.pwd,inputMessage.length()+1);
        EEPROM_Write((uint8_t *)&EE.pwd[0], strlen(EE.pwd)+1);
      }
    }
    if (request->hasParam("customProd1")) {
      inputMessage = request->getParam("customProd1")->value();
      inputNumber = inputMessage.toFloat();
      inputMessage2 = "Saved\r\nWhiskey:  " +String(inputNumber) ;
      EE.customRecipe[0]=(uint16_t)(inputNumber*10);
      EEPROM_Write((uint8_t *)&EE.customRecipe[0], 2);
      Recipe[0].prod_Amt[0]=(float)EE.customRecipe[0]/10;
      // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
      if (request->hasParam("customProd2")) {
        inputMessage = request->getParam("customProd2")->value();
        inputNumber = inputMessage.toFloat();
        inputMessage2 += "\r\nWater: " + String(inputNumber); 
        EE.customRecipe[1]=(uint16_t)(inputNumber*10);
        EEPROM_Write((uint8_t *)&EE.customRecipe[1], 2);
        Recipe[0].prod_Amt[1]=(float)EE.customRecipe[1]/10;
      }
    }
    if (request->hasParam("prod1Left")) {
      inputMessage = request->getParam("prod1Left")->value();
      inputNumber = inputMessage.toFloat();
      inputMessage2 = "Saved\r\nWhiskey:  " +String(inputNumber) ;
      EE.prodLeft[0]=(uint16_t)(inputNumber*10);
      EEPROM_Write((uint8_t *)&EE.prodLeft[0], 2);
      // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
      if (request->hasParam("prod2Left")) {
        inputMessage = request->getParam("prod2Left")->value();
        inputNumber = inputMessage.toFloat();
        inputMessage2 += "\r\nWater: " + String(inputNumber); 
        EE.prodLeft[1]=(uint16_t)(inputNumber*10);
        EEPROM_Write((uint8_t *)&EE.prodLeft[1], 2);
      }
    }
    
    
    Serial.println(inputMessage);    
    request->send(200, "text/html", inputMessage2 + "<br><a href=\"/\">Return to Home Page</a>");
  });
  server.onNotFound(notFound);
  server.begin();
  /**/
}

void loop() {
	//update() should be called at least as often as HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS
	//longer delay in sketch will reduce effective sample rate (be carefull with delay() in loop)

	//get smoothed value from data set + current calibration factor

   batchingLoop();


}

void batchingLoop(){
  uint8_t temp=0;
  uint16_t temp16=0;
  LoadCell.update();
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
      if(scrnMode==SCRN_IP){
         scrnMode=SCRN_NORMAL; 
      }
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
          scrnMode=SCRN_NORMAL;
					batchState++;
					break;
				case 2:
					//Do we have products to service?
					if(currentProd<NUM_PRODS){
						//Does the recipe call for this product?
						if(Recipe[currentRecipe].prod_Amt[currentProd]>0){
							targetWeight=currentWeight +Recipe[currentRecipe].prod_Amt[currentProd];
              startingWeight=currentWeight;
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
            if(currentProd){
              digitalWrite(brewLed, 0);
            }     
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
            scrnMode=SCRN_ERROR;
					}
					//We are at target minus cal, shut down motor
					else{
						Serial.print("MTR ");
						Serial.print(currentProd);
						Serial.println(" OFF");
						digitalWrite(heatLed, 1);
            digitalWrite(heatLed, 1);
						ledcWrite(currentProd, 0);
						batchState++;
						batchDelay=20;          //let weight settle
					}
					break;
				case 4: //update Cal
          temp16=(uint16_t)((currentWeight-startingWeight)*10);
          if(temp16>EE.prodLeft[currentProd]){
            EE.prodLeft[currentProd]=0;
          }else{
            EE.prodLeft[currentProd]-temp16;
          }
          EE.prodLeft[currentProd]-=(uint16_t)((currentWeight-startingWeight)*10);
          EEPROM_Write((uint8_t *)&EE.prodLeft[currentProd],2);
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
          //store off NV
          for(ii=0;ii<NUM_PRODS;ii++){
            temp16=(uint16_t)(Prod[ii].cal*100);
            EE.calProd[ii]=temp16; 
            EEPROM_Write((uint8_t *)&EE.calProd[ii],2);
          }
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
      scrnMode=SCRN_NORMAL;
		}
		else{ 
			if(currentRecipe<(NUM_RECIPIES-1)){
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
      switch(scrnMode){
        default:
        case SCRN_NORMAL:
          for(jj=0;jj<NUM_PRODS;jj++){
            oled.setFontType(0);         // Smallest font
            oled.setCursor(0, jj*8);     // Set cursor
            oled.print(Prod[jj].name);
            oled.print(":");       
            oled.print(Recipe[currentRecipe].prod_Amt[jj]);   // Print setpoint
          }
          break;
     
  			case SCRN_ERROR:
  				if(batchError<=NUM_PRODS){
            oled.setCursor(0, 0);        // Set cursor to top-left
            oled.setFontType(0);         // Smallest font
            oled.print(Prod[batchError-1].name); 
            oled.setCursor(0, 8); 
            oled.print("Timeout"); 
  				}
          else{
           batchError=0;
           scrnMode=SCRN_NORMAL;
          }
          break;
        case SCRN_IP:
            oled.setCursor(0, 0);        // Set cursor to top-left
            oled.setFontType(0);         // Smallest font
            oled.print("IP:"); 
            oled.setCursor(0, 8);
            oled.print(SSIDName); 
            oled.setCursor(0, 24); 
            oled.print(String(IP[0])+"."+String(IP[1])+".");
            oled.setCursor(0, 32); 
            oled.print(String(IP[2])+"."+String(IP[3]));
          break;
      }
			break;			
	}
	oled.display();							//Make it so
}

void EEPROM_Read(uint8_t *data, uint8_t bytes){
    int ii;
    int addr=data-EE_START;
    for(ii=0; ii<bytes; ii++){
      data[ii]=EEPROM.read(ii+addr);
    } 
}
void EEPROM_Write(uint8_t *data, uint8_t bytes){
    int ii;
    int addr=data-EE_START;
    for(ii=0; ii<bytes; ii++){
       EEPROM.write((ii+addr),data[ii]);
       EEPROM.commit();
    }
    
}
void backupWifiVars(){
    int zz=0;
    uint8_t sizeOfString=0;
    sizeOfString=strlen(EE.ssid)+1;
    Serial.print("Size of SSID: ");
    Serial.println(sizeOfString);
    EEPROM_Write((uint8_t *)&EE.ssid,sizeOfString);    
    sizeOfString=strlen(EE.pwd)+1;
    EEPROM_Write((uint8_t *)&EE.pwd,sizeOfString);
}
void refreshStats(){
  
  stats="<p>Whiskey Left: " + String((float)EE.prodLeft[0]/10) +
        "</p><p>Water Left: " + String((float)EE.prodLeft[1]/10) + "</p><br>";
  stats_html=statsbefore_html + stats + statsafter_html;
}
