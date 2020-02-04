# DrinkDispenser
Taking a Keurig style coffee maker and turning it into a drink dispenser

-Requires HX711 arduino library
-Requires ESPAsyncWebServer and AsyncTCP libraries
-Requires SparkFun Micro OLED Library 
	!!!!!!must change "swap" keyword to something else as there is a conflict with wifi software!!!!!!
-Change #defines in CustomInfo.h file to set internal and external wifi ssid and pwd
-Tries to connect to external wifi, fires up its own wifi if it fails.  IP 192.168.4.1
