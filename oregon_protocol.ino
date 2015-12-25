/*
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Fork of connectingStuff, Oregon Scientific v2.1 Emitter
 * http://connectingstuff.net/blog/encodage-protocoles-oregon-scientific-sur-arduino/
 * olivier.lebrun@gmail.com
 *
*/
#include <LowPower.h>

//#define DS18B20
#define DHT11

//#define THN132N
#define THGR2228N

  
#if defined  DS18B20
#include <OneWire.h>
#define DS18B20 0x28        /* Adresse 1-Wire du DS18B20 */
#define BROCHE_ONEWIRE 9    /* Broche utilisÃ©e pour le bus 1-Wire*/
#define POWER_SENSOR  8
OneWire ds(BROCHE_ONEWIRE); // CrÃ©ation de l'objet OneWire ds
#elif defined DHT11 
#include <DHT.h>
#define DHTTYPE DHT11
#define DHTPIN 8 
#define POWER_SENSOR 9
DHT dht (DHTPIN, DHTTYPE ) ;
#endif

#define POWER_TX  2 
#define TX_PIN    3

#define ANALOG_PIN       0 /*A0*/
#define ANALOG_GND_PIN   12
#define ANALOG_CAL_GAIN  (1.05)
#define ANALOG_CAL_OFF   (0)
#define BATTERIE_NOMINAL (3.7)

#define BASE_TIME 512
const unsigned long TIME = BASE_TIME;
const unsigned long TWOTIME = BASE_TIME*2;
 
#define SEND_HIGH()  digitalWrite(TX_PIN, HIGH)
#define SEND_LOW()   digitalWrite(TX_PIN, LOW)
 
// Buffer for Oregon message
#ifdef THN132N
  byte OregonMessageBuffer[8];
#else
  byte OregonMessageBuffer[9];
#endif
 
/**
 * \brief    Send logical "0" over RF
 * \details  azero bit be represented by an off-to-on transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendZero(void) 
{
  SEND_HIGH();
  delayMicroseconds(TIME);
  SEND_LOW();
  delayMicroseconds(TWOTIME);
  SEND_HIGH();
  delayMicroseconds(TIME);
}
 
/**
 * \brief    Send logical "1" over RF
 * \details  a one bit be represented by an on-to-off transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendOne(void) 
{
  SEND_LOW();
  delayMicroseconds(TIME);
  SEND_HIGH();
  delayMicroseconds(TWOTIME);
  SEND_LOW();
  delayMicroseconds(TIME);
}
 
/**
* Send a bits quarter (4 bits = MSB from 8 bits value) over RF
*
* @param data Source data to process and sent
*/
 
/**
 * \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterMSB(const byte data) 
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
}
 
/**
 * \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterLSB(const byte data) 
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
}
 
/******************************************************************/
/******************************************************************/
/******************************************************************/
 
/**
 * \brief    Send a buffer over RF
 * \param    data   Data to send
 * \param    size   size of data to send
 */
void sendData(byte *data, byte size)
{
  for(byte i = 0; i < size; ++i)
  {
    sendQuarterLSB(data[i]);
    sendQuarterMSB(data[i]);
  }
}
 
/**
 * \brief    Send preamble
 * \details  The preamble consists of 16 "1" bits
 */
inline void sendPreamble(void)
{
  byte PREAMBLE[]={0xFF,0xFF};
  sendData(PREAMBLE, 2);
}
 
/**
 * \brief    Send postamble
 * \details  The postamble consists of 8 "0" bits
 */
inline void sendPostamble(void)
{
#ifdef THN132N
  sendQuarterLSB(0x00);
#else
  byte POSTAMBLE[]={0x00};
  sendData(POSTAMBLE, 1);  
#endif
}


/**
 * \brief    Send an Oregon message
 * \param    data   The Oregon message
 */
void sendOregon(byte *data, byte size)
{
  sendPreamble();
  //sendSync();
  sendData(data, size);
  sendPostamble();
}
 
 
/**
 * \brief    Send sync nibble
 * \details  The sync is 0xA. It is not use in this version since the sync nibble
 * \         is include in the Oregon message to send.
 */
inline void sendSync(void)
{
  sendQuarterLSB(0xA);
}
 
/******************************************************************/
/******************************************************************/
/******************************************************************/
 
/**
 * \brief    Set the sensor type
 * \param    data       Oregon message
 * \param    type       Sensor type
 */
inline void setType(byte *data, byte* type) 
{
  data[0] = type[0];
  data[1] = type[1];
}
 
/**
 * \brief    Set the sensor channel
 * \param    data       Oregon message
 * \param    channel    Sensor channel (0x10, 0x20, 0x30)
 */
inline void setChannel(byte *data, byte channel) 
{
  data[2] = channel;
}
 
/**
 * \brief    Set the sensor ID
 * \param    data       Oregon message
 * \param    ID         Sensor unique ID
 */
inline void setId(byte *data, byte ID) 
{
  data[3] = ID;
}
 
/**
 * \brief    Set the sensor battery level
 * \param    data       Oregon message
 * \param    level      Battery level (0 = low, 1 = high)
 */
void setBatteryLevel(byte *data, byte level)
{
  if(!level) data[4] = 0x0C;
  else data[4] = 0x00;
}
 
/**
 * \brief    Set the sensor temperature
 * \param    data       Oregon message
 * \param    temp       the temperature
 */
void setTemperature(byte *data, float temp) 
{
  // Set temperature sign
  if(temp < 0)
  {
    data[6] = 0x08;
    temp *= -1;  
  }
  else
  {
    data[6] = 0x00;
  }
  // Determine decimal and float part
  int tempInt = (int)temp;
  int td = (int)(tempInt / 10);
  int tf = (int)round((float)((float)tempInt/10 - (float)td) * 10);
  int tempFloat =  (int)round((float)(temp - (float)tempInt) * 10);
 
  // Set temperature decimal part
  data[5] = (td << 4);
  data[5] |= tf;
 
  // Set temperature float part
  data[4] |= (tempFloat << 4);
}
 
/**
 * \brief    Set the sensor humidity
 * \param    data       Oregon message
 * \param    hum        the humidity
 */
void setHumidity(byte* data, byte hum)
{
    data[7] = (hum/10);
    data[6] |= (hum - data[7]*10) << 4;
}
 
/**
 * \brief    Sum data for checksum
 * \param    count      number of bit to sum
 * \param    data       Oregon message
 */
int Sum(byte count, const byte* data)
{
  int s = 0;
  for(byte i = 0; i<count;i++)
  {
    s += (data[i]&0xF0) >> 4;
    s += (data[i]&0xF);
  }
  if(int(count) != count)  s += (data[count]&0xF0) >> 4;
  return s;
}
 
/**
 * \brief    Calculate checksum
 * \param    data       Oregon message
 */
void calculateAndSetChecksum(byte* data)
{
#ifdef THN132N
  int s = ((Sum(6, data) + (data[6]&0xF) - 0xa) & 0xff);
  data[6] |=  (s&0x0F) << 4;     data[7] =  (s&0xF0) >> 4;
#else
  data[8] = ((Sum(8, data) - 0xa) & 0xFF);
#endif
}
 
/******************************************************************/
/******************************************************************/

/**
 * \brief    Get batterie voltage 
 * \param    voltage      tension of the batterie 
 */
boolean getVoltage (float *voltage)
{
  int val ; 
  digitalWrite(ANALOG_GND_PIN,LOW);
  delay (100);
  val = analogRead(ANALOG_PIN);    // read the input pin
  *voltage =  (((float)(val)) / 1024 ) * 1.1 * ( (56+10) / 10 ) ;  
  *voltage = *voltage * ANALOG_CAL_GAIN + ANALOG_CAL_OFF ;
  digitalWrite(ANALOG_GND_PIN,HIGH);   
  return 1 ; 
}

/**
 * \brief    Get batterie voltage 
 * \param    hum      return humidity value in %
 */
boolean getHumidity(float *hum)
{
  // Set Humidity 
#ifndef THN132N
#ifdef DHT11
  *hum = dht.readHumidity();
  return 1;
#else
  *hum = 0 ; 
  return 1;     
#endif   œ
  return 0 ;     
#endif  
}


/**
 * \brief    Get batterie voltage 
 * \param    temp      return temperature in C
 */
boolean getTemperature(float *temp)
{
#ifdef DS18B20
  byte data[9], addr[8];
  // data : DonnÃ©es lues depuis le scratchpad
  // addr : adresse du module 1-Wire dÃ©tectÃ©
 
  
  if (!ds.search(addr)) { // Recherche un module 1-Wire
    ds.reset_search();    // RÃ©initialise la recherche de module
    Serial.println("\n#Error Seach module 1-Wire \n");
    return false;         // Retourne une erreur
  }
   
  if (OneWire::crc8(addr, 7) != addr[7]) // VÃ©rifie que l'adresse a Ã©tÃ© correctement reÃ§ue
  {
    Serial.println("\n#Error Crc 1-Wire \n");
    return false;                        // Si le message est corrompu on retourne une erreur
  }

  if (addr[0] != DS18B20) // VÃ©rifie qu'il s'agit bien d'un DS18B20
  {
     Serial.println("\n#Error Read \n");    
    return false;         // Si ce n'est pas le cas on retourne une erreur
   }
  ds.reset();             // On reset le bus 1-Wire
  ds.select(addr);        // On sÃ©lectionne le DS18B20
   
  ds.write(0x44, 1);      // On lance une prise de mesure de tempÃ©rature
  delay(1000);             // Et on attend la fin de la mesure
   
  ds.reset();             // On reset le bus 1-Wire
  ds.select(addr);        // On sÃ©lectionne le DS18B20
  ds.write(0xBE);         // On envoie une demande de lecture du scratchpad
 
  for (byte i = 0; i < 9; i++) // On lit le scratchpad
    data[i] = ds.read();       // Et on stock les octets reÃ§us
   
  // Calcul de la tempÃ©rature en degrÃ© Celsius
  *temp = ((data[1] << 8) | data[0]) * 0.0625; 
   
  // Pas d'erreur
  return true;
#elif DHT11
  *temp  = dht.readTemperature();
  return true;
#endif   
}
 
 
/******************************************************************/
 
void setup()
{
  float dummy_volt ; 

  Serial.begin(9600);
  Serial.println("\n[Oregon V2.1 encoder]");
  
  // Init voltage pinout
  analogReference (INTERNAL);
  pinMode(ANALOG_GND_PIN, OUTPUT);
  delay (1000);
  getVoltage(&dummy_volt);
  // Init power line 
  digitalWrite(13,LOW);
  pinMode(POWER_SENSOR, OUTPUT);
  digitalWrite(POWER_SENSOR,LOW);
  
  // Init Tx Pinout 
  pinMode(POWER_TX, OUTPUT);
  pinMode(TX_PIN, OUTPUT);
  SEND_LOW();  
 
#ifdef THN132N  
  // Create the Oregon message for a temperature only sensor (TNHN132N)
  byte ID[] = {0xEA,0x4C};
#else
  // Create the Oregon message for a temperature/humidity sensor (THGR2228N)
  byte ID[] = {0x1A,0x2D};
#endif  
 
  setType(OregonMessageBuffer, ID);
  setChannel(OregonMessageBuffer, 0x20);
  setId(OregonMessageBuffer, 0xCC);
}
 
void loop()
{
  // Get Temperature, humidity and battery level from sensors
  boolean mesure_valide ; 
  float temp = 0;
  float hum  = 0 ; 
  float volt = 0 ; 

  digitalWrite(POWER_SENSOR,HIGH);
  delay (250);
  mesure_valide = true ;
  if (getTemperature(&temp)) 
  {
    // (ie: 1wire DS18B20 for tempÃ©rature, ...)
    setTemperature(OregonMessageBuffer, temp);
  }
  else
  {
    setTemperature(OregonMessageBuffer, 0);  
    mesure_valide = false ;
  }

#ifndef THN132N
  if (getHumidity (&hum))
  {
    setHumidity(OregonMessageBuffer, hum);
  }
  else
  {
    setHumidity(OregonMessageBuffer, 0);  
    mesure_valide = false ;
  }
#endif

  digitalWrite(POWER_SENSOR,LOW);
  if (getVoltage (&volt))
  {
    if ( volt < (((float)BATTERIE_NOMINAL)  * 0.9 ))
    {
      setBatteryLevel(OregonMessageBuffer, 0); // 0 : low
    }
    else
    {
      setBatteryLevel(OregonMessageBuffer, 1); //  1 : high
    }
  }


  {
  // Calculate the checksum
    calculateAndSetChecksum(OregonMessageBuffer);

    if (mesure_valide)
    {
      Serial.print("Temperature : ");Serial.print(temp); Serial.write(' C'); Serial.println();
#ifndef THN132N
      Serial.print("Humidity    : ");Serial.print(hum); Serial.write(' %'); Serial.println();
#endif     
      Serial.print("Tension     : ");Serial.print(volt); Serial.write(' V'); Serial.println();
      Serial.print("Msg Oregon  : ");
      // Show the Oregon Message
      for (byte i = 0; i < sizeof(OregonMessageBuffer); ++i)   
      {
        Serial.print(OregonMessageBuffer[i] >> 4, HEX);
        Serial.print(OregonMessageBuffer[i] & 0x0F, HEX);
      }
      Serial.println();

      digitalWrite(POWER_TX,HIGH);
      delay (100);
      // Send the Message over RF
      sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
      // Send a "pause"
      SEND_LOW();
      delayMicroseconds(TWOTIME*8);
      // Send a copie of the first message. The v2.1 protocol send the
      // message two time 
      sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
      SEND_LOW();
      digitalWrite(POWER_TX,LOW);
    }
    else
    {
      Serial.print  ("#Error Invalide mesure");
      Serial.println();
    }
  }
 
  // Wait for 30 seconds before send a new message 
  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
/*
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  

  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  

  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
*/
  
  //delay(30000);
}
