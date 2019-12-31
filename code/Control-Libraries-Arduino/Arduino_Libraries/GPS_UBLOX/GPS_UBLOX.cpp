/*
	GPS_UBLOX.cpp - Ublox GPS library for Arduino
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com
	This code works with boards based on ATMega168/328 and ATMega1280 (Serial port 1)

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	GPS configuration : Ublox protocol
	Baud rate : 38400
	Active messages : 
		NAV-POSLLH Geodetic Position Solution, PAGE 66 of datasheet
		NAV-VELNED Velocity Solution in NED, PAGE 71 of datasheet
		NAV-STATUS Receiver Navigation Status
		  or 
		NAV-SOL Navigation Solution Information

	Methods:
		Init() : GPS Initialization
		Read() : Call this funcion as often as you want to ensure you read the incomming gps data
		
	Properties:
		Lattitude : Lattitude * 10000000 (long value)
		Longitude : Longitude * 10000000 (long value)
		Altitude :  Altitude * 100 (meters) (long value)
		Ground_speed : Speed (m/s) * 100 (long value)
		Ground_course : Course (degrees) * 100 (long value)
		NewData : 1 when a new data is received.
		          You need to write a 0 to NewData when you read the data
		Fix : 1: GPS FIX, 0: No Fix (normal logic)
			
*/

#include "GPS_UBLOX.h"
#include <FastSerial.h>
#include <stdlib.h>
//#include <avr/interrupt.h>
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


// Constructors ////////////////////////////////////////////////////////////////
GPS_UBLOX_Class::GPS_UBLOX_Class()
{
}


// Public Methods //////////////////////////////////////////////////////////////
void GPS_UBLOX_Class::Init(void)
{
	ck_a=0;
	ck_b=0;
	UBX_step=0;
	NewData=0;
	Fix=0;
	PrintErrors=0;
	GPS_timer=millis();   //Restarting timer...
	// Initialize serial port
	#if defined(__AVR_ATmega1280__)
		Serial1.begin(38400);         // Serial port 1 on ATMega1280
	#else
		Serial2.begin(38400);
	#endif
}

// optimization : This code don´t wait for data, only proccess the data available
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_ubx_gps() to parse and update the GPS info.
void GPS_UBLOX_Class::Read(void)
{
  static unsigned long GPS_timer=0;
  byte data;
  int numc;
  
  /*#if defined(__AVR_ATmega1280__)    // If AtMega1280 then Serial port 1...
	numc = Serial1.available();
  #else
	numc = Serial2.available();
  #endif*/
  while (Serial2.available()){
    numc = Serial2.available();
	for (int i=0;i<numc;i++)  // Process bytes received
      {
	  #if defined(__AVR_ATmega1280__)
        data = Serial1.read();
      #else
		data = Serial2.read();
	  #endif
      switch(UBX_step)     //Normally we start from zero. This is a state machine
      {
      case 0:  
        if(data==0xB5)  // UBX sync char 1
          UBX_step++;   //OH first data packet is correct, so jump to the next step
        break; 
      case 1:  
        if(data==0x62)  // UBX sync char 2
          UBX_step++;   //ooh! The second data packet is correct, jump to the step 2
        else 
          UBX_step=0;   //Nop, is not correct so restart to step zero and try again.     
        break;
      case 2:
        UBX_class=data;
        ubx_checksum(UBX_class);
        UBX_step++;
        break;
      case 3:
        UBX_id=data;
        ubx_checksum(UBX_id);
        UBX_step++;
        break;
      case 4:
        UBX_payload_length_hi=data;
        ubx_checksum(UBX_payload_length_hi);
        UBX_step++;
		// We check if the payload lenght is valid...
		if (UBX_payload_length_hi>=UBX_MAXPAYLOAD)
        {
		  if (PrintErrors)
			Serial.println("ERR:GPS_BAD_PAYLOAD_LENGTH!!");          
          UBX_step=0;   //Bad data, so restart to step zero and try again.     
          ck_a=0;
          ck_b=0;
        }
        break;
      case 5:
        UBX_payload_length_lo=data;
        ubx_checksum(UBX_payload_length_lo);
        UBX_step++;
		UBX_payload_counter=0;
        break;
      case 6:         // Payload data read...
	if (UBX_payload_counter < UBX_payload_length_hi)  // We stay in this state until we reach the payload_length
        {
          UBX_buffer[UBX_payload_counter] = data;
          ubx_checksum(data);
          UBX_payload_counter++;
          if (UBX_payload_counter==UBX_payload_length_hi)
            UBX_step++;
        }
        break;
      case 7:
        UBX_ck_a=data;   // First checksum byte
        UBX_step++;
        break;
      case 8:
        UBX_ck_b=data;   // Second checksum byte
       
	  // We end the GPS read...
        if((ck_a==UBX_ck_a)&&(ck_b==UBX_ck_b))   // Verify the received checksum with the generated checksum.. 
	  		parse_ubx_gps();               // Parse the new GPS packet
        else
		  {
		  if (PrintErrors)
			Serial.println("ERR:GPS_CHK!!");
		  }
        // Variable initialization
        UBX_step=0;
        ck_a=0;
        ck_b=0;
        GPS_timer=millis(); //Restarting timer...
        NewData=1;
		break;
	  }
    }    // End for...
  // If we don´t receive GPS packets in 2 seconds => Bad FIX state
  if ((millis() - GPS_timer)>2000)
    {
	Fix = 0;
	//Serial.println("GPS_TIMEOUT");
	if (PrintErrors) Serial.println("ERR:GPS_TIMEOUT!!");
    }
	
}
}
/****************************************************************
 * 
 ****************************************************************/
// Private Methods //////////////////////////////////////////////////////////////
void GPS_UBLOX_Class::parse_ubx_gps(void)
{
  int j;
//Verifing if we are in class 1, you can change this "IF" for a "Switch" in case you want to use other UBX classes.. 
//In this case all the message im using are in class 1, to know more about classes check PAGE 60 of DataSheet.
  if(UBX_class==0x01) 
  {
    switch(UBX_id)//Checking the UBX ID
    {
    case 0x02: //ID NAV-POSLLH 
      j=0;
      Time = join_4_bytes(&UBX_buffer[j]); // ms Time of week
      j+=4;
      Longitude = join_4_bytes(&UBX_buffer[j]); // lon*10000000
      j+=4;
      Lattitude = join_4_bytes(&UBX_buffer[j]); // lat*10000000
      j+=4;
      //Altitude = join_4_bytes(&UBX_buffer[j]);  // elipsoid heigth mm
      j+=4;
      Altitude = (float)join_4_bytes(&UBX_buffer[j]);  // MSL heigth mm
      //j+=4;
      /*
      hacc = (float)join_4_bytes(&UBX_buffer[j])/(float)1000;
      j+=4;
      vacc = (float)join_4_bytes(&UBX_buffer[j])/(float)1000;
      j+=4;
      */
      
      break;
    case 0x03://ID NAV-STATUS 
      //if(UBX_buffer[4] >= 0x03)
	  if((UBX_buffer[4] >= 0x03)&&(UBX_buffer[5]&0x01))        
        Fix=1; //valid position        
      else
        Fix=0; //invalid position
      break;

    case 0x06://ID NAV-SOL
      if((UBX_buffer[10] >= 0x03)&&(UBX_buffer[11]&0x01))
        Fix=1; //valid position
      else
        Fix=0; //invalid position        
      UBX_ecefVZ=join_4_bytes(&UBX_buffer[36]);  //Vertical Speed in cm/s
      NumSats=UBX_buffer[47];                    //Number of sats...     
      break;

    case 0x12:// ID NAV-VELNED 
      j=16;
      Speed_3d = join_4_bytes(&UBX_buffer[j]); // cm/s
      j+=4;
      Ground_Speed = join_4_bytes(&UBX_buffer[j]); // Ground speed 2D cm/s
      j+=4;
      Ground_Course = join_4_bytes(&UBX_buffer[j]); // Heading 2D deg*100000
      Ground_Course /= 1000;	// Rescale heading to deg * 100
      j+=4;
      /*
      sacc = join_4_bytes(&UBX_buffer[j]) // Speed accuracy
      j+=4;
      headacc = join_4_bytes(&UBX_buffer[j]) // Heading accuracy
      j+=4;
      */
      break; 
      }
    }   
}

float GPS_UBLOX_Class::distance_between (float lat1, float long1, float lat2, float long2) 
{
  // returns distance in meters between two positions, both specified 
  // as signed decimal-degrees latitude and longitude. Uses great-circle 
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1-long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = sq(delta); 
  delta += sq(clat2 * sdlong); 
  delta = sqrt(delta); 
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  delta = atan2(delta, denom); 
  return delta * 6378137; 
}

float GPS_UBLOX_Class::course_to (float lat1, float long1, float lat2, float long2) 
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  //if (a2 < 0.0)
  //{
    //a2 += TWO_PI;
  //}
  //return degrees(a2);
  return (a2);
}
/****************************************************************
 * 
 ****************************************************************/
 // Join 4 bytes into a long
long GPS_UBLOX_Class::join_4_bytes(unsigned char Buffer[])
{
  union long_union {
	int32_t dword;
	uint8_t  byte[4];
} longUnion;

  longUnion.byte[0] = *Buffer;
  longUnion.byte[1] = *(Buffer+1);
  longUnion.byte[2] = *(Buffer+2);
  longUnion.byte[3] = *(Buffer+3);
  return(longUnion.dword);
}

/****************************************************************
 * 
 ****************************************************************/
// Ublox checksum algorithm
void GPS_UBLOX_Class::ubx_checksum(byte ubx_data)
{
  ck_a+=ubx_data;
  ck_b+=ck_a; 
}

//GPS_UBLOX_Class GPS;