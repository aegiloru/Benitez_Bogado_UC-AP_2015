/*
    Analog2Digital.h - A library for analog-to-digital conversions.
    For AVR ATMega328p (Arduino Uno) and ATMega2560 (Arduino Mega).
    This is part of the AVRTools library.
    Copyright (c) 2014 Igor Mikolic-Torreira.  All right reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


/*!
 * \file
 *
 * \brief This file provides functions that access the analog-to-digital conversion capability of the ATmega328 and ATmega2560
 * microcontrollers.
 *
 * To use these functions, include Analog2Digital.h in your source code and link against Analog2Digital.cpp.
 *
 */


#ifndef Analog2Digital_h
#define Analog2Digital_h

#include <stdint.h>

#include "GpioPinMacros.h"


#if defined(__AVR_ATmega2560__)


/*! \brief Constants representing voltage references
 *
 */

enum A2DVoltageReference
{
    kA2dReferenceAREF = 0x00,    //!< Reference is AREF pin, internal VREF turned off \hideinitializer
    kA2dReferenceAVCC = 0x01,    //!< Reference is AVCC pin, internal VREF turned off \hideinitializer
    kA2dReference11V  = 0x02,    //!< Reference is internal 1.1V VREF \hideinitializer
    kA2dReference256V = 0x03     //!< Reference is internal 2.56V VREF (only available on ATmega2560) \hideinitializer
};

#else

enum A2DVoltageReference
{
    kA2dReferenceAREF = 0x00,    // 0x00 -> AREF pin, internal VREF turned off
    kA2dReferenceAVCC = 0x01,    // 0x01 -> AVCC pin, internal VREF turned off
    kA2dReference11V  = 0x03     // 0x03 -> Internal 1.1V VREF
};

#endif


/*
    The following macro is not intended for end-user use; it is needed to support the pin naming
    macros in conjunction with the C/C++ preprocessor's re-scanning rules.
*/

#define _readGpioPinAnalog( ddr, port, pin, nbr, adc, ocr, com, tccr )      readA2D( adc )




/*!
 * \brief Read the analog value of the pin.
 *
 * This function returns a number between 0 and 1023 that corresponds to voltage between
 * 0 and a maximum reference value.  The reference value is set using one of the
 * setA2DVoltageReferenceXXX() functions.
 *
 * \arg \c pinName a pin name macro generated by GpioPinAnalog().
 *
 * \returns an value between 0 and 1023.
 *
 * \note Before calling this function must fist initialize the analog-to-digital sub-system
 * by calling initA2D().
 *
 * \hideinitializer
 */

#define readGpioPinAnalog( pinName )                                        _readGpioPinAnalog( pinName )


int readA2D( int8_t channel );



/*!
 * \brief Read the analog value of the pin.
 *
 * This function returns a number between 0 and 1023 that corresponds to voltage between
 * 0 and a maximum reference value.  The reference value is set using one of the
 * setA2DVoltageReferenceXXX() functions.
 *
 * \arg \c pinVar a pin variable that has analog-to-digital capabilities (i.e., initialized with makeGpioVarFromGpioPinAnalog()).
 *
 * \returns an value between 0 and 1023.
 *
 * \note Before calling this function must fist initialize the analog-to-digital sub-system
 * by calling initA2D().
 */

inline uint16_t readGpioPinAnalogV( const GpioPinVariable& pinVar )
{
    return readA2D( pinVar.adcNbr() );
}


/*!
 * \brief Initialize the analog-to-digital system.
 *
 * You must call this function before using any of the analog-to-digital functions.
 *
 * \arg \c ref provides the voltage reference to be used for analog-to-digital conversions.  Pass
 * one of the constants from enum A2DVoltageReference.  If no value is provided, the default is
 * kA2dReferenceAVCC.
 */

void initA2D( uint8_t ref = kA2dReferenceAVCC );



/*!
 * \brief Turn off the analog-to-digital system.
 *
 */

void turnOffA2D();


/*!
 * \brief Set the voltage reference for the analog-to-digital system.
 *
 * After your have initialized the analog-to-digital system with initA2D(), you can
 * use this function to change the voltage reference.
 *
 * \arg \c ref provides the voltage reference to be used for analog-to-digital conversions.  Pass
 * one of the constants from enum A2DVoltageReference.
 */

void setA2DVoltageReference( A2DVoltageReference ref );



/*!
 * \brief Set the voltage reference for the analog-to-digital system to AREF.
 *
 * This is an inline synonym for setA2DVoltageReference( kA2dReferenceAREF )
 */

inline void setA2DVoltageReferenceAREF()
{ setA2DVoltageReference( kA2dReferenceAREF ); }



/*!
 * \brief Set the voltage reference for the analog-to-digital system to AREF.
 *
 * This is an inline synonym for setA2DVoltageReference( kA2dReferenceAVCC )
 */

inline void setA2DVoltageReferenceAVCC()
{ setA2DVoltageReference( kA2dReferenceAVCC ); }



/*!
 * \brief Set the voltage reference for the analog-to-digital system to AREF.
 *
 * This is an inline synonym for setA2DVoltageReference( kA2dReference11V )
 */

inline void setA2DVoltageReference11V()
{ setA2DVoltageReference( kA2dReference11V ); }


#if defined(__AVR_ATmega2560__)


/*!
 * \brief Set the voltage reference for the analog-to-digital system to AREF.
 *
 * This is an inline synonym for setA2DVoltageReference( kA2dReference256V )
 *
 * \note this function is only available on ATmega2560 (not on the ATmega328).
 */

inline void setA2DVoltageReference256V()
{ setA2DVoltageReference( kA2dReference256V ); }

#endif



/*!
 * \brief Read an analog voltage value.
 *
 * Voltage is read relative to the currently set reference value.
 *
 * \arg \c channel is an ADC channel number (between 0 and 7 on ATmega328; between 0 and 15 on ATMega2560).
 *
 * \returns a number between 0 and 1023.
 *
 * \note Generally users will not call this function but instead call readPinAnalog() passing it a
 * pin name macro generated by Analog().
 *
 * \hideinitializer
 */

int readA2D( int8_t channel );

#endif
