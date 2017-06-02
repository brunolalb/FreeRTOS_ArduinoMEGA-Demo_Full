/*
 * Demo Full project
 *
 * ArduinoMEGA with FreeRTOS 9.0.0
 *
 * Compiler: WinAVR
 * Burner: AVR Dude (STK500v2)
 * IDE: Eclipse Neon.3
 *
 * Description:
 * 	Implementation of the same Full Demo project available for Windows (MinGW), with modifications to run on ArduinoMEGA.
 * 	The code for Demo Blinky was kept.
 * 	The code for Demo AVR323 was kept
 *
 * 	This file contains the error messages
 *
 * Initial version (2017-04-29): Bruno Landau Albrecht (brunolalb@gmail.com)
 *
 */


/* The size of the array for error messages */
#define mainERRORSIZE						( ( unsigned char ) 32 )

const char *pcErrorOK = 			"OK                              ";
const char *pcErrorNotification = 	"Error:  Notification            ";
