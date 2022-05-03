#include "GPS.h"
#include "stm32f4xx_hal.h"
#include <math.h>

//GPS variables
extern uint8_t read_serial_byte, incomming_message[100], number_used_sats, fix_type;
extern uint8_t waypoint_set, latitude_north, longiude_east ;
extern uint16_t message_counter;
extern int16_t gps_add_counter;
extern int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
extern int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;
extern float gps_pitch_adjust_north, gps_pitch_adjust, gps_roll_adjust_north, gps_roll_adjust;
extern float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;
extern uint8_t new_line_found, new_gps_data_available, new_gps_data_counter;
extern uint8_t gps_rotating_mem_location, return_to_home_step;
extern int32_t gps_lat_total_avarage, gps_lon_total_avarage;
extern int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
extern int32_t gps_lat_error, gps_lon_error;
extern int32_t gps_lat_error_previous, gps_lon_error_previous;
extern uint32_t gps_watchdog_timer;

extern float l_lon_gps_float_adjust, l_lat_gps_float_adjust, gps_man_adjust_heading;
extern float return_to_home_lat_factor, return_to_home_lon_factor, return_to_home_move_factor;
extern uint8_t home_point_recorded;
extern int32_t lat_gps_home, lon_gps_home;


void read_gps(void){
	HAL_UART_Receive(&huart1,(uint8_t *)&read_serial_byte,1,10);
		
		while (HAL_UART_Receive(&huart1,(uint8_t *)&read_serial_byte,1,10) == HAL_OK && new_line_found == 0) {                                                   //Stay in this loop as long as there is serial information from the GPS available.
    if (read_serial_byte == '$') {                                                                       //If the new byte equals a $ character.
      for (message_counter = 0; message_counter <= 99; message_counter ++) {                             //Clear the old data from the incomming buffer array.
        incomming_message[message_counter] = '-';                                                        //Write a - at every position.
      }
      message_counter = 0;                                                                               //Reset the message_counter variable because we want to start writing at the begin of the array.
    }
    else if (message_counter <= 99)message_counter ++;                                                   //If the received byte does not equal a $ character, increase the message_counter variable.
    incomming_message[message_counter] = read_serial_byte;                                               //Write the new received byte to the new position in the incomming_message array.
    if (read_serial_byte == '*') new_line_found = 1;                                                     //Every NMEA line end with a *. If this character is detected the new_line_found variable is set to 1.
  }

  //If the software has detected a new NMEA line it will check if it's a valid line that can be used.
  if (new_line_found == 1) {                                                                             //If a new NMEA line is found.
    new_line_found = 0;                                                                                  //Reset the new_line_found variable for the next line.
    if (incomming_message[4] == 'L' && incomming_message[5] == 'L' && incomming_message[7] == ',') {     //When there is no GPS fix or latitude/longitude information available.
      //digitalWrite(STM32_board_LED, !digitalRead(STM32_board_LED));                                      //Change the LED on the STM32 to indicate GPS reception.
      //Set some variables to 0 if no valid information is found by the GPS module. This is needed for GPS lost when flying.
      l_lat_gps = 0;
      l_lon_gps = 0;
      lat_gps_previous = 0;
      lon_gps_previous = 0;
      number_used_sats = 0;
    }
    //If the line starts with GA and if there is a GPS fix we can scan the line for the latitude, longitude and number of satellites.
    if (incomming_message[4] == 'G' && incomming_message[5] == 'A' && (incomming_message[44] == '1' || incomming_message[44] == '2')) {
      lat_gps_actual = ((int)incomming_message[19] - 48) *  (long)10000000;                              //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[20] - 48) * (long)1000000;                               //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[22] - 48) * (long)100000;                                //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[23] - 48) * (long)10000;                                 //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[24] - 48) * (long)1000;                                  //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[25] - 48) * (long)100;                                   //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[26] - 48) * (long)10;                                    //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual /= (long)6;                                                                         //To convert the minutes to degrees we need to divide the minutes by 6.
      lat_gps_actual += ((int)incomming_message[17] - 48) *  (long)100000000;                            //Add the degrees multiplied by 10.
      lat_gps_actual += ((int)incomming_message[18] - 48) *  (long)10000000;                             //Add the degrees multiplied by 10.
      lat_gps_actual /= 10;                                                                              //Divide everything by 10.

      lon_gps_actual = ((int)incomming_message[33] - 48) *  (long)10000000;                              //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[34] - 48) * (long)1000000;                               //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[36] - 48) * (long)100000;                                //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[37] - 48) * (long)10000;                                 //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[38] - 48) * (long)1000;                                  //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[39] - 48) * (long)100;                                   //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[40] - 48) * (long)10;                                    //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual /= (long)6;                                                                         //To convert the minutes to degrees we need to divide the minutes by 6.
      lon_gps_actual += ((int)incomming_message[30] - 48) * (long)1000000000;                            //Add the degrees multiplied by 10.
      lon_gps_actual += ((int)incomming_message[31] - 48) * (long)100000000;                             //Add the degrees multiplied by 10.
      lon_gps_actual += ((int)incomming_message[32] - 48) * (long)10000000;                              //Add the degrees multiplied by 10.
      lon_gps_actual /= 10;                                                                              //Divide everything by 10.

      if (incomming_message[28] == 'N')latitude_north = 1;                                               //When flying north of the equator the latitude_north variable will be set to 1.
      else latitude_north = 0;                                                                           //When flying south of the equator the latitude_north variable will be set to 0.

      if (incomming_message[42] == 'E')longiude_east = 1;                                                //When flying east of the prime meridian the longiude_east variable will be set to 1.
      else longiude_east = 0;                                                                            //When flying west of the prime meridian the longiude_east variable will be set to 0.

      number_used_sats = ((int)incomming_message[46] - 48) * (long)10;                                   //Filter the number of satillites from the GGA line.
      number_used_sats += (int)incomming_message[47] - 48;                                               //Filter the number of satillites from the GGA line.

      if (lat_gps_previous == 0 && lon_gps_previous == 0) {                                              //If this is the first time the GPS code is used.
        lat_gps_previous = lat_gps_actual;                                                               //Set the lat_gps_previous variable to the lat_gps_actual variable.
        lon_gps_previous = lon_gps_actual;                                                               //Set the lon_gps_previous variable to the lon_gps_actual variable.
      }

      lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 10.0;                              //Divide the difference between the new and previous latitude by ten.
      lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 10.0;                              //Divide the difference between the new and previous longitude by ten.

      l_lat_gps = lat_gps_previous;                                                                      //Set the l_lat_gps variable to the previous latitude value.
      l_lon_gps = lon_gps_previous;                                                                      //Set the l_lon_gps variable to the previous longitude value.

      lat_gps_previous = lat_gps_actual;                                                                 //Remember the new latitude value in the lat_gps_previous variable for the next loop.
      lon_gps_previous = lon_gps_actual;                                                                 //Remember the new longitude value in the lat_gps_previous variable for the next loop.
		}  
}
}	