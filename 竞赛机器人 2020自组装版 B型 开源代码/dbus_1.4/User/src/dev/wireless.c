/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of聽
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.聽 See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
  *********************** (C) COPYRIGHT 2018 DJI **********************
  * @update
  * @history
  * Version     Date           Author           Modification
  * V1.0.0      June-26-2018   ric.luo
  * @verbatim
  *********************** (C) COPYRIGHT 2018 DJI **********************
  */

#include "dev.h"
#include "tutorial_lib.h"

#include "pid.h"

uint8_t wireless_id;
wireless_data_t wireless_val[4];

void wireless_recv_handler(uint8_t send_id, wireless_data_t   *data)
{
  wireless_id = send_id;

  switch (send_id)
  {
    case 1:
      memcpy(wireless_val, data, sizeof(wireless_data_t));

    break;

    case 2:
      memcpy(wireless_val+1, data, sizeof(wireless_data_t));
    break;

    case 3:
      memcpy(wireless_val+2, data, sizeof(wireless_data_t));
    break;

    case 4:
      memcpy(wireless_val+3, data, sizeof(wireless_data_t));
    break;
  }

}





