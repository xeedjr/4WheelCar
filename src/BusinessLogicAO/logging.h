/*
 * logging.h
 *
 *  Created on: 18 лют. 2021 р.
 *      Author: Bogdan
 */

#pragma once

#define LOG_SERIAL

#ifdef LOG_SERIAL
  #define LOG_DEBUG(x) printf(x);
#else
  #define LOG_DEBUG(x)
#endif

