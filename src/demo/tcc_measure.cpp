/*
 * Copyright (c) 2020 Intel Corporation
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file multi-axis.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/axis.hpp>
#include <RTmotion/global.hpp>
#include <RTmotion/fb/fb_power.hpp>
#include <RTmotion/fb/fb_move_relative.hpp>
#include <RTmotion/fb/fb_move_velocity.hpp>

#include <thread>
#include <errno.h>
#include <mqueue.h>
#include <signal.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <getopt.h>

#include <ittnotify/ittnotify.h>
#include "tcc/measurement.h"
#include "tcc/measurement_helpers.h"
#include "tcc/err_code.h"
#include <inttypes.h>
#include <getopt.h>
#include <sched.h>

#define VAR_NAME_TCC_USE_SHARED_MEMORY "TCC_USE_SHARED_MEMORY"
#define VAR_NAME_TCC_MEASUREMENTS_BUFFERS "TCC_MEASUREMENTS_BUFFERS"
#define VAR_NAME_COLLECTOR_LIBRARY_NAME "INTEL_LIBITTNOTIFY64"
#define TCC_DOMAIN_NAME "TCC"
#define TCC_MEASUREMENT_NAME "Measurement"
#define TCC_MEAUSREMENT_BUFFER_SIZE 2
#define TCC_COLLECTOR_NAME "libtcc_collector.so"


#define CYCLE_US 1000
#define BUFFER_SIZE 1000
#define AXIS_NUM 84

static volatile int run = 1;
static pthread_t cyclic_thread;
static int64_t* execute_time;

static double running_time = 1.0;
static unsigned int running_time_t = (unsigned int)(BUFFER_SIZE * running_time);

static std::map<std::string, double> time_stamps;

static __itt_domain* tcc_domain = NULL;
static __itt_string_handle* measurement_handler = NULL;
static struct tcc_measurement_buffer* buffer = NULL;

#define STR(s) STR_(s)
#define STR_(s) #s
static int set_env_var(const char* name, const char* value, bool is_replace)
{
    if (setenv(name, value, is_replace) == -1) {
        printf("setenv: can't set %s variable to '%s' %s replace: %s(%i)\n",
            name,
            value,
            is_replace ? "with" : "without",
            errno);
        return -1;
    }
    return 0;
}

void* my_thread(void* arg)
{
  RTmotion::AXIS_REF axis[AXIS_NUM];
  for (size_t i = 0; i < AXIS_NUM; i++)
  {
    axis[i] = std::make_shared<RTmotion::Axis>();
    axis[i]->setAxisId(1);
    axis[i]->setAxisName("X" + i);
  }

  std::shared_ptr<RTmotion::Servo> servo[AXIS_NUM];
  for (size_t i = 0; i < AXIS_NUM; i++)
  {
    servo[i] = std::make_shared<RTmotion::Servo>();
    axis[i]->setServo(servo[i]);
  }
  printf("Axis initialized.\n");

  RTmotion::FbPower fb_power[AXIS_NUM];
  for (size_t i = 0; i < AXIS_NUM; i++)
  {
    fb_power[i].setAxis(axis[i]);
    fb_power[i].setExecute(true);
    fb_power[i].setEnablePositive(true);
    fb_power[i].setEnableNegative(true);
  }
  
  RTmotion::FbMoveRelative fb_move_rel[AXIS_NUM];
  for (size_t i = 0; i < AXIS_NUM; i++)
  {
    fb_move_rel[i].setAxis(axis[i]);
    fb_move_rel[i].setContinuousUpdate(false);
    fb_move_rel[i].setDistance(500);
    fb_move_rel[i].setVelocity(100);
    fb_move_rel[i].setAcceleration(100);
    fb_move_rel[i].setJerk(5000);
    fb_move_rel[i].setBufferMode(RTmotion::mcAborting);
  }
  printf("Function block initialized.\n");

  struct timespec next_period, start_time, end_time;
  unsigned int cycle_counter = 0;

  struct sched_param param = {};
  param.sched_priority = 99;
  pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  clock_gettime(CLOCK_MONOTONIC, &next_period);

  while (run != 0)
  {
    next_period.tv_nsec += CYCLE_US * 1000;
    while (next_period.tv_nsec >= NSEC_PER_SEC)
    {
      next_period.tv_nsec -= NSEC_PER_SEC;
      next_period.tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);

    //clock_gettime(CLOCK_MONOTONIC, &start_time);
    __itt_task_begin(tcc_domain, __itt_null, __itt_null, measurement_handler);
    for (size_t i = 0; i < AXIS_NUM; i++)
    {
      axis[i]->runCycle();
      fb_power[i].runCycle();
      fb_move_rel[i].runCycle();
    }
 
   // clock_gettime(CLOCK_MONOTONIC, &end_time);
    
    // if (cycle_counter < running_time_t)
    //   execute_time[cycle_counter] = DIFF_NS(start_time, end_time);
    // else
    //   run = 0;
    
    for (size_t i = 0; i < AXIS_NUM; i++)
    {
      if (!fb_move_rel[i].isEnabled() && fb_power[i].getPowerStatus())
        fb_move_rel[i].setExecute(true);

      // if (fb_move_rel[i].isDone() && (cycle_counter % 6000 == 0))
      if (cycle_counter % 600 == 0)
        fb_move_rel[i].setExecute(false);
    }
    __itt_task_end(tcc_domain);
    cycle_counter++;
  }
  return NULL;
}

void signal_handler(int sig)
{
  run = 0;
}

static void getOptions(int argc, char** argv)
{
  int index;
  static struct option longOptions[] = {
    // name		has_arg				flag	val
    { "time", required_argument, NULL, 't' },
    { "help", no_argument, NULL, 'h' },
    {}
  };
  do
  {
    index = getopt_long(argc, argv, "t:h", longOptions, NULL);
    switch (index)
    {
      case 't':
        running_time = atof(optarg);
        running_time_t = (unsigned int)(BUFFER_SIZE * running_time);
        printf("Time: Set running time to %d ms\n", running_time_t);
        break;
      case 'h':
        printf("Global options:\n");
        printf("    --time  -t  Set running time(s).\n");
        printf("    --help  -h  Show this help.\n");
        exit(0);
        break;5576
    }
  } while (index != -1);
}

/****************************************************************************
 * Main function
 ***************************************************************************/
int main(int argc, char* argv[])
{
  getOptions(argc, argv);
  execute_time = (int64_t*)malloc(sizeof(int64_t) * int(running_time_t));
  signal(SIGTERM, signal_handler);
  signal(SIGINT, signal_handler);
  mlockall(MCL_CURRENT | MCL_FUTURE);
  int tcc_status = 0;

    /* Initialize measurement */
  if (set_env_var(VAR_NAME_TCC_USE_SHARED_MEMORY, "true", true) == -1 ||
        set_env_var(
            VAR_NAME_TCC_MEASUREMENTS_BUFFERS, TCC_MEASUREMENT_NAME ":" STR(TCC_MEAUSREMENT_BUFFER_SIZE), true) == -1 ||
        /* Set up ittnotify environment variable to allow latency measurement */
        set_env_var(VAR_NAME_COLLECTOR_LIBRARY_NAME, TCC_COLLECTOR_NAME, false) == -1) {
        return EXIT_FAILURE;
  }
    struct tcc_measurement* tcc_measurement_ptr = NULL;
    /* Initialize the ITT domain to collect performance data for the sample workloads */
    tcc_domain = __itt_domain_create(TCC_DOMAIN_NAME);
    /* Initialize the ITT handler to collect performance data for sample workloads. */
    measurement_handler = __itt_string_handle_create(TCC_MEASUREMENT_NAME);
    if (!tcc_domain || !measurement_handler) {
        printf("Unable to create ITT handles\n");
        return EXIT_FAILURE;
    }
   /* Retrieves the measurement structure by ITT domain and task name. */
    if ((tcc_status = tcc_measurement_get(tcc_domain, measurement_handler, &tcc_measurement_ptr)) != TCC_E_SUCCESS) {
        printf("Unable to get access to measurement structure\n");
        return -TCC_E_NOT_AVAILABLE;
    }
  /* Create cyclic RT-thread */
  pthread_attr_t thattr;
  pthread_attr_init(&thattr);
  pthread_attr_setdetachstate(&thattr, PTHREAD_CREATE_JOINABLE);

  if (pthread_create(&cyclic_thread, &thattr, &my_thread, NULL))
  {
    fprintf(stderr, "pthread_create cyclic task failed\n");
    return 1;
  }
  
  while (run)
  {
    sched_yield();
  }

  pthread_join(cyclic_thread, NULL);
  tcc_measurement_print(tcc_measurement_ptr, TCC_TU_NS);
  // Open file for writing
  FILE* fptr;
  if ((fptr = fopen("data.log", "wb")) == NULL)
  {
    printf("Error! opening file: data.log");
    exit(1);  // Program exits if the file pointer returns NULL.
  }

  // Write cycle running time data
  for (size_t i = 0; i < running_time_t; ++i)
    fprintf(fptr, "%ld\n", execute_time[i]);
  fclose(fptr);

  printf("End of Program\n");
  return 0;
}

/****************************************************************************/
