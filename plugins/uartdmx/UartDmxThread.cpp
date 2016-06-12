/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * UartDmxThread.cpp
 * The DMX through a UART plugin for ola
 * Copyright (C) 2011 Rui Barreiros
 * Copyright (C) 2014 Richard Ash
 */

#include <math.h>
#include <unistd.h>
#include <string>
#include "ola/Clock.h"
#include "ola/Logging.h"
#include "ola/StringUtils.h"
#include "plugins/uartdmx/UartWidget.h"
#include "plugins/uartdmx/UartDmxThread.h"

namespace ola {
namespace plugin {
namespace uartdmx {

UartDmxThread::UartDmxThread(UartWidget *widget, unsigned int breakt,
                             unsigned int malft)
  : m_granularity(UNKNOWN),
    m_widget(widget),
    m_term(false),
    m_breakt(breakt),
    m_malft(malft) {
}

UartDmxThread::~UartDmxThread() {
  Stop();
}


/**
 * Stop this thread
 */
bool UartDmxThread::Stop() {
  {
    ola::thread::MutexLocker locker(&m_term_mutex);
    m_term = true;
  }
  return Join();
}


/**
 * Copy a DMXBuffer to the output thread
 */
bool UartDmxThread::WriteDMX(const DmxBuffer &buffer) {
  ola::thread::MutexLocker locker(&m_buffer_mutex);
  m_buffer.Set(buffer);
  return true;
}


/**
 * The method called by the thread
 */
void *UartDmxThread::Run() {
  Clock clock;
  CheckTimeGranularity();
  DmxBuffer buffer;

  /* members for performance / error tracking */
  int err_breakstart, err_breakstop, err_write, frames;
  TimeStamp lastprint, now;
  // how often to print out, in microseconds
  TimeInterval m_printinterval = TimeInterval(1e6);
  TimeInterval since_lastprint;


  // Setup the widget
  if (!m_widget->IsOpen())
    m_widget->SetupOutput();

  // zero counters
  err_breakstart = 0;
  err_breakstop = 0;
  err_write = 0;
  frames = 0;
  clock.CurrentTime(&lastprint);

  while (1) {
    {
      ola::thread::MutexLocker locker(&m_term_mutex);
      if (m_term)
        break;
    }

    {
      ola::thread::MutexLocker locker(&m_buffer_mutex);
      buffer.Set(m_buffer);
    }

    if (!m_widget->SetBreak(true)) {
      err_breakstart++;
      goto framesleep;
    }

    if (m_granularity == GOOD)
      usleep(m_breakt);

    if (!m_widget->SetBreak(false)) {
      err_breakstop++;
      goto framesleep;
    }

    if (m_granularity == GOOD)
      usleep(DMX_MAB);

    if (!m_widget->Write(buffer)) {
      err_write++;
      goto framesleep;
    }

  framesleep:
    // Sleep for the remainder of the DMX frame time
    usleep(m_malft);


  /* do houskeeping stuff (would be nice to put it somewhere non-critical */
  clock.CurrentTime(&now);
  since_lastprint = now - lastprint;
  if (since_lastprint > m_printinterval) {
    // need to do printout
    lastprint = now;

    OLA_INFO << "UART thread frames " << frames 
       << ", errors: start break " << err_breakstart 
       << ", stop break " << err_breakstop 
       << ", write " << err_write;

    /* clear all counters to go again */
    err_breakstart = 0;
    err_breakstop = 0;
    err_write = 0;
    frames = 0;
  }
  }
  return NULL;
}


/**
 * Check the granularity of usleep.
 */
void UartDmxThread::CheckTimeGranularity() {
  TimeStamp ts1, ts2;
  Clock clock;
  /** If sleeping for 1ms takes longer than this, don't trust
   * usleep for this session
   */
  const int threshold = 3;

  clock.CurrentTime(&ts1);
  usleep(1000);
  clock.CurrentTime(&ts2);

  TimeInterval interval = ts2 - ts1;
  m_granularity = interval.InMilliSeconds() > threshold ? BAD : GOOD;
  OLA_INFO << "Granularity for UART thread is "
           << (m_granularity == GOOD ? "GOOD" : "BAD");
}
}  // namespace uartdmx
}  // namespace plugin
}  // namespace ola
