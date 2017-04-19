/**
 * Copyright (C) 2016 Chalmers Revere
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
 */

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <time.h>

#include <opendavinci/odcore/base/KeyValueConfiguration.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>

#include <odvdminiature/GeneratedHeaders_ODVDMiniature.h>

#include "Ping.h"

namespace opendlv {
namespace proxy {
namespace miniature {

Ping::Ping(const int &argc, char **argv)
    : TimeTriggeredConferenceClientModule(argc, argv, "proxy-miniature-ping")
    , m_debug()
    , m_initialised()
    , m_path()
    , m_pins()
{
}

Ping::~Ping() 
{
}

void Ping::setUp()
{
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();

  m_debug = (kv.getValue<int32_t>("proxy-miniature-ping.debug") == 1);

  m_path = kv.getValue<std::string>("proxy-miniature-ping.systemPath");

  std::string const pinsString = 
      kv.getValue<std::string>("proxy-miniature-ping.pins");
  std::vector<std::string> pinsVector = 
      odcore::strings::StringToolbox::split(pinsString, ',');

  for (uint32_t i = 0; i < pinsVector.size(); i++) {
    uint16_t pin = std::stoi(pinsVector.at(i));
    m_pins.push_back(pin);
  }
  if (m_debug) {
    std::cout << "[" << getName() << "] " << "Initialised pins: ";
    for (auto pin : m_pins) {
      std:: cout << pin << " ";
    }
    std::cout << std::endl;
  }

  OpenGpio();
  SetDirection(m_pins.at(0), "out");
  SetValue(m_pins.at(0), false);
  SetDirection(m_pins.at(1), "in");

  m_initialised = true;
}

void Ping::tearDown()
{
  CloseGpio();
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Ping::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == 
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    SendPulse(m_pins.at(0));
    int64_t duration = GetTimeOfFlight(m_pins.at(1));
    std::cout << "Time of flight (microseconds): " << duration << std::endl;
    std::cout << "Distance (centimeter): " << (duration / (2 * 29.1)) << std::endl;
  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

void Ping::SendPulse(uint16_t const a_pin)
{
  SetValue(a_pin, true);
  // Pulse duration of 10 ms
  struct timespec sleeptime;
  sleeptime.tv_sec = 0;
  sleeptime.tv_nsec = 10000000L;
  nanosleep(&sleeptime, NULL);
  SetValue(a_pin, false);
}

int64_t Ping::GetTimeOfFlight(uint16_t const a_pin)
{
  odcore::data::TimeStamp begin;

  struct timespec sleeptime;
  sleeptime.tv_sec = 0;
  sleeptime.tv_nsec = 1000L;

  odcore::data::TimeStamp timestamp;
  const int64_t TIMEOUT_MICROSECONDS = 10000;
  while (GetValue(a_pin) == 0  && 
      (timestamp - begin).toMicroseconds() < TIMEOUT_MICROSECONDS) {
    nanosleep(&sleeptime, NULL);
    timestamp = odcore::data::TimeStamp();
  }
  

  int64_t durationMs = (timestamp - begin).toMicroseconds();

  return durationMs;
}

void Ping::nextContainer(odcore::data::Container &)
{
}

void Ping::OpenGpio()
{
  std::string filename = m_path + "/export";
  std::ofstream exportFile(filename, std::ofstream::out);
  
  if (exportFile.is_open()) {
    for (auto pin : m_pins) {
      exportFile << pin;
      exportFile.flush();
    }
    Reset();
  } else {
    cerr << "[" << getName() << "] Could not open " << filename << "." 
        << std::endl;
  }
  exportFile.close();
}

void Ping::CloseGpio()
{
  std::string filename = m_path + "/unexport";
  std::ofstream unexportFile(filename, std::ofstream::out);
  
  if (unexportFile.is_open()) {
    for (auto pin : m_pins) {
      unexportFile << pin;
      unexportFile.flush();
    }
  } else {
    cerr << "[" << getName() << "] Could not open " << filename << "." 
        << std::endl;
  }
  unexportFile.close();
}

void Ping::Reset()
{
  for (uint16_t i = 0; i < m_pins.size(); i++) {
    uint16_t pin = m_pins[i];
    std::string initialDirection = "in";
    SetDirection(pin, initialDirection);
  }
}

void Ping::SetDirection(uint16_t const a_pin, std::string const a_str)
{
  std::string gpioDirectionFilename = m_path + "/gpio" + std::to_string(a_pin) 
      + "/direction";

  std::ofstream gpioDirectionFile(gpioDirectionFilename, std::ofstream::out);
  if (gpioDirectionFile.is_open()) {
    gpioDirectionFile << a_str;
    gpioDirectionFile.flush();
  } else {
    cerr << "[" << getName() << "] Could not open " << gpioDirectionFilename 
        << "." << std::endl;
  }

  gpioDirectionFile.close();
}

std::string Ping::GetDirection(uint16_t const a_pin) const
{
  std::string gpioDirectionFilename = m_path + "/gpio" + std::to_string(a_pin) 
      + "/direction";
  std::string line;

  std::ifstream gpioDirectionFile(gpioDirectionFilename, std::ifstream::in);
  if (gpioDirectionFile.is_open()) {
    std::getline(gpioDirectionFile, line);
    std::string direction = line;
    gpioDirectionFile.close();
    return direction;
  } else {
    cerr << "[" << getName() << "] Could not open " << gpioDirectionFilename 
        << "." << std::endl;
    gpioDirectionFile.close();
    return "";
  }
}

void Ping::SetValue(uint16_t const a_pin, bool const a_value)
{
  std::string gpioValueFilename = 
      m_path + "/gpio" + std::to_string(a_pin) + "/value";

  std::ofstream gpioValueFile(gpioValueFilename, std::ofstream::out);
  if (gpioValueFile.is_open()) {
    gpioValueFile << static_cast<uint16_t>(a_value);
    gpioValueFile.flush();
  } else {
    cerr << "[" << getName() << "] Could not open " << gpioValueFilename 
        << "." << std::endl;
  }
  gpioValueFile.close();
}

bool Ping::GetValue(uint16_t const a_pin) const
{
  std::string gpioValueFilename = 
      m_path + "/gpio" + std::to_string(a_pin) + "/value";
  std::string line;

  std::ifstream gpioValueFile(gpioValueFilename, std::ifstream::in);
  if (gpioValueFile.is_open()) {
    std::getline(gpioValueFile, line);
    bool value = (std::stoi(line) == 1);
    gpioValueFile.close();
    return value;
  } else {
    cerr << "[" << getName() << "] Could not open " << gpioValueFilename 
        << "." << std::endl;
    gpioValueFile.close();
    return NULL;
  }
}

}
}
}
