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


#include <cstdlib>
#include <iostream>

#include <opendavinci/odcore/base/KeyValueConfiguration.h>
#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/strings/StringToolbox.h>

#include <odvdminiature/GeneratedHeaders_ODVDMiniature.h>

#include "Navigation.h"

namespace opendlv {
namespace logic {
namespace miniature {

/*
  Constructor.
*/
Navigation::Navigation(const int &argc, char **argv)
    : TimeTriggeredConferenceClientModule(argc, argv, "logic-miniature-navigation")
    , m_mutex()
    , m_analogReadings()
    , m_gpioReadings()
    , m_gpioOutputPins()
    , m_pwmOutputPins()
{
}

/*
  Destructor.
*/
Navigation::~Navigation()
{
}

/*
  This method reads values from the configuration file. Note that there is only
  one global configuration storage loaded by the central odsupercomponent
  module. If the the configuration file is changed, the odsupercompnent module
  needs to be restarted.
*/
void Navigation::setUp()
{
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
  std::string const gpioPinsString =
      kv.getValue<std::string>("logic-miniature-navigation.gpio-pins");
  std::vector<std::string> gpioPinsVector =
      odcore::strings::StringToolbox::split(gpioPinsString, ',');
  for (auto pin : gpioPinsVector) {
    m_gpioOutputPins.push_back(std::stoi(pin));
  }

  std::string const pwmPinsString =
      kv.getValue<std::string>("logic-miniature-navigation.pwm-pins");
  std::vector<std::string> pwmPinsVector =
      odcore::strings::StringToolbox::split(pwmPinsString, ',');
  for (auto pin : pwmPinsVector) {
    m_pwmOutputPins.push_back(std::stoi(pin));
  }
}

/*
  This method is run automatically when the system is shutting down (before the
  destructor). It is typically used to close log files and de-allocate
  dynamically allocated memory.
*/
void Navigation::tearDown()
{
}

/*
  The while loop in this method runs at a predefined (in configuration)
  frequency.
*/
odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Navigation::body()
{
  std::string FSMstate = "wander";
  uint32_t pwmValueLeftWheel = 35000;
  uint32_t pwmValueRightWheel = 33500;
  int leftForward = 1;
  int rightForward = 1;
  int counter = 0;
  int blinkLED = 0;
  opendlv::proxy::ToggleRequest::ToggleState leftRotation_1;
  opendlv::proxy::ToggleRequest::ToggleState leftRotation_2;
  opendlv::proxy::ToggleRequest::ToggleState rightRotation_1;
  opendlv::proxy::ToggleRequest::ToggleState rightRotation_2;
opendlv::proxy::ToggleRequest::ToggleState blinkLED_state;
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

    // The mutex is required since 'body' and 'nextContainer' competes by
    // reading and writing to the class global maps, see also 'nextContainer'.
    odcore::base::Lock l(m_mutex);

    //// Example below.


    // Print some data collected from the 'nextContainer' method below.
    float voltageReadingPin0 = m_analogReadings[0];
    std::cout << "Reading from analog pin 0: " << voltageReadingPin0 << std::endl;

    int leftWhiskerActive = m_gpioReadings[26];
    int rightWhiskerActive = m_gpioReadings[46];
   
    std::cout << "leftWhiskerActive: " << leftWhiskerActive << std::endl;
    std::cout << "rightWhiskerActive: " << rightWhiskerActive << std::endl;
    
    std::cout << "State: " << FSMstate << std::endl;
    std::cout << "Counter: " << counter << std::endl;


    // default values for wander base state
    pwmValueLeftWheel = 35000;
    pwmValueRightWheel = 33500;
    leftForward = 1;
    rightForward = 1;
    blinkLED = 0;

    if (leftWhiskerActive && rightWhiskerActive && FSMstate!="backUp" && FSMstate!="rotate") {
     FSMstate = "backUp";
    }
    else if (leftWhiskerActive && FSMstate!="backUp" && FSMstate!="rotate") {
     // pwmValueRightWheel = 10000;
	FSMstate = "backUp";
    }
    else if (rightWhiskerActive && FSMstate!="backUp" && FSMstate!="rotate") {
     // pwmValueLeftWheel = 10000;
	FSMstate = "backUp";
    }

    if (FSMstate == "backUp") {
      blinkLED = 1;
      leftForward = 0;
      rightForward = 0;
      if (counter > 20) {
        FSMstate = "rotate";
        counter = 0;
      }
      counter++;
    }
    if (FSMstate == "rotate") {
std::cout << "################################: " << std::endl;
std::cout << "################################: " << std::endl;
std::cout << "################################: " << std::endl;
std::cout << "################################: " << std::endl;
std::cout << "################################: " << std::endl;
std::cout << "################################: " << std::endl;

      leftForward = 0;
      rightForward = 1;
      if (counter > 15) {
        FSMstate = "wander";
        counter = 0;
      }
      counter++;
    }



    if (leftForward) {
      leftRotation_1 = opendlv::proxy::ToggleRequest::Off;
      leftRotation_2 = opendlv::proxy::ToggleRequest::On;
    } else {
      leftRotation_1 = opendlv::proxy::ToggleRequest::On;
      leftRotation_2 = opendlv::proxy::ToggleRequest::Off;
    }

    if (rightForward) {
      rightRotation_1 = opendlv::proxy::ToggleRequest::On;
      rightRotation_2 = opendlv::proxy::ToggleRequest::Off;
    } else {
      rightRotation_1 = opendlv::proxy::ToggleRequest::Off;
      rightRotation_2 = opendlv::proxy::ToggleRequest::On;
    }

    if (blinkLED) {
      blinkLED_state = opendlv::proxy::ToggleRequest::On;
    } else {
      blinkLED_state = opendlv::proxy::ToggleRequest::Off;
    }

    opendlv::proxy::ToggleRequest requestLED(47, blinkLED_state);
    odcore::data::Container cLED(requestLED);
    getConference().send(cLED);

// ROTATION.
    opendlv::proxy::ToggleRequest request30(30, rightRotation_1);
    odcore::data::Container c30(request30);
    getConference().send(c30);

    opendlv::proxy::ToggleRequest request31(31, rightRotation_2);
    odcore::data::Container c31(request31);
    getConference().send(c31);

    opendlv::proxy::ToggleRequest request60(60, leftRotation_1);
    odcore::data::Container c60(request60);
    getConference().send(c60);

    opendlv::proxy::ToggleRequest request51(51, leftRotation_2);
    odcore::data::Container c51(request51);
    getConference().send(c51);


// VELOCITY.
    opendlv::proxy::PwmRequest requestRightWheel(0, pwmValueRightWheel);
    odcore::data::Container cRightWheel(requestRightWheel);
    cRightWheel.setSenderStamp(1);
    getConference().send(cRightWheel);

    opendlv::proxy::PwmRequest requestLeftWheel(0, pwmValueLeftWheel);
    odcore::data::Container cLeftWheel(requestLeftWheel);
    cLeftWheel.setSenderStamp(2);
    getConference().send(cLeftWheel);


      std::cout << "[" << getName() << "] Sending PwmRequest: "
          << requestRightWheel.toString() << std::endl;

          std::cout << "[" << getName() << "] Sending PwmRequest: "
              << requestLeftWheel.toString() << std::endl;
  //  }

    ///// Example above.

    ///// TODO: Add proper behaviours.
    std::cout << "TODO: Add proper behaviour." << std::endl;
  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

/*
  This method receives messages from all other modules (in the same conference
  id, cid). Here, the messages AnalogReading and ToggleReading is received
  from the modules interfacing to the hardware.
*/
void Navigation::nextContainer(odcore::data::Container &a_c)
{
  odcore::base::Lock l(m_mutex);

  int32_t dataType = a_c.getDataType();
  if (dataType == opendlv::proxy::AnalogReading::ID()) {
    opendlv::proxy::AnalogReading reading =
        a_c.getData<opendlv::proxy::AnalogReading>();

    uint16_t pin = reading.getPin();
    float voltage = reading.getVoltage();

    m_analogReadings[pin] = voltage; // Save the input to the class global map.

    std::cout << "[" << getName() << "] Received an AnalogReading: "
        << reading.toString() << "." << std::endl;

  } else if (dataType == opendlv::proxy::ToggleReading::ID()) {
    opendlv::proxy::ToggleReading reading =
        a_c.getData<opendlv::proxy::ToggleReading>();

    uint16_t pin = reading.getPin();
    bool state;
    if (reading.getState() == opendlv::proxy::ToggleReading::On) {
      state = true;
    } else {
      state = false;
    }

    m_gpioReadings[pin] = state; // Save the state to the class global map.

    std::cout << "[" << getName() << "] Received a ToggleReading: "
        << reading.toString() << "." << std::endl;
  }
}

}
}
}
