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

#ifndef PROXY_MINIATURE_PING_H
#define PROXY_MINIATURE_PING_H


#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
namespace opendlv {
namespace proxy {
namespace miniature {

/**
 * Interface to PING.
 */
class Ping : public odcore::base::module::TimeTriggeredConferenceClientModule {
 public:
  Ping(const int &, char **);
  Ping(const Ping &) = delete;
  Ping &operator=(const Ping &) = delete;
  virtual ~Ping();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();
  virtual odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

  void SendPulse(uint16_t const);
  int64_t GetTimeOfFlight(uint16_t const);
  void OpenGpio();
  void CloseGpio();
  void Reset();
  void SetDirection(uint16_t const, std::string);
  std::string GetDirection(uint16_t const) const;
  void SetValue(uint16_t const, bool const);
  bool GetValue(uint16_t const) const;

  bool m_debug;
  bool m_initialised;
  std::string m_path;
  std::vector<uint16_t> m_pins;
};

}
}
}

#endif
