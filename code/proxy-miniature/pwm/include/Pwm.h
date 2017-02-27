/**
 * proxy-miniature-pwm - Interface to pwm.
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

#ifndef PROXY_MINIATURE_PWM_H
#define PROXY_MINIATURE_PWM_H


#include <memory>
#include <string>
#include <vector>
#include <utility>

#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>

namespace opendlv {
namespace proxy {
namespace miniature {

/**
 * Interface to PWM.
 */
class Pwm : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  Pwm(const int &, char **);
  Pwm(const Pwm &) = delete;
  Pwm &operator=(const Pwm &) = delete;
  virtual ~Pwm();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();

  void OpenPwm();
  void ClosePwm();
  void Reset();
  void SetEnabled(uint16_t const, bool const);
  bool GetEnabled(uint16_t const) const;
  void SetDutyCycleNs(uint16_t const, uint32_t const);
  uint32_t GetDutyCycleNs(uint16_t const) const;
  void SetPeriodNs(uint16_t const, uint32_t const);
  uint32_t GetPeriodNs(uint16_t const) const;

  bool m_debug;
  bool m_initialised;
  std::string m_path;
  std::vector<uint16_t> m_pins;
  std::vector<uint32_t> m_periodsNs;
  std::vector<uint32_t> m_dutyCyclesNs;
};

}
}
}

#endif
