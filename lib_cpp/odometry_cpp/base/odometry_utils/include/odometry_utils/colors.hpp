/*
 * Copyright 2026 Maximilian Leitenstern, Marcel Weinmann
 *
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

#pragma once
#include <string>

namespace tam::core::state::utils {
/**
 * @brief struct to get TUMcolor code from string
 *
 * @param[in] name                  - std::string:
 *                                    name of TUMcolor
 */
struct TUMcolor {
  explicit TUMcolor(const std::string name)
  {
    this->a = 255;
    // Presentation
    if (name == "Blue") {
      this->r = 0;
      this->g = 101;
      this->b = 189;
    } else if (name == "Blue1") {
      this->r = 0;
      this->g = 51;
      this->b = 89;
    } else if (name == "Blue2") {
      this->r = 0;
      this->g = 82;
      this->b = 147;
    } else if (name == "Blue3") {
      this->r = 100;
      this->g = 160;
      this->b = 200;
    } else if (name == "Blue4") {
      this->r = 152;
      this->g = 198;
      this->b = 234;
    } else if (name == "Gray1") {
      this->r = 51;
      this->g = 51;
      this->b = 51;
    } else if (name == "Gray2") {
      this->r = 127;
      this->g = 127;
      this->b = 127;
    } else if (name == "Gray3") {
      this->r = 204;
      this->g = 204;
      this->b = 204;
    } else if (name == "Ivory") {
      this->r = 218;
      this->g = 215;
      this->b = 203;
    } else if (name == "Orange") {
      this->r = 227;
      this->g = 114;
      this->b = 34;
    } else if (name == "Green") {
      this->r = 162;
      this->g = 173;
      this->b = 0;
    } else if (name == "Black") {
      this->r = 0;
      this->g = 0;
      this->b = 0;
      // Web
    } else if (name == "WEBBlueDark") {
      this->r = 7;
      this->g = 33;
      this->b = 64;
    } else if (name == "WEBBlueLight") {
      this->r = 94;
      this->g = 148;
      this->b = 212;
    } else if (name == "WEBYellow") {
      this->r = 254;
      this->g = 215;
      this->b = 2;
    } else if (name == "WEBOrange") {
      this->r = 247;
      this->g = 129;
      this->b = 30;
    } else if (name == "WEBPink") {
      this->r = 181;
      this->g = 92;
      this->b = 165;
    } else if (name == "WEBBlueBright") {
      this->r = 143;
      this->g = 129;
      this->b = 234;
    } else if (name == "WEBRed") {
      this->r = 234;
      this->g = 114;
      this->b = 55;
    } else if (name == "WEBGreen") {
      this->r = 159;
      this->g = 186;
      this->b = 54;
      // otherwise white
    } else {
      this->r = 255;
      this->g = 255;
      this->b = 255;
    }
  }

  std::uint8_t r, g, b, a;
};
}  // namespace tam::core::state::utils
