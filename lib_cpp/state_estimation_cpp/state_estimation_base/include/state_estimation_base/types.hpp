/*
 * Copyright 2023 Marcel Weinmann, Maximilian Leitenstern
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
namespace tam::types::state::measurements
{
    /**
    * @brief Kalman filter base constants: Order of the states in the Kalman filter
    */
    enum TYPE {POS, ORIENTATION, VEL, IMU};

    /**
    * @brief struct buffering the received odomety input and corresponding timestamp and config
    */
    struct identifier
    {
        TYPE type{};
        uint8_t num{};
        bool operator==(const identifier& other) const {
                return type == other.type && num == other.num;
        }
    };

    /**
    * @brief hash function to use the identifier type in a unordered_map
    */
    struct identifier_hash {
        std::size_t operator()(const tam::types::state::measurements::identifier& id) const {
            return (std::hash<int>{}(id.type) << 8) ^ std::hash<uint8_t>{}(id.num);
        }
    };
} // tam::types::state::measurements

namespace tam::types::state
{
    /**
    * @brief definition of a unordered_map with the identifier as key
    */
    template <typename T>
    using unordered_identifier_map = std::unordered_map<tam::types::state::measurements::identifier, T, tam::types::state::measurements::identifier_hash>;
} // tam::types::state
