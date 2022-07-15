/* 
 * Copyright 2019 Southwest Research Institute
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
/* 
 * Copyright 2019 Southwest Research Institute
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
#ifndef REACH_VISUALIZER_MESH_COLORIZATION_UTILS_H
#define REACH_VISUALIZER_MESH_COLORIZATION_UTILS_H

#include <vector>

namespace reach_visualizer
{
namespace utils
{

struct rgb
{
  double r;       // a fraction between 0 and 1
  double g;       // a fraction between 0 and 1
  double b;       // a fraction between 0 and 1
};

struct hsv
{
  double h;       // angle in degrees
  double s;       // a fraction between 0 and 1
  double v;       // a fraction between 0 and 1
};

hsv rgb2hsv(const rgb& in);

rgb hsv2rgb(const hsv& in);

/**
 * @brief scoresToColors converts reach study pose scores to colors given a maximum score threshold
 * @param scores
 * @param max_score
 * @return
 */
std::vector<rgb> scoresToColors(const std::vector<double> &scores,
                                const double max_score);

/**
 * @brief scoresToColors converts reach study pose scores to colors given no maximum score threshold
 * @param scores
 * @return
 */
std::vector<rgb> scoresToColors(const std::vector<double>& scores);

} // namespace utils
} // namespace reach_visualizer

#endif // REACH_VISUALIZER_MESH_COLORIZATION_UTILS_H
