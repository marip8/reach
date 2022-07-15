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
#include <algorithm>
#include <reach_visualizer/colorization_utils.h>

// TO-DO: Give credit to StackOverflow creator of color utils

namespace reach_visualizer
{
namespace utils
{

hsv rgb2hsv(const rgb& in)
{
  hsv out;
  double min, max, delta;

  min = in.r < in.g ? in.r : in.g;
  min = min  < in.b ? min  : in.b;

  max = in.r > in.g ? in.r : in.g;
  max = max  > in.b ? max  : in.b;

  out.v = max; // v
  delta = max - min;
  if (delta < 0.00001)
  {
      out.s = 0;
      out.h = 0; // undefined, maybe nan?
      return out;
  }

  if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
      out.s = (delta / max);                  // s
  }
  else
  {
      // if max is 0, then r = g = b = 0
      // s = 0, h is undefined
      out.s = 0.0;
      out.h = NAN;                            // its now undefined
      return out;
  }

  if( in.r >= max )
  {
    // > is bogus, just keeps compilor happy
      out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
  }
  else
  {
    if( in.g >= max )
    {
      out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    }
    else
    {
      out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan
    }
  }

  out.h *= 60.0;                              // degrees

  if( out.h < 0.0 )
  {
    out.h += 360.0;
  }

  return out;
}

rgb hsv2rgb(const hsv& in)
{
  double hh, p, q, t, ff;
  long i;
  rgb out;

  if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
      out.r = in.v;
      out.g = in.v;
      out.b = in.v;
      return out;
  }

  hh = in.h;

  if(hh >= 360.0)
  {
    hh = 0.0;
  }

  hh /= 60.0;
  i = (long)hh;
  ff = hh - i;
  p = in.v * (1.0 - in.s);
  q = in.v * (1.0 - (in.s * ff));
  t = in.v * (1.0 - (in.s * (1.0 - ff)));

  switch(i)
  {
  case 0:
      out.r = in.v;
      out.g = t;
      out.b = p;
      break;
  case 1:
      out.r = q;
      out.g = in.v;
      out.b = p;
      break;
  case 2:
      out.r = p;
      out.g = in.v;
      out.b = t;
      break;

  case 3:
      out.r = p;
      out.g = q;
      out.b = in.v;
      break;
  case 4:
      out.r = t;
      out.g = p;
      out.b = in.v;
      break;
  case 5:
  default:
      out.r = in.v;
      out.g = p;
      out.b = q;
      break;
  }

  return out;
}

std::vector<rgb> scoresToColors(const std::vector<double> &scores,
                                const double max_score)
{
  std::vector<rgb> colors;
  colors.reserve(scores.size());

  const static double max_hue = 240.0f;
  for(const double score : scores)
  {
    hsv hsv_color;
    hsv_color.h = ((score - 0.0f) / (max_score - 0.0f)) * max_hue;
    hsv_color.s = 1.0;
    if(score <= 0.0f || std::isnan(score))
    {
      // Make negative or NaN scores black
      hsv_color.v = 0.0;
    }
    else
    {
      hsv_color.v = 0.85;
    }

    colors.push_back(hsv2rgb(hsv_color));
  }

  return colors;
}

std::vector<rgb> scoresToColors(const std::vector<double>& scores)
{
  double max_score = *(std::max_element(scores.begin(), scores.end()));
  return scoresToColors(scores, max_score);
}

} // namespace utils
} // namespace reach_visualizer
