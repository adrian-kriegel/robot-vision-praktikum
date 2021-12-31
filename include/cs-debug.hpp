#ifndef AK_DEBUG_HPP
#define AK_DEBUG_HPP

#include <math.h>

#include <image/Img.h>

#include "util.hpp"

namespace cs_debug
{
  void draw_background(
    rv::GrayImage* img,
    rv::GrayImage* background,
    double factor = 0.2
  )
  {
    for (uint i = 0; i < img->width(); i++)
    {
      for (uint j = 0; j < img->height(); j++)
      {
        (*img)(i,j) = (*background)(i,j) * factor;
      }
    }
  }

  void draw_point(
    rv::GrayImage* img,
    double x, double y,
    uint color = 255,
    int radius = 4
  )
  {
    int xi = x*img->width();
    int yi = (1.0-y)*img->height();

    for (int i = std::max(xi-radius, 0); i < std::min(xi+radius, img->width() - 1); i++)
    {
      for (int j = std::max(yi-radius, 0); j < std::min(yi+radius, img->width() - 1); j++)
      {
        if ((i-xi)*(i-xi)+(j-yi)*(j-yi) <= radius*radius)
        {
          (*img)(i,j) = color;
        }
      }
    }
  }
}; // namespace cs_debug

#endif // AK_DEBUG_HPP
