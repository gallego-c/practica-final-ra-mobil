#ifndef CTF_NAVIGATION_VISION_RED_FLAG_DETECTOR_HPP
#define CTF_NAVIGATION_VISION_RED_FLAG_DETECTOR_HPP

#include <opencv2/core.hpp>

namespace ctf_navigation
{
namespace vision
{

struct RedDetectorConfig
{
  int h_low1 = 0;
  int h_high1 = 10;
  int h_low2 = 170;
  int h_high2 = 180;
  int s_min = 120;
  int v_min = 70;
  int min_blob_area = 200;
  bool use_rgb_fallback = true;
  int rgb_red_min = 80;
  double rgb_red_dominance = 1.25;
};

struct RedDetection
{
  bool found = false;
  double centroid_x = -1.0;  // pixel column
  double area = 0.0;
  cv::Mat mask;              // optional debug mask (may be empty)
};

/// Segmenta el color rojo en BGR y devuelve el blob más grande.
RedDetection detectRedBlob(const cv::Mat& bgr, const RedDetectorConfig& cfg);

/// Bearing horizontal (rad): positivo = izquierda del centro de la imagen.
double bearingFromCentroid(double centroid_x, int image_width, double horizontal_fov);

}  // namespace vision
}  // namespace ctf_navigation

#endif
