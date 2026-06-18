#ifndef CTF_NAVIGATION_VISION_ARUCO_FLAG_DETECTOR_HPP
#define CTF_NAVIGATION_VISION_ARUCO_FLAG_DETECTOR_HPP

#include <opencv2/core.hpp>
#include <vector>

namespace ctf_navigation
{
namespace vision
{

struct ArucoDetectorConfig
{
  int marker_id = 0;        // ID del marcador ArUco a buscar
  int dictionary_id = 0;    // cv::aruco::DICT_4X4_50 = 0
};

struct ArucoDetection
{
  bool found = false;
  double centroid_x = -1.0;  // columna del centroide (px)
  double centroid_y = -1.0;  // fila del centroide (px)
  double area = 0.0;         // área del marcador (px²)
  std::vector<cv::Point2f> corners;  // 4 esquinas (para debug)
};

/// Detecta un marcador ArUco específico en la imagen BGR.
ArucoDetection detectArucoFlag(const cv::Mat& bgr, const ArucoDetectorConfig& cfg);

}  // namespace vision
}  // namespace ctf_navigation

#endif
