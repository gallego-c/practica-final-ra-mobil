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
  int dictionary_id = 0;    // cv::aruco::DICT_4X4_50 = 0 (preferido, se prueban todos)
};

struct ArucoDetection
{
  bool found = false;
  double centroid_x = -1.0;  // columna del centroide (px)
  double centroid_y = -1.0;  // fila del centroide (px)
  double area = 0.0;         // área del marcador (px²)
  int detected_dict_id = -1; // diccionario que lo detectó
  int detected_marker_id = -1; // ID real del marcador detectado
  std::vector<cv::Point2f> corners;  // 4 esquinas (para debug)
};

/// Detecta un marcador ArUco específico en la imagen BGR.
/// Prueba primero el diccionario configurado; si falla, prueba todos los demás.
ArucoDetection detectArucoFlag(const cv::Mat& bgr, const ArucoDetectorConfig& cfg);

/// Nombre legible del diccionario ArUco (para logging).
const char* arucoDictName(int id);

}  // namespace vision
}  // namespace ctf_navigation

#endif
