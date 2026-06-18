#include "ctf_navigation/vision/aruco_flag_detector.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

namespace ctf_navigation
{
namespace vision
{

ArucoDetection detectArucoFlag(const cv::Mat& bgr, const ArucoDetectorConfig& cfg)
{
  ArucoDetection result;

  // Crear diccionario y parámetros de detección
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cfg.dictionary_id);
  cv::Ptr<cv::aruco::DetectorParameters> params =
      cv::aruco::DetectorParameters::create();

  // Detectar todos los marcadores en la imagen
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(bgr, dictionary, corners, ids, params);

  if (ids.empty())
  {
    return result;
  }

  // Buscar el marcador con el ID configurado
  int target_idx = -1;
  for (size_t i = 0; i < ids.size(); ++i)
  {
    if (ids[i] == cfg.marker_id)
    {
      target_idx = static_cast<int>(i);
      break;
    }
  }

  if (target_idx < 0)
  {
    return result;
  }

  // Calcular centroide como media de las 4 esquinas
  const auto& c = corners[static_cast<size_t>(target_idx)];
  result.corners = c;

  double cx = 0.0, cy = 0.0;
  for (const auto& pt : c)
  {
    cx += pt.x;
    cy += pt.y;
  }
  result.centroid_x = cx / 4.0;
  result.centroid_y = cy / 4.0;

  // Área del cuadrilátero (Shoelace formula)
  double area = 0.0;
  for (size_t i = 0; i < c.size(); ++i)
  {
    size_t j = (i + 1) % c.size();
    area += c[i].x * c[j].y;
    area -= c[j].x * c[i].y;
  }
  result.area = std::abs(area) / 2.0;

  result.found = true;
  return result;
}

}  // namespace vision
}  // namespace ctf_navigation
