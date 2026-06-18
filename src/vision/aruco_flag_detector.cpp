#include "ctf_navigation/vision/aruco_flag_detector.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdio>

namespace ctf_navigation
{
namespace vision
{

// Intenta detectar el marcador en un diccionario concreto.
static bool tryDetect(const cv::Mat& input, int dict_id, int target_marker_id,
                      ArucoDetection& result)
{
  // ── Sharpening (unsharp mask) para imágenes borrosas ──
  cv::Mat sharpened;
  cv::GaussianBlur(input, sharpened, cv::Size(0, 0), 2.0);
  cv::addWeighted(input, 1.5, sharpened, -0.5, 0, sharpened);

  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(dict_id);
  cv::Ptr<cv::aruco::DetectorParameters> params =
      cv::aruco::DetectorParameters::create();

  // Parámetros más tolerantes para imágenes comprimidas/borrosas
  params->adaptiveThreshWinSizeMin = 3;
  params->adaptiveThreshWinSizeMax = 23;
  params->adaptiveThreshWinSizeStep = 4;
  params->minMarkerPerimeterRate = 0.01;   // detectar marcadores más pequeños
  params->maxMarkerPerimeterRate = 4.0;
  params->polygonalApproxAccuracyRate = 0.08;  // más tolerante con bordes imperfectos
  params->perspectiveRemoveIgnoredMarginPerCell = 0.2;

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(sharpened, dictionary, corners, ids, params);

  if (ids.empty())
  {
    return false;
  }

  // Buscar el marcador con el ID objetivo
  int target_idx = -1;
  for (size_t i = 0; i < ids.size(); ++i)
  {
    if (ids[i] == target_marker_id)
    {
      target_idx = static_cast<int>(i);
      break;
    }
  }

  if (target_idx < 0)
  {
    return false;
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
  result.detected_dict_id = dict_id;
  return true;
}

// Nombres legibles de los diccionarios para logging.
static const char* dictName(int id)
{
  switch (id)
  {
    case 0: return "4X4_50";
    case 1: return "4X4_100";
    case 2: return "4X4_250";
    case 3: return "4X4_1000";
    case 4: return "5X5_50";
    case 5: return "5X5_100";
    case 6: return "5X5_250";
    case 7: return "5X5_1000";
    case 8: return "6X6_50";
    case 9: return "6X6_100";
    case 10: return "6X6_250";
    case 11: return "6X6_1000";
    case 12: return "7X7_50";
    case 13: return "7X7_100";
    case 14: return "7X7_250";
    case 15: return "7X7_1000";
    case 16: return "ARUCO_ORIGINAL";
    default: return "UNKNOWN";
  }
}

ArucoDetection detectArucoFlag(const cv::Mat& bgr, const ArucoDetectorConfig& cfg)
{
  ArucoDetection result;

  // 1. Intentar con el diccionario configurado
  if (tryDetect(bgr, cfg.dictionary_id, cfg.marker_id, result))
  {
    return result;
  }

  // 2. Si no se encontró, probar TODOS los diccionarios (auto-detect)
  //    Esto permite que funcione sin importar qué generador usó el usuario.
  for (int dict = 0; dict <= 16; ++dict)
  {
    if (dict == cfg.dictionary_id)
    {
      continue;  // Ya lo probamos arriba
    }
    if (tryDetect(bgr, dict, cfg.marker_id, result))
    {
      // Log a stderr para que el usuario vea qué diccionario funciona
      fprintf(stderr,
              "[ArUco] Marker id=%d found with dictionary %s (id=%d). "
              "Set aruco_dictionary: %d in vision_real.yaml for faster detection.\n",
              cfg.marker_id, dictName(dict), dict, dict);
      return result;
    }
  }

  return result;
}

const char* arucoDictName(int id)
{
  return dictName(id);
}

}  // namespace vision
}  // namespace ctf_navigation
