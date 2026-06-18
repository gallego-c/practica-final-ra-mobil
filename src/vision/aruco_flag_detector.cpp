#include "ctf_navigation/vision/aruco_flag_detector.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdio>
#include <string>

namespace ctf_navigation
{
namespace vision
{

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
    case 17: return "APRILTAG_16h5";
    case 18: return "APRILTAG_25h9";
    case 19: return "APRILTAG_36h10";
    case 20: return "APRILTAG_36h11";
    default: return "UNKNOWN";
  }
}

// Parámetros tolerantes para imágenes borrosas/comprimidas.
static cv::Ptr<cv::aruco::DetectorParameters> makeTolerantParams()
{
  cv::Ptr<cv::aruco::DetectorParameters> params =
      cv::aruco::DetectorParameters::create();
  params->adaptiveThreshWinSizeMin = 3;
  params->adaptiveThreshWinSizeMax = 53;
  params->adaptiveThreshWinSizeStep = 4;
  params->adaptiveThreshConstant = 7;
  params->minMarkerPerimeterRate = 0.005;
  params->maxMarkerPerimeterRate = 4.0;
  params->polygonalApproxAccuracyRate = 0.1;
  params->minCornerDistanceRate = 0.02;
  params->minDistanceToBorder = 1;
  params->perspectiveRemoveIgnoredMarginPerCell = 0.25;
  params->maxErroneousBitsInBorderRate = 0.5;
  params->errorCorrectionRate = 1.0;  // máxima corrección de errores
  return params;
}

// Intenta detectar el marcador target_marker_id en un diccionario.
// Si target_marker_id == -1, acepta CUALQUIER marcador.
static bool tryDetect(const cv::Mat& image, int dict_id, int target_marker_id,
                      ArucoDetection& result,
                      std::vector<int>* all_found_ids = nullptr)
{
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(dict_id);
  cv::Ptr<cv::aruco::DetectorParameters> params = makeTolerantParams();

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(image, dictionary, corners, ids, params);

  // Guardar todos los IDs encontrados para logging
  if (all_found_ids && !ids.empty())
  {
    for (int id : ids)
    {
      all_found_ids->push_back(id);
    }
  }

  if (ids.empty())
  {
    return false;
  }

  // Buscar el marcador con el ID objetivo (-1 = acepta cualquiera)
  int target_idx = -1;
  for (size_t i = 0; i < ids.size(); ++i)
  {
    if (target_marker_id < 0 || ids[i] == target_marker_id)
    {
      target_idx = static_cast<int>(i);
      break;
    }
  }

  if (target_idx < 0)
  {
    return false;
  }

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
  result.detected_marker_id = ids[static_cast<size_t>(target_idx)];
  return true;
}

ArucoDetection detectArucoFlag(const cv::Mat& bgr, const ArucoDetectorConfig& cfg)
{
  ArucoDetection result;

  // Preparar dos versiones: original y con sharpening
  cv::Mat sharpened;
  cv::GaussianBlur(bgr, sharpened, cv::Size(0, 0), 2.0);
  cv::addWeighted(bgr, 1.5, sharpened, -0.5, 0, sharpened);

  const cv::Mat* images[] = { &bgr, &sharpened };
  const char* img_names[] = { "original", "sharpened" };

  // ── Fase 1: buscar el marker_id configurado en todos los diccionarios ──
  for (int img_idx = 0; img_idx < 2; ++img_idx)
  {
    for (int dict = 0; dict <= 20; ++dict)
    {
      if (tryDetect(*images[img_idx], dict, cfg.marker_id, result))
      {
        if (dict != cfg.dictionary_id)
        {
          fprintf(stderr,
                  "[ArUco] id=%d found with dict %s (%s image). "
                  "Set aruco_dictionary: %d in vision_real.yaml\n",
                  cfg.marker_id, dictName(dict), img_names[img_idx], dict);
        }
        return result;
      }
    }
  }

  // ── Fase 2: buscar CUALQUIER marcador para debug ──
  // Solo logear cada 3 segundos para no saturar
  static int scan_count = 0;
  if (++scan_count % 30 == 1)  // cada ~3s a 10fps
  {
    std::string found_summary;
    for (int dict = 0; dict <= 20; ++dict)
    {
      std::vector<int> found_ids;
      ArucoDetection dummy;
      tryDetect(bgr, dict, -1, dummy, &found_ids);
      if (!found_ids.empty())
      {
        found_summary += dictName(dict);
        found_summary += ":[";
        for (size_t i = 0; i < found_ids.size(); ++i)
        {
          if (i > 0) found_summary += ",";
          found_summary += std::to_string(found_ids[i]);
        }
        found_summary += "] ";
      }
    }
    if (!found_summary.empty())
    {
      fprintf(stderr,
              "[ArUco] Target id=%d NOT found, but OTHER markers detected: %s\n",
              cfg.marker_id, found_summary.c_str());
    }
    else
    {
      fprintf(stderr,
              "[ArUco] NO markers detected at all in any dictionary (image %dx%d). "
              "Check lighting, camera focus, and marker print quality.\n",
              bgr.cols, bgr.rows);
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
