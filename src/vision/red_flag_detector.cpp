#include "ctf_navigation/vision/red_flag_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <vector>

namespace ctf_navigation
{
namespace vision
{

RedDetection detectRedBlob(const cv::Mat& bgr, const RedDetectorConfig& cfg)
{
  RedDetection result;

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  const cv::Scalar lower1(cfg.h_low1, cfg.s_min, cfg.v_min);
  const cv::Scalar upper1(cfg.h_high1, 255, 255);
  const cv::Scalar lower2(cfg.h_low2, cfg.s_min, cfg.v_min);
  const cv::Scalar upper2(cfg.h_high2, 255, 255);

  cv::Mat mask1, mask2;
  cv::inRange(hsv, lower1, upper1, mask1);
  cv::inRange(hsv, lower2, upper2, mask2);
  cv::bitwise_or(mask1, mask2, result.mask);

  if (cfg.use_rgb_fallback)
  {
    std::vector<cv::Mat> bgr_channels;
    cv::split(bgr, bgr_channels);

    cv::Mat red_min_mask;
    cv::threshold(bgr_channels[2], red_min_mask, cfg.rgb_red_min, 255,
                  cv::THRESH_BINARY);

    cv::Mat r32, g32, b32;
    bgr_channels[2].convertTo(r32, CV_32F);
    bgr_channels[1].convertTo(g32, CV_32F, cfg.rgb_red_dominance);
    bgr_channels[0].convertTo(b32, CV_32F, cfg.rgb_red_dominance);

    cv::Mat red_gt_green, red_gt_blue, rgb_mask;
    cv::compare(r32, g32, red_gt_green, cv::CMP_GT);
    cv::compare(r32, b32, red_gt_blue, cv::CMP_GT);
    cv::bitwise_and(red_min_mask, red_gt_green, rgb_mask);
    cv::bitwise_and(rgb_mask, red_gt_blue, rgb_mask);
    cv::bitwise_or(result.mask, rgb_mask, result.mask);
  }

  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(result.mask, result.mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(result.mask, result.mask, cv::MORPH_DILATE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(result.mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty())
  {
    return result;
  }

  size_t best_idx = 0;
  double best_area = 0.0;
  for (size_t i = 0; i < contours.size(); ++i)
  {
    const double a = cv::contourArea(contours[i]);
    if (a > best_area)
    {
      best_area = a;
      best_idx = i;
    }
  }

  result.area = best_area;
  if (result.area < cfg.min_blob_area)
  {
    result.found = false;
    return result;
  }
  if (cfg.max_blob_area > 0 && result.area > cfg.max_blob_area)
  {
    result.found = false;
    return result;
  }

  // ── Filtro de aspect ratio (ancho / alto) ──
  // Rechaza blobs cuya forma no encaja con la bandera (cartulina rectangular).
  // Extintores (altos y estrechos) tienen aspect ratio bajo (< 0.4).
  // Paredes largas y horizontales tienen aspect ratio muy alto (> 5).
  if (cfg.min_aspect_ratio > 0.0 || cfg.max_aspect_ratio < 99.0)
  {
    const cv::Rect br = cv::boundingRect(contours[best_idx]);
    const double aspect = (br.height > 0)
                              ? static_cast<double>(br.width) / br.height
                              : 0.0;
    if (aspect < cfg.min_aspect_ratio || aspect > cfg.max_aspect_ratio)
    {
      result.found = false;
      return result;
    }
  }

  const cv::Moments m = cv::moments(contours[best_idx]);
  if (m.m00 <= 0.0)
  {
    return result;
  }

  result.centroid_x = m.m10 / m.m00;
  result.found = true;
  return result;
}

double bearingFromCentroid(double centroid_x, int image_width, double horizontal_fov)
{
  return (0.5 * static_cast<double>(image_width) - centroid_x) /
         static_cast<double>(image_width) * horizontal_fov;
}

}  // namespace vision
}  // namespace ctf_navigation
