/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez.
 *  Copyright (c) 2022, Achala Athukorala
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <map_merge_2d/cv_core/cv_core.hpp>

using namespace map_merge_2d;

cv::Ptr<cv::Feature2D> cv_core::chooseFeatureFinder(FeatureType type)
{
  switch (type) {
    case FeatureType::AKAZE:
      return cv::AKAZE::create();
    case FeatureType::ORB:
      return cv::ORB::create();
    case FeatureType::SURF:
    #ifdef HAVE_OPENCV_XFEATURES2D
          return cv::xfeatures2d::SURF::create();
    #else
          return cv::AKAZE::create();
    #endif
  }

  assert(false);
  return {};
}

void cv_core::image_transform(const CVImage src, CVImage &dest, cv::Mat transform)
{
  // Create vertices of input image
  cv::Size size = src.image.size();
  std::vector<cv::Point2i> vertices = {
        {0,                     0},
        {size.width,            0},
        {size.width,  size.height},
        {0,           size.height}};

  // Transform input image vertices
  std::vector<cv::Point2i> transformed_vertices;
  cv::transform(vertices, transformed_vertices, transform);
  
  std::vector<double> x_cordinates, y_cordinates;
  for (auto &vertice : transformed_vertices)
  {
    x_cordinates.emplace_back(vertice.x);
    y_cordinates.emplace_back(vertice.y);
  }

  // Calculate transformed vertices min-max
  double min_x, max_x, min_y, max_y;
  cv::minMaxLoc(x_cordinates, &min_x, &max_x);
  cv::minMaxLoc(y_cordinates, &min_y, &max_y);

  // Create a new adjusted transform to align output image into valid coordinate space
  //  (opencv matrices cannot have negative pixel coordinates for images)
  cv::Mat adjusted_transform = transform.clone();
  adjusted_transform.at<float>(0,2) -= (_Float32)min_x;
  adjusted_transform.at<float>(1,2) -= (_Float32)min_y;

  // Adjust offset to account for transform modification
  dest.origin.x = src.origin.x + min_x;
  dest.origin.y = src.origin.y + min_y;

  // Transform image
  cv::Size dsize(max_x - min_x, max_y - min_y);
  cv::warpAffine(src.image, dest.image, adjusted_transform, dsize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255)); // 255: White border
} 

std::map<int, cv::Mat> cv_core::estimateTransforms(std::vector<cv::Mat> images, FeatureType feature_type, double confidence)
{
  std::vector<cv::detail::ImageFeatures> image_features;
  std::vector<cv::detail::MatchesInfo> pairwise_matches;
  std::vector<cv::detail::CameraParams> transforms;
  std::vector<int> good_indices;
  std::map<int, cv::Mat> image_transforms;

  auto finder = chooseFeatureFinder(feature_type);
  cv::Ptr<cv::detail::FeaturesMatcher> matcher = cv::makePtr<cv::detail::AffineBestOf2NearestMatcher>();
  cv::Ptr<cv::detail::Estimator> estimator = cv::makePtr<cv::detail::AffineBasedEstimator>();
  cv::Ptr<cv::detail::BundleAdjusterBase> adjuster = cv::makePtr<cv::detail::BundleAdjusterAffinePartial>();

  if (images.empty()) 
  {
    std::cout << "No images to match" << std::endl;
    return image_transforms;
  }

  /* find features in images */
  image_features.reserve(images.size());
  for (const cv::Mat& image : images) {
    image_features.emplace_back();
    if (!image.empty()) {
      cv::detail::computeImageFeatures(finder, image, image_features.back());
    }
  }
  
  /* find corespondent features */
  (*matcher)(image_features, pairwise_matches);

  #ifndef NDEBUG
    cv_core_helper::writeDebugMatchingInfo(images, image_features, pairwise_matches);
  #endif

  /* use only matches that has enough confidence. leave out matches that are not connected (small components) */
  good_indices = cv::detail::leaveBiggestComponent(image_features, pairwise_matches, static_cast<float>(confidence));

  /* estimate transform */
  if (!(*estimator)(image_features, pairwise_matches, transforms)) 
  {
    std::cout << "Error: Cannot estimate transform" << std::endl;
    return image_transforms;
  }

  /* levmarq optimization */
  // openCV just accepts float transforms
  for (auto& transform : transforms) {
    transform.R.convertTo(transform.R, CV_32F);
  }

  // TODO Fix? Bundle adjustment results in incorrect transformations for test images. Hence removed
  // adjuster->setConfThresh(confidence);
  // if (!(*adjuster)(image_features, pairwise_matches, transforms)) {
  //   std::cout << "Bundle adjusting failed. Could not estimate transforms." << std::endl;
  //   return;
  // }

  // If valid matches are found, indice could will be > 2
  if (good_indices.size() > 1)
  {
    for (uint i = 0; i < good_indices.size(); i++)
    {
      // insert estimated transforms into image_transforms, selecting 2D affine matrix only [2d rot, 2d trans]
      image_transforms.insert({good_indices.at(i), transforms.at(i).R(cv::Range(0,2), cv::Range(0,3))});
    }
  }

  return image_transforms;
}