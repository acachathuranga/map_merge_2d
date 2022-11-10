#ifndef CV_CORE_H
#define CV_CORE_H

#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <map_merge_2d/cv_core/cv_helpers.hpp>

// #define NDEBUG

namespace map_merge_2d
{
    namespace cv_core
    {
        /**
         * @brief Struct containing an image and global coordinate offset
         * 
         * @var image : cv::Mat object containing image data
         * @var origin: cv::Point2d object containing 'real' [x,y] coordinates of the (0,0) pixel in image
         * 
         * @note cv::Mat images only contain positive pixel coordinates. Hence, an image cannot be
         *              directly represented in negative pixel locations (e.g. Cannot show an
         *              image which was translated by -100, -100 pixels in x and y axes)
         *              To represent images in full cartesian pixel plane, the 'orgin' variable is used. 
         *              The 'image' matrix will infact contain image data in positive pixel coordinate system,
         *              but the 'origin' variable will track the absolute location of the (0,0) pixel in 'image' matrix.
         */
        struct CVImage
        {
            CVImage(cv::Mat image_=cv::Mat() , cv::Point2d origin_ = cv::Point2d())
            {
                image = image_;
                origin = origin_;
            }

            CVImage(cv::Mat image_, double x, double y)
            {
                image = image_;
                origin = cv::Point2d(x, y);
            }

            cv::Mat image;      /** Image */
            cv::Point2d origin; /** Offset of Image(0,0) pixel in global coordinate frame */
        };

        enum class FeatureType 
        { 
            AKAZE, 
            ORB, 
            SURF 
        };

        cv::Ptr<cv::Feature2D> chooseFeatureFinder(FeatureType type);

        void image_transform(const CVImage src, CVImage &dest, cv::Mat transform);
        std::map<int, cv::Mat> estimateTransforms(std::vector<cv::Mat> images, FeatureType feature_type, double confidence);

        
    } // namespace cv_core

}  // namespace map_merge_2d

#endif  // CV_CORE_H