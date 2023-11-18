/*
 * Copyright (c) 2023 Achala Athukorala <chhathuranga@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CV_CORE_HELPERS_H
#define CV_CORE_HELPERS_H

#include <opencv2/opencv.hpp>

namespace map_merge_2d
{
    namespace cv_core_helper
    {
        inline static void writeDebugMatchingInfo(
            const std::vector<cv::Mat>& images,
            const std::vector<cv::detail::ImageFeatures>& image_features,
            const std::vector<cv::detail::MatchesInfo>& pairwise_matches)
        {
            for (auto& match_info : pairwise_matches) {
                if (match_info.H.empty() ||
                    match_info.src_img_idx >= match_info.dst_img_idx) {
                continue;
                }
                // std::cout << match_info.src_img_idx << " " << match_info.dst_img_idx
                //         << std::endl
                //         << "features: "
                //         << image_features[size_t(match_info.src_img_idx)].keypoints.size()
                //         << " "
                //         << image_features[size_t(match_info.dst_img_idx)].keypoints.size()
                //         << std::endl
                //         << "matches: " << match_info.matches.size() << std::endl
                //         << "inliers: " << match_info.num_inliers << std::endl
                //         << "inliers/matches ratio: "
                //         << match_info.num_inliers / double(match_info.matches.size())
                //         << std::endl
                //         << "confidence: " << match_info.confidence << std::endl
                //         << match_info.H << std::endl;
                cv::Mat img;
                // draw all matches
                cv::drawMatches(images[size_t(match_info.src_img_idx)],
                                image_features[size_t(match_info.src_img_idx)].keypoints,
                                images[size_t(match_info.dst_img_idx)],
                                image_features[size_t(match_info.dst_img_idx)].keypoints,
                                match_info.matches, img);
                // cv::imwrite(std::to_string(match_info.src_img_idx) + "_" +
                //                 std::to_string(match_info.dst_img_idx) + "_matches.png",
                //             img);
                // draw inliers only
                // cv::imshow(std::to_string(match_info.src_img_idx) + "_" +
                //                 std::to_string(match_info.dst_img_idx) + "_matches", img);
                // cv::waitKey(0);
                cv::drawMatches(
                    images[size_t(match_info.src_img_idx)],
                    image_features[size_t(match_info.src_img_idx)].keypoints,
                    images[size_t(match_info.dst_img_idx)],
                    image_features[size_t(match_info.dst_img_idx)].keypoints,
                    match_info.matches, img, cv::Scalar::all(-1), cv::Scalar::all(-1)); // , *reinterpret_cast<const std::vector<char>*>(&match_info.inliers_mask)
                // cv::imwrite(std::to_string(match_info.src_img_idx) + "_" +
                //                 std::to_string(match_info.dst_img_idx) +
                //                 "_matches_inliers.png",
                //             img);
                cv::imshow(std::to_string(match_info.src_img_idx) + "_" +
                                std::to_string(match_info.dst_img_idx) + "_matches_inliers", img);
                cv::waitKey(500);
            }
        }
    } // cv_core_helper
} // map_merge_2d

#endif // CV_CORE_HELPERS_H