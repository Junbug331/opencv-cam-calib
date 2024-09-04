#include <iostream>
#include <tuple>
#include <vector>
#include <regex>
#include <filesystem>
namespace fs = std::filesystem;

#include "OpenCVCamCalib.hpp"
#include <spdlog/spdlog.h>


double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>> &objectPoints,
                                 const std::vector<std::vector<cv::Point2f>> &imagePoints,
                                 const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
                                 const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                 std::vector<float> &perViewErrors, bool fisheye)
{
    using namespace std;
    using namespace cv;

    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr    = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (size_t i = 0; i < objectPoints.size(); ++i)
    {
        if (fisheye)
        {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                   distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }
        cout << "imagePoints2 size is: " << imagePoints2.size() << endl;
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n         = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);

    return 0.0;
}

std::optional<std::tuple<std::vector<cv::Point2f>, cv::Mat>> FindCheckerboard(std::filesystem::path a_oPath,
                                                                              BOARD_PATTERN a_eBoardPattern,
                                                                              const int32_t a_nBoardWidth,
                                                                              const int32_t a_nBoardHeight,
                                                                              bool a_bUseFisheye)
{
    auto oImage(cv::imread(a_oPath.string(), 0));

    std::optional<std::tuple<std::vector<cv::Point2f>, cv::Mat>> oResult;
    std::vector<cv::Point2f> cornerPts;

    int nChessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;

    if (!a_bUseFisheye)
    {
        // fast check erroneously fails with high distortions like fisheye
        nChessBoardFlags |= cv::CALIB_CB_FAST_CHECK;
    }

    if (a_eBoardPattern == BOARD_PATTERN::CHESSBOARD &&
        cv::findChessboardCorners(oImage,
                                  cv::Size(a_nBoardWidth, a_nBoardHeight),
                                  cornerPts, nChessBoardFlags))
    {

        oResult.emplace(std::make_tuple(std::move(cornerPts), std::move(oImage)));
    }
    else if (a_eBoardPattern == BOARD_PATTERN::CIRCLES_GRID &&
             cv::findCirclesGrid(oImage,
                                 cv::Size(a_nBoardWidth, a_nBoardHeight),
                                 cornerPts, cv::CALIB_CB_SYMMETRIC_GRID))
    {
        oResult.emplace(std::make_tuple(std::move(cornerPts), std::move(oImage)));
    }
    else
    {
        /*
        try
        {
            if (fs::remove(a_oPath))
            {
                std::cout << "File " << a_oPath << " was deleted successfully.\n";
            }
            else
            {
                std::cout << "File " << a_oPath << " not found.\n";
            }
        }
        catch (const fs::filesystem_error &e)
        {
            std::cerr << "Error: " << e.what() << "\n";
        }
        */
    }

    return oResult;
}

bool CheckChessBoard(const cv::Mat &a_rImage, int a_nBoardWidth, int a_nBoardHeight, float a_fThreshold, float *a_pPixelRatio, bool a_bShowDebug)
{
    int nBoardWidth  = a_nBoardWidth - 1;
    int nBoardHeight = a_nBoardHeight - 1;
    cv::Mat oGray;
    if (a_rImage.channels() > 1)
    {
        cv::cvtColor(a_rImage, oGray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        oGray = a_rImage;
    }
    std::vector<cv::Point2f> cornerPts;
    if (!cv::findChessboardCorners(oGray, cv::Size(nBoardWidth, nBoardHeight),
                                   cornerPts, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK))
    {
        spdlog::info("Can't detect chessboard on input image");
        return false;
    }

    // Determine bounding rectangle
    cv::Rect boundingRect = cv::boundingRect(cornerPts);
    cv::Mat croppedImage  = oGray(boundingRect);

    std::vector<cv::Point2f> croppedCornerPts(cornerPts.size());
    for (int i = 0; i < cornerPts.size(); ++i)
    {
        croppedCornerPts[i] = cornerPts[i] - cv::Point2f(boundingRect.tl().x, boundingRect.tl().y);
    }

    int height_ = nBoardHeight - 1;
    int width_  = nBoardWidth - 1;

    int y = 0;
    int x = 0;
    std::vector<std::vector<cv::Point>> pts(1, std::vector<cv::Point>(4));
    cv::Point2f p0 = croppedCornerPts[(y * nBoardWidth) + x];
    cv::Point2f p1 = croppedCornerPts[(y * nBoardWidth) + (x + width_)];
    cv::Point2f p2 = croppedCornerPts[(y + height_) * nBoardWidth + (x + width_)];
    cv::Point2f p3 = croppedCornerPts[(y + height_) * nBoardWidth + (x)];

    pts[0][0] = p0;
    pts[0][1] = p1;
    pts[0][2] = p2;
    pts[0][3] = p3;

    cv::Mat mask = cv::Mat::zeros({boundingRect.width, boundingRect.height}, CV_8UC1);
    cv::fillPoly(mask, pts, cv::Scalar(255));
    float fArea = static_cast<float>(cv::countNonZero(mask));

    // Crop the region of interest (ROI) based on the bounding rectangle
    cv::Mat maskedImage;
    croppedImage.copyTo(maskedImage, mask);

    // Apply Canny edge detector
    cv::Mat oEdgeImg;
    cv::Canny(maskedImage, oEdgeImg, 100, 300, 3);

    std::vector<std::vector<cv::Point>> oContours;
    cv::Mat oFilledImg;
    oEdgeImg.copyTo(oFilledImg);
    cv::findContours(oEdgeImg, oContours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < oContours.size(); i++)
    {
        cv::drawContours(oFilledImg, oContours, i, cv::Scalar(255), cv::FILLED);
    }

    // Count the number of non-zero pixels (edge pixels)
    int nFilledPixels = cv::countNonZero(oFilledImg);
    float fPixelRatio = static_cast<float>(nFilledPixels) / fArea;

    *a_pPixelRatio = fPixelRatio;

    return fPixelRatio > a_fThreshold;
}

double cvCheckerboardCalibration(BOARD_PATTERN a_eBoardPattern,
                                 int a_nBoardWidth,
                                 int a_nBoardHeight,
                                 float a_fSquareSize_mm,
                                 const std::string &a_rImageDirPath,
                                 cv::Mat &a_rK,
                                 cv::Mat &a_rDistCoeffs,
                                 int &a_rWidth,
                                 int &a_rHeight,
                                 CAM_MODEL a_eCamModel,
                                 const bool a_bDebug)
{
    spdlog::info("Checker board width = {}, height = {}, square size (mm) = {}", a_nBoardWidth, a_nBoardHeight, a_fSquareSize_mm);

    int nBoardWidth  = (a_eBoardPattern == BOARD_PATTERN::CHESSBOARD) ? a_nBoardWidth - 1 : a_nBoardWidth;
    int nBoardHeight = (a_eBoardPattern == BOARD_PATTERN::CHESSBOARD) ? a_nBoardHeight - 1 : a_nBoardHeight;

    std::vector<std::vector<cv::Point3f>> objPoints;
    std::vector<std::vector<cv::Point2f>> imgPoints;

    float fSquareSize_m = a_fSquareSize_mm / 1000.0;
    std::vector<cv::Point3f> objPoint;
    for (int i = nBoardHeight - 1; i >= 0; --i)
    {
        for (int j = 0; j < nBoardWidth; ++j)
            objPoint.emplace_back(j * fSquareSize_m, i * fSquareSize_m, 0.0);
    }

    std::vector<cv::String> vImagePath;
    vImagePath.reserve(50);
    std::regex oPattern(R"((.*\.(jpg|jpeg|png))$)", std::regex_constants::icase);// Case-insensitive match for .jpg, .jpeg, and .png
    for (const auto &entry: fs::directory_iterator(a_rImageDirPath))
    {
        if (fs::is_regular_file(entry.status()))
        {
            std::string strFileName = entry.path().filename().string();
            if (std::regex_match(strFileName, oPattern))
            {
                spdlog::info("Found image: {}", entry.path().string());
                vImagePath.push_back(entry.path().string());
            }
        }
    }

    spdlog::info("Finding checkerboard in images from directory {}", a_rImageDirPath);

    cv::Size oImageSize;
    std::vector<cv::Mat> oImages;
    oImages.reserve(vImagePath.size());
    for (int i = 0; i < vImagePath.size(); ++i)
    {
        spdlog::info("Processing image {}", vImagePath.at(i));
        auto oResult(FindCheckerboard(vImagePath.at(i), a_eBoardPattern, nBoardWidth, nBoardHeight, (a_eCamModel == CAM_MODEL::FISHEYE)));
        if (oResult)
        {
            auto &[cornerPts, oGray](*oResult);
            cv::Size oWinSize = {11, 11};
            bool bLargeWindow = (oGray.rows > 5000 || oGray.cols > 3000);

            oImageSize = oGray.size();

            cv::cornerSubPix(oGray, cornerPts, oWinSize, {-1, -1}, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

            if (a_bDebug)
            {
                fs::path oImgPath      = vImagePath.at(i);
                std::string strImgName = oImgPath.filename().string();
                // DEBUG
                cv::Mat oDebugChessBoardImg;
                cv::cvtColor(oGray, oDebugChessBoardImg, cv::COLOR_GRAY2BGR);
                cv::drawChessboardCorners(oDebugChessBoardImg, cv::Size(nBoardWidth, nBoardHeight), cornerPts, true);
                // cv::resize(oDebugChessBoardImg, oDebugChessBoardImg, {}, 0.5, 0.5);
                cv::imshow(strImgName, oDebugChessBoardImg);
                cv::waitKey(0);
                cv::destroyAllWindows();
            }
            objPoints.push_back(objPoint);
            imgPoints.push_back(cornerPts);

            oImages.push_back(cv::Mat());
            cv::cvtColor(oGray, oImages.back(), cv::COLOR_GRAY2BGR);
        }
        else
        {
            spdlog::info("No checkerboard found in image: {}", vImagePath.at(i));
        }
    }

    spdlog::info("Found checkerboard in {} images out of {}", objPoints.size(), vImagePath.size());

    a_rWidth  = oImageSize.width;
    a_rHeight = oImageSize.height;

    double dRmsReprojectionError;
    std::vector<cv::Mat> _rvecs, _tvecs;

    if (a_eCamModel == CAM_MODEL::PINHOLE)
    {
        spdlog::info("Pinhole camera model calibration");
        int nFlags = 0;
        nFlags |= cv::CALIB_FIX_ASPECT_RATIO;
        nFlags |= cv::CALIB_ZERO_TANGENT_DIST;
        nFlags |= cv::CALIB_FIX_PRINCIPAL_POINT;

        spdlog::info("Calibrating camera: This is slow! Please be patient");
        // dRmsReprojectionError is overall re-projection root mean square error in pixels
        // See: https://stackoverflow.com/questions/29628445/meaning-of-the-retval-return-value-in-cv2-calibratecamera
        // See: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d
        // From: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html - 'The closer the re-projection error is to zero, the more accurate the parameters we found are.'
        // Suggestion is that it should be less than 1
        dRmsReprojectionError = cv::calibrateCamera(objPoints,
                                                    imgPoints,
                                                    oImageSize,
                                                    a_rK,
                                                    a_rDistCoeffs,
                                                    _rvecs,
                                                    _tvecs,
                                                    nFlags);
    }
    else
    {
        spdlog::info("Fisheye camera model calibration");
        int nFlags = 0;
        nFlags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
        nFlags |= cv::fisheye::CALIB_FIX_PRINCIPAL_POINT;

        dRmsReprojectionError = cv::fisheye::calibrate(objPoints,
                                                       imgPoints,
                                                       oImageSize,
                                                       a_rK,
                                                       a_rDistCoeffs,
                                                       _rvecs,
                                                       _tvecs,
                                                       nFlags);

        // DEBUG
        if (0)
        {
            for (int i = 0; i < oImages.size(); ++i)
            {
                std::vector<cv::Point2f> _img_pts;
                cv::fisheye::projectPoints(objPoint, _img_pts, _rvecs[i], _tvecs[i], a_rK, a_rDistCoeffs);

                cv::Mat img = oImages.at(i).clone();

                double error = 0.0;
                for (int j = 0; j < _img_pts.size(); ++j)
                {
                    cv::circle(img, _img_pts[j], 3, cv::Scalar(0, 0, 255), 2);
                    double norm = cv::norm(imgPoints[i][j] - _img_pts[j]);
                    error += norm;
                }
                cv::imshow("fisheye projection image", img);
                cv::waitKey(0);
                cv::destroyAllWindows();

                spdlog::info("{} avg Error: {}", i, error / double(_img_pts.size()));
            }
        }

        // DEBUG
        if (1)
        {
            cv::Size dim     = oImageSize;
            cv::Mat scaled_K = a_rK.clone();
            cv::Mat new_K;
            cv::fisheye::estimateNewCameraMatrixForUndistortRectify(a_rK, a_rDistCoeffs, oImageSize, cv::Matx33d::eye(), new_K, 1.0);

            std::cout << "new_K: \n"
                      << new_K << std::endl
                      << std::endl;

            cv::Mat map1, map2;
            cv::fisheye::initUndistortRectifyMap(a_rK, a_rDistCoeffs, cv::Mat(), new_K, oImageSize, CV_16SC2, map1, map2);

            for (int i = 0; i < oImages.size(); ++i)
            {
                cv::Mat oUndistortedImg;
                cv::remap(oImages.at(i), oUndistortedImg, map1, map2, cv::INTER_LINEAR);
                cv::imshow("Original Image", oImages.at(i));
                cv::imshow("Undistorted Image", oUndistortedImg);
                cv::waitKey(0);
            }
        }
    }


    spdlog::info("Lens calibration complete with error {} pixels", dRmsReprojectionError);

    return dRmsReprojectionError;
}

double ChessBoardCalibration(int a_nBoardWidth,
                             int a_nBoardHeight,
                             float a_fSquareSize_mm,
                             const std::vector<std::vector<cv::Point2f>> &a_rImgPoints,
                             const cv::Size &a_rImgSize,
                             cv::Mat &a_rK,
                             cv::Mat &a_rDistCoeffs,
                             std::vector<cv::Mat> &rvecs,
                             std::vector<cv::Mat> &tvecs,
                             CAM_MODEL a_eCamModel,
                             const bool a_bDebug)
{
    std::vector<std::vector<cv::Point3f>> objPoints;

    float fSquareSize_m = a_fSquareSize_mm / 1000.0;
    std::vector<cv::Point3f> objPoint;
    for (int i = a_nBoardHeight - 1; i >= 0; --i)
    {
        for (int j = 0; j < a_nBoardWidth; ++j)
        {
            objPoint.emplace_back(j * fSquareSize_m, i * fSquareSize_m, 0.0);
        }
    }

    int nImages = a_rImgPoints.size();

    for (int i = 0; i < nImages; ++i)
        objPoints.push_back(objPoint);

    double dRmsReprojectionError;

    if (a_eCamModel == CAM_MODEL::PINHOLE)
    {
        if (a_bDebug)
        {
            spdlog::info("Pinhole camera model calibration");
            spdlog::info("Calibrating camera: This is slow! Please be patient");
        }
        int nFlags = 0;
        nFlags |= cv::CALIB_ZERO_TANGENT_DIST;
        nFlags |= cv::CALIB_FIX_K3;

        // dRmsReprojectionError is overall re-projection root mean square error in pixels
        // See: https://stackoverflow.com/questions/29628445/meaning-of-the-retval-return-value-in-cv2-calibratecamera
        // See: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d
        // From: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html - 'The closer the re-projection error is to zero, the more accurate the parameters we found are.'
        // Suggestion is that it should be less than 1
        dRmsReprojectionError = cv::calibrateCamera(objPoints,
                                                    a_rImgPoints,
                                                    a_rImgSize,
                                                    a_rK,
                                                    a_rDistCoeffs,
                                                    rvecs,
                                                    tvecs,
                                                    nFlags);
    }
    else
    {
        if (a_bDebug)
            spdlog::info("Fisheye camera model calibration");
        int nFlags = 0;
        nFlags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
        nFlags |= cv::fisheye::CALIB_FIX_PRINCIPAL_POINT;
        nFlags |= cv::fisheye::CALIB_FIX_SKEW;

        dRmsReprojectionError = cv::fisheye::calibrate(objPoints,
                                                       a_rImgPoints,
                                                       a_rImgSize,
                                                       a_rK,
                                                       a_rDistCoeffs,
                                                       rvecs,
                                                       tvecs,
                                                       nFlags);
    }

    return dRmsReprojectionError;
}
