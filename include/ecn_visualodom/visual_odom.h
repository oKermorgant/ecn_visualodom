#ifndef VISUALODOM_H
#define VISUALODOM_H

#include <visp/vpPoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <vector>
#include <algorithm>
#include <visp/vpSubColVector.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <visp/vpHomography.h>
#include <time.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpSubMatrix.h>


// utility structure
struct Homography
{
  vpRotationMatrix R;
  vpTranslationVector t, n;

  void buildFrom(cv::Mat1d _R, cv::Mat1d _t, cv::Mat1d _n)
  {
    for(unsigned int i=0;i<3;++i)
    {
      t[i] = _t(i,0);
      n[i] = _n(i,0);
      for(unsigned int j=0;j<3;++j)
        R[i][j] = _R(i,j);
    }
  }

  vpMatrix H()
  {
    return ((vpMatrix) R) + t*n.t();
  }

  void print()
  {
    vpThetaUVector tu(R);
    std::cout << "R: " << tu.t()*180/CV_PI << std::endl;
    std::cout << "t: "  << t.t() << std::endl;
    std::cout << "n: " << n.t() << std::endl;
  }
};





class VisualOdom
{
public:
  // constructor from given camera parameters
  VisualOdom(const vpCameraParameters cam, bool _relative_to_initial = false) :matcher(cv::NORM_HAMMING)
  {
    // init calibration matrices from camera parameters
    Kcv = (cv::Mat1d(3, 3) <<
           cam.get_px(), 0, cam.get_u0(),
           0, cam.get_py(), cam.get_v0(),
           0, 0, 1);
    K = cam.get_K();
    Ki = cam.get_K_inverse();

    // no image yet
    first_time = true;
    relative_to_initial = _relative_to_initial;

    // matching
    akaze = cv::AKAZE::create();

    // default guesses
    n_guess.resize(3);
    n_guess[2] = 1;
  }

  // first guess for normal
  inline void setNormalGuess(double x, double y, double z)
  {
    n_guess[0] = x;
    n_guess[1] = y;
    n_guess[2] = z;
    n_guess /= n_guess.euclideanNorm();
  }

  inline void setInitialTranslation(vpTranslationVector t)
  {
    t0 = t;
  }


  // process a new image and writes relative transform
  bool process(cv::Mat &im2, vpHomogeneousMatrix &_M);

  // finds a 3D rotation that changes the normal to Z axis
  void getRotationFromNormal(vpRotationMatrix &_R) const
  {
    vpColVector z(3);z[2]=1;
    vpColVector ax = vpColVector::crossProd(z, n_guess);
    const double s = ax.euclideanNorm();
    const double a = atan2(s, z.t()*n_guess)/s;
    _R.buildFrom(a*ax[0], a*ax[1], a*ax[2]);
  }

protected:
  // calibration (OpenCV and ViSP formats)
  cv::Mat1d Kcv;
  vpMatrix K, Ki;

  // first time or not
  bool first_time;
  bool relative_to_initial;

  // matching stuff
  cv::Mat img,im1,imatches;
  cv::Ptr<cv::AKAZE> akaze;
  std::vector<cv::KeyPoint> kp1, kp2;
  cv::Mat des1, des2, Hp, mask;
  cv::BFMatcher matcher;
  std::vector<cv::DMatch> matches;

  // homographies
  double d_guess = 0;
  vpColVector n_guess, t0;
  std::vector<Homography> H;
  std::vector<cv::Mat> R, t, nor;

  // utility function, convert cv::Points to a (3 x nb_pts) matrix of normalized coordinates
  vpMatrix cvPointToNormalized(const std::vector<cv::Point2f> &p) const
  {
    vpMatrix X(3,p.size());
    // build homogeneous pixel coordinates
    for(unsigned int i=0;i<p.size();++i)
    {
      X[0][i] = p[i].x;
      X[1][i] = p[i].y;
      X[2][i] = 1;
    }
    // to normalized coordinates
    return Ki*X;
  }

  uint removeOutliers(std::vector<cv::Point2f> &matched1, std::vector<cv::Point2f> &matched2)
  {
    const auto firstInvalid = [&](std::vector<cv::Point2f> &matched)
    {
      return std::remove_if(matched.begin(), matched.end(),
                            [&](const cv::Point2f &p)
      {
        return !mask.at<bool>(&p-matched.data());
      });
    };

    matched1.erase(firstInvalid(matched1), matched1.end());
    matched2.erase(firstInvalid(matched2), matched2.end());
    return static_cast<uint>(matched1.size());
  }
};

#endif // VISUALODOM_H
