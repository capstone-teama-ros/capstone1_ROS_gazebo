#ifndef DATA_INTEGRATE_ROBOT_DRAWING_H
#define DATA_INTEGRATE_ROBOT_DRAWING_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/viz/types.hpp>

/**
 * LIDAR로 탐지한 점, OpenCV로 탐지한 공과 같은 지형지물을 2D 이미지로 보여주는 클래스입니다.
 * 모든 좌표는 로봇에 대한 상대적 위치를 사용하며, x축 방향이 이미지의 오른쪽, y축 방향이 이미지의 위쪽입니다.
 */
class RobotDrawing
{
public:
  /**
   * @param width   이미지의 너비 (x축) (pixels)
   * @param height  이미지의 높이 (y축) (pixels)
   * @param pixels_per_meter  이미지의 해상도. 실제 길이 1미터당 나타낼 픽셀의 갯수입니다.
   */
  RobotDrawing(int width, int height, double pixels_per_meter)
    : image_(height, width, CV_8UC3), pixels_per_meter_(pixels_per_meter), meters_per_pixel_(1 / pixels_per_meter)
  {
  }

  /**
   * 선분을 그립니다. 자세한 것은 @c cv::line() 의 설명을 참조하세요:
   * https://docs.opencv.org/3.2.0/d6/d6e/group__imgproc__draw.html#ga7078a9fae8c7e7d13d24dac2520ae4a2
   *
   * @param pt1       선분의 시작점. 길이 단위는 미터입니다.
   * @param pt2       선분의 끝점. 길이 단위는 미터입니다.
   * @param color     선분의 색깔
   * @param thickness 선분의 두께 (pixels)
   * @param lineType  선분의 종류
   */
  void line(cv::Point2d pt1, cv::Point2d pt2, const cv::Scalar& color, int thickness = 1, int lineType = cv::LINE_8);

  /**
   * 원을 그립니다. 자세한 것은 @c cv::circle() 의 설명을 참조하세요:
   * https://docs.opencv.org/3.2.0/d6/d6e/group__imgproc__draw.html#gaf10604b069374903dbd0f0488cb43670
   *
   * @param center    원의 중심. 길이 단위는 미터입니다.
   * @param radius    원의 반지름 (pixels)
   * @param color     원의 색깔
   * @param thickness 원의 경계선의 두께 (pixels)
   * @param lineType  선의 종류
   */
  void circle(cv::Point2d center, int radius, const cv::Scalar& color, int thickness = 1, int lineType = cv::LINE_8);

  /**
   * 마커를 그립니다. 자세한 것은 @c cv::drawMarker() 의 설명을 참조하세요:
   * https://docs.opencv.org/3.2.0/d6/d6e/group__imgproc__draw.html#ga482fa7b0f578fcdd8a174904592a6250
   *
   * @param position    마커의 위치. 길이 단위는 미터입니다.
   * @param color       마커의 색깔
   * @param markerType  마커의 종류. 자세한 것은 @c cv::MarkerTypes 의 설명을 참조하세요:
   * https://docs.opencv.org/3.2.0/d6/d6e/group__imgproc__draw.html#ga0ad87faebef1039ec957737ecc633b7b
   * @param markerSize  마커의 크기 (pixels)
   * @param thickness   선의 두께 (pixels)
   * @param lineType    선분의 종류
   */
  void drawMarker(cv::Point2d position, const cv::Scalar& color, int markerType = cv::MARKER_CROSS, int markerSize = 20,
                  int thickness = 1, int line_type = cv::LINE_8);

  /**
   * 직사각형을 그립니다. 자세한 것은 @c cv::rectangle() 의 설명을 참조하세요:
   *
   * @param pt1       직사각형의 첫 꼭짓점. 길이 단위는 미터입니다.
   * @param pt2       직사각형의 첫 꼭짓점과 대각선 맞은편에 있는 꼭짓점. 길이 단위는 미터입니다.
   * @param color     직사각형의 색깔
   * @param thickness 원의 경계선의 두께 (pixels)
   * @param lineType  선분의 종류
   */
  void rectangle(cv::Point2d pt1, cv::Point2d pt2, const cv::Scalar& color, int thickness = 1,
                 int lineType = cv::LINE_8);

  /**
   * 이미지의 모든 픽셀을 같은 색으로 통일합니다.
   *
   * @param color 색깔(BGR). 생략하면 검정색으로 통일합니다.
   */
  void clear(const cv::Scalar& color = cv::viz::Color::black())
  {
    image_.setTo(color);
  }

  /**
   * @c cv::imshow() 와 @c cv::waitKey() 를 순서대로 호출합니다.
   *
   * @param winname 띄울 창의 제목
   * @param delay   화면을 띄워놓는 시간 (milliseconds). 0을 입력하면 무한히 지속됩니다.
   * @returns @c cv::waitKey() 의 리턴값에 0xFF
   */
  int drawAndWaitKey(const cv::String& winname, int delay = 0) const;

  /// @returns 이미지의 너비 (pixels)
  int getWidth() const
  {
    return image_.cols;
  }

  /// @returns 이미지의 높이 (pixels)
  int getHeight() const
  {
    return image_.rows;
  }

  /// @returns 실제 좌표계의 원점이 이미지 상에서 갖는 x좌표 (pixels)
  int getOriginX() const
  {
    return (getWidth() + 1) / 2;
  }

  /// @returns 실제 좌표계의 원점의 이미지 상에서 갖는 y좌표 (pixels)
  int getOriginY() const
  {
    return (getHeight() + 1) / 2;
  }

  /// @returns 미터 길이를 픽셀로 변환한 값
  int toPixels(double meters) const
  {
    return cvRound(meters * pixels_per_meter_);
  }

  /// @returns 픽셀 길이를 미터로 변환한 값
  double toMeters(int pixels) const
  {
    return pixels * meters_per_pixel_;
  }

  /**
   * 로봇을 기준으로 한 물리적 좌표를 이미지 상의 점의 좌표로 변환합니다.
   *
   * @param point 로봇을 기준으로 한 물리적 좌표. 거리 단위는 미터입니다.
   * @returns 이미지 상의 점의 좌표. 거리 단위는 픽셀입니다.
   */
  cv::Point fromRobotFrame(const cv::Point2d pt) const
  {
    return { toPixels(pt.x) + getOriginX(), -toPixels(pt.y) + getOriginY() };
  }

private:
  /// 주의: OpenCV의 행렬은 row = y, column = x이다.
  cv::Mat image_;
  double pixels_per_meter_ = 0;
  double meters_per_pixel_ = 0;
};

#endif  // DATA_INTEGRATE_ROBOT_DRAWING_H
