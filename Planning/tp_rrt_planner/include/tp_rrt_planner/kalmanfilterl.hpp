#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;
class KalmanFilterL {
public:
    KalmanFilterL() {
        KF = KalmanFilter(4, 2, 0);
        state = Mat_<float>(4, 1);
        processNoise = Mat(4, 1, CV_32F);
        measurement= Mat_<float>(2,1); 
        measurement.setTo(Scalar(0));
        KF.statePre.at<float>(0) = 0.0;
        KF.statePre.at<float>(1) = 0.0;
        KF.statePre.at<float>(2) = 0.0;
        KF.statePre.at<float>(3) = 0.0;
        KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1, 0,0,1,0,  0,0,0,1);

        setIdentity(KF.measurementMatrix);

        setIdentity(KF.processNoiseCov, Scalar::all(1e-4));

        setIdentity(KF.measurementNoiseCov, Scalar::all(1));

        setIdentity(KF.errorCovPost, Scalar::all(.1));

    }
    std::pair<float, float> update(const float& x, const float& y){
        Mat prediction = KF.predict();
        Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

        measurement(0) = x;
        measurement(1) = y;

        Mat estimated = KF.correct(measurement);
        return std::make_pair(estimated.at<float>(0), estimated.at<float>(1));
    }
private:
    KalmanFilter KF;
    Mat_<float> state;
    Mat processNoise;
    Mat_<float> measurement; 
};