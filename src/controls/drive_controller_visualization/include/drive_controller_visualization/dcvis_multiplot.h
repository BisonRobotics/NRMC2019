#include <opencv2/opencv.hpp>

class dcvis_multiplot
{
public:
    dcvis_multiplot(int num_series, cv::String a_title);
    //void set_xwidth(double x_width);
    //void set_sample_period(double period);
    //void set_ylim(double ymin, double ymax);
    void add_point(double y,double t, int series);
    void draw(cv::Mat frame);
private:
    static const int num_samples = 1000;
    std::vector<std::array<cv::Point2d, num_samples> > series_list;
    //std::vector<cv::String> names;
    double sample_period;
    double xwidth;
    double ymin;
    double ymax;
    double xmax;
    double xtick_period;
    int yticks;
    std::deque<double> xtick_deque;
    cv::String title;
};