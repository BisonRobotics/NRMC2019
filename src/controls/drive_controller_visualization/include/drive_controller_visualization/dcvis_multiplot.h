#include <opencv2/opencv.hpp>

class dcvis_multiplot
{
public:
    dcvis_multiplot(int num_series, cv::String title,
                    double ymin, double ymax, double yticks, 
                    double xwidth, double xtick_period,
                    cv::String* names,
                    cv::Scalar* colors);
    //void set_xwidth(double x_width);
    //void set_ylim(double ymin, double ymax);
    void add_point(double y,double t, int series);
    void draw(cv::Mat frame);
private:
    static const int num_samples = 1000;
    std::vector<std::array<cv::Point2d, num_samples> > series_list;
    cv::String* names;
    cv::Scalar* colors;
    int num_series;
    double xwidth;
    double ymin;
    double ymax;
    int yticks;
    double xmax;
    double xtick_period;
    std::deque<double> xtick_deque;
    cv::String title;
    
    
};