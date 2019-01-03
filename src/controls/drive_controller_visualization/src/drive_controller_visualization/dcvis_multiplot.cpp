#include <drive_controller_visualization/dcvis_multiplot.h>

dcvis_multiplot::dcvis_multiplot(int num_series, cv::String title)
{
    std::array<cv::Point2d, dcvis_multiplot::num_samples> dummy;
    for (int index=0; index < dcvis_multiplot::num_samples; index++)
    {
        series_list.push_back(dummy);
    }
    sample_period = .1;
    xwidth = 30;
    ymin = -5;
    ymax = 5;
}

void dcvis_multiplot::add_point(double y, int series)
{
    cv::Point2d* my_p = series_list.at(series).data();
    memmove(my_p, my_p+1, dcvis_multiplot::num_samples -1);
    //cv::Point2d temp_point(my_p[dcvis_multiplot::num_samples-1].x + sample_period, y);
    my_p[dcvis_multiplot::num_samples-1].x += sample_period;
    my_p[dcvis_multiplot::num_samples-1].y = y;
    xmax = my_p[dcvis_multiplot::num_samples-1].x;
}

void dcvis_multiplot::draw(cv::Mat frame)
{
    //clear frame
    frame.setTo(cv::Scalar(0,50,50));
    //need ymax to be at row 0, ymin to be at last row
    //need first sample at first col, last at last
    double width = frame.cols;
    double height = frame.rows;
    double yscale = height/(ymax - ymin); 
    double xscale = width/xwidth;
    double xmin = xmax - xwidth;
    //(data.y - ymin)*yscale is plot row
    //(data.x - xmin)*xscale is plot col
    int start_idex = dcvis_multiplot::num_samples - xwidth/sample_period;
    //transform data
    //plot
    cv::Point p1, p2;
    p1.x = (series_list.at(0)[start_idex].x - xmin)*xscale;
    p1.y = (series_list.at(0)[start_idex].y - ymin)*yscale;
    for (int idex = start_idex+1; idex < dcvis_multiplot::num_samples; idex++)
    {
        p2.x = (series_list.at(0)[idex].x - xmin)*xscale;
        p2.y = (series_list.at(0)[idex].y - ymin)*yscale;
        cv::line(frame, p1, p2, cv::Scalar(200,200,200), 1, 8, 0);
        p1 = p2;
    }
    //other stuff
}