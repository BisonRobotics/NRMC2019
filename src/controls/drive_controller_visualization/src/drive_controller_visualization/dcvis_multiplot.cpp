#include <drive_controller_visualization/dcvis_multiplot.h>

dcvis_multiplot::dcvis_multiplot(int num_series, cv::String title,
                                 double ymin, double ymax, double yticks, 
                                 double xwidth, double xtick_period,
                                 cv::String* names,
                                 cv::Scalar* colors)
:num_series(num_series), title(title),
 ymin(ymin), ymax(ymax), yticks(yticks),
 xwidth(xwidth), xtick_period(xtick_period),
 names(names), colors(colors)
{
    std::array<cv::Point2d, dcvis_multiplot::num_samples> dummy;
    for (int index=0; index < dcvis_multiplot::num_samples; index++)
    {
        series_list.push_back(dummy);
    }
    xtick_deque.push_back(0);
}

void dcvis_multiplot::add_point(double y, double t, int series)
{
    cv::Point2d* my_p = series_list.at(series).data();
    memmove(my_p, my_p+1, (dcvis_multiplot::num_samples -1)*sizeof(cv::Point2d));
    my_p[dcvis_multiplot::num_samples-1].x = t;
    my_p[dcvis_multiplot::num_samples-1].y = y;
    xmax = t;//my_p[dcvis_multiplot::num_samples-1].x;
}

void dcvis_multiplot::draw(cv::Mat frame)
{
    //clear frame
    frame.setTo(cv::Scalar(0,50,50));
    //draw title
    int dummy;
    double font_scale = .5;
    cv::Scalar font_color = cv::Scalar(150,150,150);
    cv::Size title_size = cv::getTextSize(title, CV_FONT_HERSHEY_SIMPLEX, font_scale, 1, &dummy);
    cv::Point title_point((frame.cols - title_size.width)/2, (title_size.height));
    cv::putText(frame, title, title_point, CV_FONT_HERSHEY_SIMPLEX, font_scale, font_color);
    //draw yticks, largest one has top left corner at bottom height of title, smallest one is at bottom of frame
    //so thats (frame.rows - 2*title_size.height)/yticks spacing
    cv::Size number_size = cv::getTextSize("-2.50", CV_FONT_HERSHEY_SIMPLEX, font_scale, 1, &dummy);
    for (int index = 0; index < yticks; index++)
    {
        double scale_fact = ((double)(index)) / ((double)(yticks - 1.0));
        char data[10];
        sprintf(data, "% .2f", (ymax - ymin) * (1.0-scale_fact) + ymin);
        cv::Size this_size = cv::getTextSize(data, CV_FONT_HERSHEY_SIMPLEX, font_scale, 1, &dummy);
        title_point.x = number_size.width - this_size.width;
        title_point.y = (frame.rows - title_size.height - number_size.height) * scale_fact + title_size.height + number_size.height;
        cv::putText(frame, data, title_point, CV_FONT_HERSHEY_SIMPLEX, font_scale, font_color);
    }
    //draw xticks
    //keep track of which ones are on the screen
    if (xmax > xtick_deque.back() + xtick_period)
    {
        xtick_deque.push_back(xtick_deque.back() + xtick_period);
    }
    if (xtick_deque.front() < xmax - xwidth)
    {
        xtick_deque.pop_front();
    }
    //draw all the ones in the deque
    title_point.y = frame.rows;
    double width = frame.cols - number_size.width;
    double xscale = width/xwidth;
    for (const auto &tick : xtick_deque)
    {
        char data[10];
        sprintf(data, "%.0f", tick);
        title_point.x = width - (xmax - tick) * xscale + number_size.width;
        cv::putText(frame, data, title_point, CV_FONT_HERSHEY_SIMPLEX, font_scale, font_color);
    }
    //draw side, top and bottom lines
    cv::line(frame, cv::Point(number_size.width, number_size.height), 
                    cv::Point(number_size.width, frame.rows - title_size.height), cv::Scalar(200,200,200));
    cv::line(frame, cv::Point(number_size.width, number_size.height), 
                    cv::Point(frame.cols, number_size.height), cv::Scalar(200,200,200));
    cv::line(frame, cv::Point(number_size.width, frame.rows - title_size.height), 
                    cv::Point(frame.cols, frame.rows - title_size.height), cv::Scalar(200,200,200));
    //transform data and plot
    double height = frame.rows - title_size.height - number_size.height;
    double yscale = height/(ymax - ymin); 
    double xmin = xmax - xwidth;

    int start_idex=0;
    //find starting index
    for (start_idex = 0; start_idex <dcvis_multiplot::num_samples && series_list.at(0)[start_idex].x < xmin; start_idex++);
    
    for (int odex = 0; odex < num_series; odex++)
    {
        cv::Point p1, p2;
        p1.x = (series_list.at(odex)[start_idex].x - xmin)*xscale + number_size.width;
        p1.y = height - (series_list.at(odex)[start_idex].y - ymin)*yscale + title_size.height;
        for (int idex = start_idex+1; idex < dcvis_multiplot::num_samples; idex++)
        {
            p2.x = (series_list.at(odex)[idex].x - xmin)*xscale + number_size.width;
            p2.y = height - (series_list.at(odex)[idex].y - ymin)*yscale + title_size.height;
            cv::line(frame, p1, p2, colors[odex], 1, 8, 0);
            p1 = p2;
        }
    }

    //other stuff
}