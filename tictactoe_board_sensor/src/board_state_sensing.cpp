
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

//#include <QFile>
//#include <QXmlStreamReader>

#include <boost/lexical_cast.hpp>

#include "ttt_definitions.h"
#include "ttt_cells.h"

namespace ttt
{


class BoardState
{
private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    std::string board_config;

    t_Board board; // A vector of cells representing the board game


public:
    BoardState()
        : it_(nh_)
    {
        image_sub_ = it_.subscribe("image_in", 1, &BoardState::imageCb, this);

        /* Reading cells definition data from the parameter server */
        if(!Cells::read_from_parameter_server(this->board,Cells::CELLS_DATA_PARAM_NAME))
        {
            ROS_FATAL_STREAM("No cell data to display!");
            ROS_BREAK();
        }

    }

    ~BoardState()
    {
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        //converting ROS image format to opencv image format
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        foreach (t_Cell cell, this->board)
        {
            cv::Mat cropped_cell = Cells::masked_cell_image(cv_ptr->image,cell);
        }
    }
};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_cells");
    ttt::BoardState cd;
    ros::spin();
    return 0;
}
