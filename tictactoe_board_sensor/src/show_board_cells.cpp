#include <ros/ros.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QFile>
#include <QXmlStreamReader>

#include "src/board_cells_delimitation.h"

namespace ttt
{


class CellDisplay
{
private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    const short POINT_RADIUS;

    static const char WINDOW[];

    std::string board_config;

    t_Board board; // A vector of cells representing the board game


    void read_cells_from_file()
    {
        ROS_DEBUG("Ready to read xml data");

        this->board.clear(); //cleaning all previous cells

        QXmlStreamReader xml;
        xml.addData(QString::fromStdString(this->board_config)); //adding the xml content to the input stream        
        while(!xml.atEnd() && !xml.hasError()) {
            /* Read next element.*/
            QXmlStreamReader::TokenType token = xml.readNext();
            ROS_DEBUG_STREAM("name=" << xml.name().toString().toStdString());
            /* If token is just StartDocument, we'll go to next.*/
            if(token == QXmlStreamReader::StartDocument) {
                continue;
            }
            /* If token is StartElement, we'll see if we can read it.*/
            if(token == QXmlStreamReader::StartElement) {
                /* If it's named cell, we'll add a new cell to the board.*/                
                if(xml.name() == "cell") {
                    this->board.push_back(t_Cell());
                    continue;
                }
                /* If it's named vertex, we'll dig the information from there.*/
                if(xml.name() == "vertex") {
                    /* Let's get the attributes for vertex */
                    QXmlStreamAttributes attributes = xml.attributes();
                    /* Let's check that vertex has x and y attribute. */
                    if(attributes.hasAttribute("x") && attributes.hasAttribute("y")) {
                        /* We'll add it to the cell */
                        this->board.back().push_back(cv::Point(attributes.value("x").toString().toInt(), attributes.value("y").toString().toInt()));
                    }
                    else xml.raiseError("Vertex corrupted: x and/or y value is missing");
                }
            }
        }
        ROS_ASSERT_MSG(!xml.hasError(),"Error parsing xml file (l.%d, c.%d):%s", (int)xml.lineNumber(), (int)xml.columnNumber(), xml.errorString().toStdString().c_str());

        ROS_INFO("Xml configuration board loaded. %d cells loaded", (int)this->board.size());
    }

public:
    CellDisplay()
        : it_(nh_),POINT_RADIUS(5)
    {
        image_sub_ = it_.subscribe("image_in", 1, &CellDisplay::imageCb, this);

        cv::namedWindow(CellDisplay::WINDOW);

        if (ros::param::get("/board_file", this->board_config))
        {
            ROS_INFO("config xml board file successfully loaded");
        }
        else
        {
            ROS_FATAL_STREAM("config xml board file not found!");
            ROS_BREAK();
        }

        this->read_cells_from_file();
    }

    ~CellDisplay()
    {
        cv::destroyWindow(CellDisplay::WINDOW);
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

        cv::Mat img_aux = cv_ptr->image.clone();

        // drawing all cells of the board game
        cv::drawContours(img_aux,this->board,-1, cv::Scalar(123,125,0),2); // drawing just the borders

        cv::imshow(CellDisplay::WINDOW, img_aux);


        int c = cv::waitKey(3);
        if( (c & 255) == 27 ) // ESC key pressed
        {
            ROS_INFO_STREAM("Exiting with ESC key ..." << this->board.size() << " cells selected");
            ros::shutdown();
        }
    }
};

const char CellDisplay::WINDOW[] = "Cell display";

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_cells");
    ttt::CellDisplay cd;
    ros::spin();
    return 0;
}
