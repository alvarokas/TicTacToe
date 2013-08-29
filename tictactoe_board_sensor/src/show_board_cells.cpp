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

#include <boost/lexical_cast.hpp>

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


    cv::Point centroid_of_cell(t_Cell& cell)
    {
        uint sumX = 0, sumY = 0;
        size_t size = cell.size();
        cv::Point centroid(0,0);
        if(size > 0){
            for (t_Cell::const_iterator it_point = cell.begin(); it_point != cell.end(); ++it_point) {
                sumX += it_point->x;
                sumY += it_point->y;
            }
            // TODO through exception if size <= 0
            centroid.x = sumX/size;
            centroid.y = sumY/size;
        }
        return centroid;
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
        for(size_t i=0;i!=this->board.size();++i)
        {
            cv::Point p = this->centroid_of_cell(this->board[i]);
            //cv::circle(img_aux, p,5,cv::Scalar(0,0, 255),-1);
            cv::putText(img_aux, boost::lexical_cast<std::string>(i+1), p, cv::FONT_HERSHEY_DUPLEX, 1,cv::Scalar(255,255,0));
        }

        cv::imshow(CellDisplay::WINDOW, img_aux);

        int c = cv::waitKey(3);
        if( (c & 255) == 27 ) // ESC key pressed
        {
            ROS_INFO_STREAM("Exiting with ESC key ..." << this->board.size() << " cells selected");
            ros::shutdown();
        }
        else if ((char)c =='s')
        {
            this->sensing_cells(cv_ptr->image);
        }
    }

    void sensing_cells(const cv::Mat& img)
    {
        ROS_DEBUG("@sensing_cells");
        short int counter=0;
        foreach (t_Cell cell, this->board) {
            cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
            cv::drawContours(mask, t_Board(1,cell), -1, cv::Scalar(255), CV_FILLED);  // CV_FILLED fills the connected components found with white (white RGB value = 255,255,255)

            cv::Mat im_crop(img.rows, img.cols, CV_8UC3);                           // let's create a new 8-bit 3-channel image with the same dimensions
            im_crop.setTo(cv::Scalar(0));                                           // we fill the new image with a color, in this case we set background to black.
            img.copyTo(im_crop, mask);                                              // copy just the elements from the original image indicated by the non-zero elements from mask to crop
            cv::normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);      // normalize so imwrite(...)/imshow(...) shows the mask correctly!

            // show the images
            /*cv::imshow("original", img);
            cv::imshow("mask", mask);*/
            std::string win_name="cell";
            win_name+=boost::lexical_cast<std::string>(++counter);
            cv::imshow(win_name, im_crop);
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
