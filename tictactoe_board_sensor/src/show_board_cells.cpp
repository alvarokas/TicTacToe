#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QFile>
#include <QXmlStreamWriter>
#include <QFileDialog>
#include <QApplication>

namespace enc = sensor_msgs::image_encodings;

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

    std::string board_file;

//    t_Board board; // A vector of cells representing the board game



    // thanks to http://bytefish.de/blog/extracting_contours_with_opencv/
    void cropping_cells(cv_bridge::CvImageConstPtr& cv_cp_img)
    {
//        cv::Mat mask = cv::Mat::zeros(cv_cp_img->image.rows, cv_cp_img->image.cols, CV_8UC1);
//        cv::drawContours(mask, this->board, -1, cv::Scalar(255), CV_FILLED);    // CV_FILLED fills the connected components found with white (white RGB value = 255,255,255)

//        cv::Mat im_crop(cv_cp_img->image.rows, cv_cp_img->image.cols, CV_8UC3); // let's create a new 8-bit 3-channel image with the same dimensions
//        im_crop.setTo(cv::Scalar(0));                                           // we fill the new image with a color, in this case we set background to black.
//        cv_cp_img->image.copyTo(im_crop, mask);                                 // copy just the elements from the original image indicated by the non-zero elements from mask to crop
//        cv::normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);      // normalize so imwrite(...)/imshow(...) shows the mask correctly!

//        // show the images
//        cv::imshow("original", cv_cp_img->image);
//        cv::imshow("mask", mask);
//        cv::imshow("cropped", im_crop);
    }

    void read_cells_from_file()
    {
//        if (!this->board.empty())
//        {
//            QApplication app(0,0);
//            QString fileName = QFileDialog::getSaveFileName(0, "Save File", QDir::currentPath(), "XML files (*.xml)", new QString("XML files (*.xml)"));
//            ROS_DEBUG_STREAM("File Name selected= " << fileName.toStdString());
//            if(!fileName.isEmpty())
//            {
//                QFile output(fileName);
//                output.open(QIODevice::WriteOnly);

//                QXmlStreamWriter stream(&output);
//                stream.setAutoFormatting(true);
//                stream.writeStartDocument();

//                stream.writeStartElement("board");

//                for (size_t i=0; i< this->board.size();++i)
//                {
//                    stream.writeStartElement("cell");
//                    stream.writeAttribute("id", QString::number(i));
//                    for (t_Cell::const_iterator it_vertex = this->board[i].begin(); it_vertex != this->board[i].end(); ++it_vertex) {
//                        stream.writeEmptyElement("vertex");
//                        stream.writeAttribute("x", QString::number(it_vertex->x));
//                        stream.writeAttribute("y", QString::number(it_vertex->y));
//                    }
//                    stream.writeEndElement(); // cell
//                }

//                stream.writeEndElement(); // board
//                stream.writeEndDocument();
//                output.close();
//            }
//        }
//        else ROS_INFO("No cells in the board to be saved.");
    }

public:
    CellDisplay()
        : it_(nh_),POINT_RADIUS(5)
    {
        image_sub_ = it_.subscribe("image_in", 1, &CellDisplay::imageCb, this);

        cv::namedWindow(CellDisplay::WINDOW);

        if (ros::param::get("/board_file", this->board_file))
        {
            ROS_INFO("board file loaded");
        }
    }

    ~CellDisplay()
    {
        cv::destroyWindow(CellDisplay::WINDOW);
    }

    /* mouse event handler function */
    static void onMouse( int event, int x, int y, int, void* param)
    {
        if( event != cv::EVENT_LBUTTONDBLCLK )
            return;

        cv::Point p = cv::Point(x,y);
        CellDisplay * img_conv = (CellDisplay*) param;

//        // If the point is inside an already defined cell, the cell is removed
//        if(img_conv->remove_cell(p)) {
//            ROS_INFO_STREAM("Cell deleted. Remaining " << img_conv->board.size() << " cells");
//            return;
//        }

//        // If the point correspond to a vertex, the vertex is removed
//        if(img_conv->remove_point(p)) {
//            ROS_INFO_STREAM("Point deleted. Remaining " << img_conv->points.size() << " points");
//        }
//        // If the point is new, it is added to the current cell
//        else {
//            img_conv->points.push_back(p);
//            ROS_INFO_STREAM("Point added: " << p.x << " , " << p.y << ". Total points in this cell: "  << img_conv->points.size() );
//        }

        return;
    }


    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
//        //converting ROS image format to opencv image format
//        cv_bridge::CvImageConstPtr cv_ptr;
//        try
//        {
//            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
//        }
//        catch (cv_bridge::Exception& e)
//        {
//            ROS_ERROR("cv_bridge exception: %s", e.what());
//            return;
//        }

//        cv::Mat img_aux = cv_ptr->image.clone();

//        // how-to on screen
//        this->show_how_to(img_aux);

//        // drawing all points of the current cell
//        for (t_Cell::iterator it_drawing = this->points.begin();it_drawing != this->points.end();++it_drawing) {
//            cv::circle(img_aux,*it_drawing,CellDisplay::POINT_RADIUS,cv::Scalar(0,0,255),-1);
//        }

//        // drawing all cells of the board game
//        for (t_Board::iterator it_cell = this->board.begin(); it_cell != this->board.end(); ++it_cell) {
//            cv::fillConvexPoly(img_aux,it_cell->data(),it_cell->size(), cv::Scalar(0,0, 255));
//        }
//        cv::drawContours(img_aux,this->board,-1, cv::Scalar(123,125,0),2); // drawing the borders in a different color

//        cv::setMouseCallback(CellDisplay::WINDOW, onMouse, this);
//        cv::imshow(CellDisplay::WINDOW, img_aux);


//        int c = cv::waitKey(3);
//        if( (c & 255) == 27 ) // ESC key pressed
//        {
//            ROS_INFO_STREAM("Exiting with ESC key ..." << this->board.size() << " cells selected");
//            ros::shutdown();
//        }
//        else if ((char)c == ' ') {  // SPACE bar pressed
//            if(!this->points.empty())
//            {
//                this->board.push_back(this->points);
//                this->points.clear();
//                ROS_INFO("Add points to the new cell");
//            }
//            else ROS_INFO_STREAM("The current cell is empty. Add points to it before you create a new one");
//        }
//        else if ((char)c=='r') { // 'R' key pressed
//            this->cropping_cells(cv_ptr);
//        }
//        else if ((char)c=='s') { // 'S' key pressed
//            this->save_cells_to_file();
//        }

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
