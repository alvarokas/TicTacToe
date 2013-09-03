#ifndef TTT_DEFINITIONS_H
#define TTT_DEFINITIONS_H

namespace enc = sensor_msgs::image_encodings;

namespace ttt
{

typedef std::vector<cv::Point> t_Cell;    // vector of points delimiting a cell
typedef std::vector<t_Cell> t_Board;   // vector of cells, i.e. a vector of vectors of points

}

#endif // TTT_DEFINITIONS_H
