#include "ttt_cells.h"


namespace ttt {

const std::string Cells::CELLS_DATA_PARAM_NAME = "/board_file";

/**
 Writes the contours of all the cells in a board to a file.
 The name of the file is asked to the user by means of a dialog window.
 The format of the file is as follows:
        <board>
            <cell id="0">
                <vertex x="195" y="50"/>
                [...]
            </cell>
            [...]
        </board>
 @param board where all the contours are stored
 @return true if cell data have been successfully saved, false otherwise.
 */
bool Cells::save_to_file(const t_Board& board)
{

    if (!board.empty())
    {
        QApplication app(0,0);
        QString fileName = QFileDialog::getSaveFileName(0, "Save File", QDir::currentPath(), "XML files (*.xml)", new QString("XML files (*.xml)"));
        ROS_DEBUG_STREAM("File Name selected= " << fileName.toStdString());
        if(!fileName.isEmpty())
        {
            QFile output(fileName);
            if(!output.open(QIODevice::WriteOnly))
            {
                ROS_WARN("Error opening file to write cells data");
                return false;
            }

            QXmlStreamWriter stream(&output);
            stream.setAutoFormatting(true);
            stream.writeStartDocument();

            stream.writeStartElement("board");

            for (size_t i=0; i< board.size();++i)
            {
                stream.writeStartElement("cell");
                stream.writeAttribute("id", QString::number(i));
                for (t_Cell::const_iterator it_vertex = board[i].begin(); it_vertex != board[i].end(); ++it_vertex) {
                    stream.writeEmptyElement("vertex");
                    stream.writeAttribute("x", QString::number(it_vertex->x));
                    stream.writeAttribute("y", QString::number(it_vertex->y));
                }
                stream.writeEndElement(); // cell
            }

            stream.writeEndElement(); // board
            stream.writeEndDocument();
            output.close();
        }
        else return false;
    }
    else
    {
        ROS_INFO("No cells in the board to be saved.");
        return false;
    }
    return true;
}

/**
 Reads the contours of the cells from the parameter server.
 The contours are stored in the parameter server, in a param named cells_param.
 @param board where the contours are gonna be stored
 @param cells_param the name of the parameter where the raw data from the cells is stored. This data is formated as a xml file.
 The format of the data is as follows:
            <board>
                <cell id="0">
                    <vertex x="195" y="50"/>
                    [...]
                </cell>
                [...]
            </board>
 @return true if cell data have been successfully loaded, false otherwise.
 */
bool Cells::read_from_parameter_server(t_Board& board, std::string cells_param)
{
    ROS_DEBUG("Ready to read xml data of cells");

    std::string xml_cells;

    if (ros::param::get(cells_param, xml_cells)) //reading the xml data from the parameter server
    {
        ROS_INFO("xml cell data successfully loaded");

        board.clear(); //cleaning all previous cells

        QXmlStreamReader xml;
        xml.addData(QString::fromStdString(xml_cells)); //adding the xml content to the input stream
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
                    board.push_back(t_Cell());
                    continue;
                }
                /* If it's named vertex, we'll dig the information from there.*/
                if(xml.name() == "vertex") {
                    /* Let's get the attributes for vertex */
                    QXmlStreamAttributes attributes = xml.attributes();
                    /* Let's check that vertex has x and y attribute. */
                    if(attributes.hasAttribute("x") && attributes.hasAttribute("y")) {
                        /* We'll add it to the cell */
                        board.back().push_back(cv::Point(attributes.value("x").toString().toInt(), attributes.value("y").toString().toInt()));
                    }
                    else xml.raiseError("Vertex corrupted: x and/or y value is missing");
                }
            }
        }
        ROS_WARN_COND(xml.hasError(),"Error parsing xml data (l.%d, c.%d):%s", (int)xml.lineNumber(), (int)xml.columnNumber(), xml.errorString().toStdString().c_str());

        ROS_INFO("Xml data successfully loaded. %d cells loaded", (int)board.size());
        return true;
    }
    else
    {
        ROS_FATAL_STREAM("xml cell data not loaded!");
        return false;
    }
}

/**
 Computes the centroid of a cell, i.e. the center point considering all the points in the contour.
 @param cell the cell whose centroid is going to be computed
 @param centroid the central point. This is used like an output param in the function.
 @return true if the centroid has been successfully computed, false otherwise. In this case, it returns false if the number of points in the contour is 0.
 */
bool Cells::get_centroid_of_cell(const t_Cell& cell, cv::Point& centroid)
{
    uint sumX = 0, sumY = 0;
    size_t size = cell.size();
    centroid.x=centroid.y=0;
    if(size > 0){
        for (t_Cell::const_iterator it_point = cell.begin(); it_point != cell.end(); ++it_point) {
            sumX += it_point->x;
            sumY += it_point->y;
        }
        // TODO through exception if size <= 0
        centroid.x = sumX/size;
        centroid.y = sumY/size;
        return true;
    }
    else
        return false;
}

/**
 Returns a mask for a cell.
 That is, a new image that keeps just the portion of the oringinal image corresponding to a cell. The rest of the image is set to black.
 @param img the original image, where the mask will be extracted from.
 @param cell the cell delimiting the mask, that is, the portion of the original image that will be preserved.
 @return the mask for a cell. It has the same size than the original image img.
 */
cv::Mat Cells::masked_cell_image(const cv::Mat img, const t_Cell cell)
{
    cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
    cv::drawContours(mask, t_Board(1,cell), -1, cv::Scalar(255), CV_FILLED);  // CV_FILLED fills the connected components found with white (white RGB value = 255,255,255)

    cv::Mat im_crop(img.rows, img.cols, CV_8UC3);                           // let's create a new 8-bit 3-channel image with the same dimensions
    im_crop.setTo(cv::Scalar(0));                                           // we fill the new image with a color, in this case we set background to black.
    img.copyTo(im_crop, mask);                                              // copy just the elements from the original image indicated by the non-zero elements from mask to crop
    /*cv::normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);*/      // normalize so imwrite(...)/imshow(...) shows the mask correctly!

    // show the images
    /*cv::imshow("original", img);
    cv::imshow("mask", mask);
    cv::imshow("masked cell", im_crop);*/

    return im_crop;
}

}
