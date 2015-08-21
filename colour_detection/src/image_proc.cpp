
#include "colour_detection/colour_detection_node.h"


namespace colour_detect {


int ColourVision::fluxWebBE(cv::Mat &newFrame)
{  cv::Mat smooth;
   cv::Mat hsv;
   cv::Mat imGloBin;


    
   smooth = cv::Mat(newFrame.rows,newFrame.cols,CV_8UC3);//cvCreateImage(cvGetSize(newFrame),newFrame->depth,3);
   imGloBin = cv::Mat(newFrame.rows,newFrame.cols,CV_8UC1);//cvCreateImage(cvGetSize(newFrame),newFrame->depth,1);
   hsv = cv::Mat(newFrame.rows,newFrame.cols,CV_8UC3);//cvCreateImage(cvGetSize(newFrame),newFrame->depth,newFrame->nChannels);



   cv::boxFilter(newFrame , smooth , smooth .depth(), cv::Size(6, 6), cv::Point(-1,-1),CV_BLUR, cv::BORDER_REPLICATE );
   cv::cvtColor(smooth, hsv, CV_BGR2HSV);
   cv::inRange(hsv,cv::Scalar(seuilBh,seuilBs,seuilBv),cv::Scalar(seuilHh,seuilHs,seuilHv),imGloBin);

   cv::erode(imGloBin, imGloBin, cv::Mat(), cv::Point(-1,-1), nbIter,cv::BORDER_CONSTANT,cv::morphologyDefaultBorderValue() );
   cv::dilate(imGloBin, imGloBin, cv::Mat(), cv::Point(-1,-1), nbIterD,cv::BORDER_CONSTANT,cv::morphologyDefaultBorderValue() );

   std::vector<std::vector<cv::Point> > contours;
   std::vector<cv::Vec4i> hierarchy;
   cv::findContours( imGloBin,contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
   //cvDrawContours( newFrame, cont,
       //            CV_RGB(255,255,0), CV_RGB(0,255,0),
            //                      1, 2, 8, cvPoint(0,0));
   newFrame.copyTo(newFrame,imGloBin);
   
   cv::drawContours( newFrame, contours, -1, cv::Scalar(0, 255, 255, 0.0), 2, 8, hierarchy, 2, cv::Point() );

   /*for(c=cont; c!=NULL; c=c->h_next )
   {
      result = cvApproxPoly(c, sizeof(CvContour), memCon, CV_POLY_APPROX_DP, cvContourPerimeter(cont)*0.99, 0);
      if(result->total==4 &&cvContourArea(result, CV_WHOLE_SEQ, 0 )>80 )
      {
          cvDrawContours( newFrame, result,
                            CV_RGB(0,0,255), CV_RGB(0,0,255),
                                             1, -1, 8, cvPoint(0,0));
       }
   }*/
}

}
