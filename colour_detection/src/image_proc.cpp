
#include "colour_detection/colour_detection_node.h"


namespace colour_detect {


int ColourVision::fluxWebBE(char *nomFenetre,char *nameSaved){
    CvCapture *stream = cvCreateCameraCapture(0);
    char key = 'n';

    if (!stream){
        printf("Echec du flux video\n");
        return 0;
    }
    cvNamedWindow(nomFenetre, CV_WINDOW_AUTOSIZE );
    nomFenet = nomFenetre;
    while(key!='q'&& key!='Q'){
        frame = cvQueryFrame(stream);

        if (!frame) break;
        smooth = cvCreateImage(cvGetSize(frame),frame->depth,3);
        imGloBin = cvCreateImage(cvGetSize(frame),frame->depth,1);
        hsv = cvCreateImage(cvGetSize(frame),frame->depth,frame->nChannels);

        memCon = cvCreateMemStorage(0);

        key = cvWaitKey(10);
        trackbarBE(0);
        if (key == 's' || key == 'S'){
            cvSaveImage(nameSaved,frame,0);
            key ='n';
        }

        cvReleaseImage(&smooth);
        cvReleaseImage(&hsv);
        cvReleaseImage(&imGloBin);
        cvReleaseMemStorage(&memCon);
    }
    cvDestroyWindow(nomFenetre);
    cvReleaseCapture(&stream);
}


void ColourVision::trackbarBE(int k){
   cvSmooth(frame, smooth, CV_BLUR, 6, 6, 0.0, 0.0);
   cvCvtColor(smooth,hsv,CV_BGR2HSV);
   cvInRangeS(hsv,cvScalar(seuilBh,seuilBs,seuilBv,0),cvScalar(seuilHh,seuilHs,seuilHv,0),imGloBin);

   cvErode(imGloBin, imGloBin, NULL, nbIter);
   cvDilate(imGloBin, imGloBin, NULL, nbIterD);



   CvSeq* cont=0;
   CvSeq* result;
   CvSeq* c;
   cvFindContours( imGloBin, memCon,
                       &cont, sizeof(CvContour),
                        CV_RETR_LIST, CV_CHAIN_APPROX_NONE,
                        cvPoint(0,0) );
   cvDrawContours( frame, cont,
                   CV_RGB(255,255,0), CV_RGB(0,255,0),
                                  1, 2, 8, cvPoint(0,0));
   printf("poly");

   for(c=cont; c!=NULL; c=c->h_next ){

        result = cvApproxPoly(c, sizeof(CvContour), memCon, CV_POLY_APPROX_DP, cvContourPerimeter(cont)*0.99, 0);
        if(result->total==4 &&cvContourArea(result, CV_WHOLE_SEQ, 0 )>80 )
        {
            cvDrawContours( frame, result,
                            CV_RGB(0,0,255), CV_RGB(0,0,255),
                                             1, -1, 8, cvPoint(0,0));
        }
    }

   printf("end");

   cvNamedWindow(nomFenet,CV_WINDOW_AUTOSIZE);

}

}
