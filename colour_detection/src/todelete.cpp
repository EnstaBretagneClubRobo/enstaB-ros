
#include "colour_detection/colour_detection_node.h"

namespace colour_detect {
/*void ColourVision::ImageInfo(const char *imageFile) //Question 1
{
        IplImage *image = cvLoadImage(imageFile,CV_LOAD_IMAGE_COLOR);
        perror("");
        printf("\nFilename: %s\n", imageFile);
        
        printf("width: %u\n", image->width);
        printf("height: %u\n", image->height);
        printf("nChannels: %u\n", image->nChannels);
        printf("depth: %u bits\n", cvIplDepth(image->depth) ); //Permet de passer du code IPL_DEPTH_XXX au nombre de bits utilisés
        printf("Image size: %u Ko\n", (int)round((double)image->imageSize/1000.0) );

        cvReleaseImage(&image);
}

float ColourVision::ImageStat(IplImage *image) //Question 2
{

        CvScalar moy , ectp;
        cvAvgSdv(image, &moy, &ectp, NULL);

        double mini=0, maxi=0;

        unsigned int c;
        for(c=0; c<image->nChannels; c++)
        {
                cvSetImageCOI(image,c+1);
                cvMinMaxLoc(image, &mini, &maxi, NULL, NULL, NULL);

                printf(	"canal %u -> min = %u  max = %u  moyenne = %5.1f  ecart-type = %5.1f\n", c+1, (int)mini, (int)maxi, moy.val[c], ectp.val[c]);
        }
         cvSetImageCOI(image,0); // Permet de réinitialiser ChannelOfInterest à la valeur All pour que cvAbsDiff fonctionne correctement
        return moy.val[c];
     }

void ColourVision::ImageDisplay(IplImage *image, const char *windowName) //Question 3
{
        cvNamedWindow(windowName,CV_WINDOW_AUTOSIZE);

        cvShowImage(windowName,image);

        cvWaitKey(0);

        cvDestroyWindow(windowName);
}

int ColourVision::fluxWeb(char *nomFenetre,char *nameSaved){
    CvCapture *stream = cvCreateCameraCapture(0);
    IplImage *frame = 0;
    char key = 'n';

    if (!stream){
        printf("Echec du flux video\n");
        return 0;
    }
    cvNamedWindow(nomFenetre, CV_WINDOW_AUTOSIZE );

    while(key!='q'&& key!='Q'){
        frame = cvQueryFrame(stream);
        if (!frame) break;
        cvShowImage(nomFenetre,frame);
        key = cvWaitKey(10);
        //printf("%d",(int)key);
        if (key == 's' || key == 'S'){
            cvSaveImage(nameSaved,frame,0);
            key ='n';
        }
    }
    cvDestroyWindow(nomFenetre);
    cvReleaseCapture(&stream);
}

void ColourVision::DecompoCanaux(char *nomfichier,char *nFicRouge,char *nFicVert,char *nFicBleu){
    IplImage *image = cvLoadImage(nomfichier,CV_LOAD_IMAGE_COLOR);

    IplImage *imageRouge = cvCreateImage(cvGetSize(image),image->depth,1);
    IplImage *imageVert = cvCreateImage(cvGetSize(image),image->depth,1);
    IplImage *imageBleu = cvCreateImage(cvGetSize(image),image->depth,1);

    cvSplit(image,imageBleu,imageVert,imageRouge,NULL);
    cvReleaseImage(&image);

    cvSaveImage(nFicRouge,imageRouge,0);
    cvSaveImage(nFicVert,imageVert,0);
    cvSaveImage(nFicBleu,imageBleu,0);

    cvReleaseImage(&imageRouge);
    cvReleaseImage(&imageVert);
    cvReleaseImage(&imageBleu);
}

IplImage *ColourVision::ImageVisetir(IplImage *img){
    int k;//variable canal
    int i;//variable width
    int j;//variable height
    IplImage *buff = cvCreateImage(cvGetSize(img),IPL_DEPTH_32F,img->nChannels);

    cvConvertScale(img,buff,1.0,0.0);// convertit buff en flottant

    IplImage *high = cvCreateImage(cvGetSize(img),IPL_DEPTH_32F,img->nChannels);// reponse

    float* buffData = (float*)buff->imageData;
    float* highData = (float*)high->imageData;
    int step = buff->widthStep/sizeof(float);

    double mini, maxi;
    for( k=0; k<img->nChannels;k++){
         cvSetImageCOI(img, k+1);//change de canal pour la fonction minmaxloc
         cvMinMaxLoc(img, &mini, &maxi, NULL, NULL, NULL);

         for( j=0;j<img->height;j++)//etirement de l'histogramme
             for( i=0;i<img->width;i++)
                   highData[j*step+i*img->nChannels+k] = (buffData[j*step+i*img->nChannels+k]-mini)/(maxi-mini);
    }

    cvReleaseImage(&buff);

    cvSetImageCOI(img, 0);//selection de tous les canaux

    return high;
}

IplImage *ColourVision::FiltreMoy(IplImage *image,int dimFiltre){
    IplImage *imfiltre = cvCreateImage(cvGetSize(image),image->depth,image->nChannels);
    cvSmooth(image,imfiltre,CV_GAUSSIAN,dimFiltre,0.0,0.0,0.0);
    return imfiltre;
}

IplImage *ColourVision::FiltreMedian(IplImage *image,int dimFiltre){
    IplImage *imfiltre = cvCreateImage(cvGetSize(image),image->depth,image->nChannels);
    cvSmooth(image,imfiltre,CV_MEDIAN,dimFiltre,0.0,0.0,0.0);
    return imfiltre;
}

//IplImage *filtreSobel(IplImage *image){
    //float filtreX[] = {{-1,0,1},{-2,0,2},{-1,0,1}};
   // CvMat cvmatX= cvMat(3,3,CV_32F,filtreX);
   // float filtreY[] = {{1,2,1},{0,0,0},{-1,-2,-1}};
   // CvMat cvmatY= cvMat(3,3,CV_32F,filtreY);
   // IplImage *imfiltreX = cvCreateImage(cvGetSize(image),image->depth,image->nChannels);
   // IplImage *imfiltreY = cvCreateImage(cvGetSize(image),image->depth,image->nChannels);
   // cvFilter2D(image,imfiltreX,&cvmatX,cvPoint(0,0));
   // cvFilter2D(image,imfiltreY,&cvmatY,cvPoint(0,0));
   // cvCartToPolar(imfiltreX,imfiltreY)
   //  return imfiltre;
//}

IplImage * ColourVision::filtreSobel(IplImage *img){
    IplImage *img_sobel=cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 3 );

    float k[] = {-1.0, 0, 1.0,
                 -2.0, 0, 2.0,
                 -1.0, 0, 1.0 };
    CvMat kernel = cvMat(3,3,CV_32F,k);
    IplImage *convX = cvCreateImage(cvGetSize(img),IPL_DEPTH_32F,3);
    cvFilter2D(img, convX, &kernel, cvPoint(0,0));
    IplImage *convY = cvCreateImage(cvGetSize(img),IPL_DEPTH_32F,3);
    cvTranspose(&kernel,&kernel);
    cvFilter2D(img, convY, &kernel, cvPoint(0,0));
    cvCartToPolar(convX, convY, img_sobel, NULL, 0);
    return img_sobel;
}

IplImage * ColourVision::canny(IplImage *image){
    IplImage *img_canny=cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 3 );
    cvCanny(image,img_canny,0.2,0.8,3);
    return img_canny;
}


IplImage* ColourVision::differe(IplImage *img1, IplImage *img2){
    IplImage *img0 = cvCreateImage( cvGetSize(img1), img1->depth, img2->nChannels);
    cvAbsDiff(img1,img2,img0);
    return img0;

}

IplImage* ColourVision::moyGrey(IplImage *image){
    IplImage *imageRouge = cvCreateImage(cvGetSize(image),image->depth,1);
    IplImage *imageVert = cvCreateImage(cvGetSize(image),image->depth,1);
    IplImage *imageBleu = cvCreateImage(cvGetSize(image),image->depth,1);

    cvSplit(image,imageBleu,imageVert,imageRouge,NULL);


    IplImage *moyenne = cvCreateImage(cvGetSize(image),image->depth,1);

    cvAddWeighted(imageRouge, 0.33, imageVert, 0.33, 0.0, moyenne);
    cvAddWeighted(moyenne, 1.0,imageBleu, 0.33, 0.0, moyenne);

    return moyenne;
}

IplImage* ColourVision::mulBin(IplImage *image,IplImage *imGloBin){
    IplImage *imBin = imGloBin;
    IplImage *imageRouge = cvCreateImage(cvGetSize(image),image->depth,1);
    IplImage *imageVert = cvCreateImage(cvGetSize(image),image->depth,1);
    IplImage *imageBleu = cvCreateImage(cvGetSize(image),image->depth,1);

    IplImage *imBrouge = cvCreateImage(cvGetSize(image),image->depth,1);
    IplImage *imBvert = cvCreateImage(cvGetSize(image),image->depth,1);
    IplImage *imBbleu = cvCreateImage(cvGetSize(image),image->depth,1);

    cvSplit(image,imageBleu,imageVert,imageRouge,NULL);
    cvMul(imageRouge,imBin,imBrouge,1);
    cvMul(imageVert,imBin,imBvert,1);
    cvMul(imageBleu,imBin,imBbleu,1);

    IplImage *dest = cvCreateImage(cvGetSize(image),image->depth,3);
    cvMerge(imBrouge,imBvert,imBbleu,0,dest);

    return dest;
}

*/


}
