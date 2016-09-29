/**Lucas Kannade Inverse Compositional Algorithm:
Inputs:

Outputs: Transformation Matrix

Author: Bismaya Sahoo
Email: bsahoo@uwaterlo.ca**/

#include <nidhi/headers.h>
#include <nidhi/nidhi_system.h>
#include <nidhi/nidhi_tracker.h>
#include <nidhi/nidhi_mapper.h>
#include <nidhi/nidhi_datastruct.h>

cv::Mat LucasKannadeICA(vector<cv::Mat> image_KF_p,vector<cv::Mat> image_CF_p,cv::Mat K,\
						int pyramid_levels,/*can be removed*/
						vector<cv::Mat> points_3D_KF/*Already has the depth hypothesis along with 1*/)
{
	cv::Mat R_initial=cv::Mat::eye(3,3,CV_64FC1);
	cv::Mat t_initial=cv::Mat::zeros(3,1,CV_64FC1);
	cv::Mat T_initial=cv::Mat::eye(4,4,CV_64FC1);

	cv::Mat Jac=cv::Mat::zeros(3,6,CV_64FC1);
	vector<cv::Mat> points_3D;
	for (int plvl=0;plvl<pyramid_levels;plvl++)
	{
		//within Pyramid levels

		/*Create a New structure for KFs;where the precomputation part is already handled whenever the constructr is called.
		Written here for reference.*/

		//Precomputation
		//Step1: Gradients of KF(template)
		cv::Mat GX=gradientX(image_KF_p[i],1);
		cv::Mat GY=gradientY(image_KF_p[plvl],1);
		//Step2: Calculate the Jacobian

		/*ensure proper format.3D points have centered coordinates*/
		points_3D[plvl]=T_initial*points_3D_KF[plvl];
		points_3D_CF[plvl]=T_initial*points_3D_KF[plvl];

		cv::Mat points_2D_KF=cv::Mat::zeros(3,points_3D[plvl].cols,CV_64FC1);
		points_2D_KF.rowRange(0,1).colRange(0,points_3D.cols)=points_3D[plvl].rowRange(0,1).colRange(0,points_3D.cols)/points_3D[plvl].rowRange(2,3).colRange(0,points_3D.cols);
		points_2D_KF.rowRange(1,2).colRange(0,points_3D.cols)=points_3D[plvl].rowRange(1,2).colRange(0,points_3D.cols)/points_3D[plvl].rowRange(2,3).colRange(0,points_3D.cols);
		points_2D_KF.rowRange(1,2).colRange(0,points_3D.cols)=cv::Mat::ones(1,colRange(0,points_3D.cols));
		points_2D_KF[plvl]=points_2D_KF.clone();

		points_2D_KF[plvl]=K*points_2D_KF[plvl];//Cx,Cy,Fx,Fy.Ensure 0,0//
		//double check the order of the aboce according to: x=KX;
		
		cv::Mat points_2D_CF=cv::Mat::zeros(3,points_3D_CF[plvl].cols,CV_64FC1);
		points_2D_CF.rowRange(0,1).colRange(0,points_3D_CF.cols)=points_3D_CF[plvl].rowRange(0,1).colRange(0,points_3D_CF.cols)/points_3D_CF[plvl].rowRange(2,3).colRange(0,points_3D_CF.cols);
		points_2D_CF.rowRange(1,2).colRange(0,points_3D_CF.cols)=points_3D_CF[plvl].rowRange(1,2).colRange(0,points_3D_CF.cols)/points_3D_CF[plvl].rowRange(2,3).colRange(0,points_3D_CF.cols);
		points_2D_CF.rowRange(1,2).colRange(0,points_3D_CF.cols)=cv::Mat::ones(1,colRange(0,points_3D_CF.cols));
		points_2D_CF[plvl]=points_2D_CF.clone();

		points_2D_CF[plvl]=K*points_2D_CF[plvl];//Cx,Cy,Fx,Fy,Ensure 0,0//
		
		/*Compute Jacobian of the KF matrix*/
		

		/*Compute Error*/
		//Check if points in CF are within image boundaries//
		int imsize_y,imsize_x;
		imsize_x=image_CF_p.cols-2;
		imsize_y=image_CF_p.rows-2;
		
		for(int ii=0;ii<points_2D_CF[plvl].cols;ii++)
		{
			if(points_2D_CF[plvl].at<double>(0,ii)>2 && points_2D_CF[plvl].at<double>(0,ii)<imsize_x \
				&& points_2D_CF[plvl].at<double>(1,ii)>2 && points_2D_CF[plvl].at<double>(1,ii)<imsize_y)
			{

			}
			else
			{

			}

		//end Precomputation



		//outside Pyramid levels
	}
} 

cv::Mat gradientY(cv::Mat &mat, float spacing)
{
    cv::Mat grad = cv::Mat::zeros(mat.rows,mat.cols,CV_32F);

    const int maxCols = mat.cols;
    const int maxRows = mat.rows;

    /*get gradients in each border*/
    /*first row*/
    cv::Mat row = (-mat.row(0) + mat.row(1))/(float)spacing;
    row.copyTo(grad(cvRect(0,0,maxCols,1)));

    /*last row*/
    row = (-mat.row(maxRows-2) + mat.row(maxRows-1))/(float)spacing;
    row.copyTo(grad(cvRect(0,maxRows-1,maxCols,1)));

    /*centered elements*/
    cv::Mat centeredMat = mat(cvRect(0,0,maxCols,maxRows-2));
    cv::Mat offsetMat = mat(cvRect(0,2,maxCols,maxRows-2));
    cv::Mat resultCenteredMat = (-centeredMat + offsetMat)/(((float)spacing)*2.0);

    resultCenteredMat.copyTo(grad(cvRect(0,1,maxCols, maxRows-2)));
    return grad;
}

cv::Mat gradientX(cv::Mat & mat, float spacing)
{
    cv::Mat grad = cv::Mat::zeros(mat.rows,mat.cols,CV_32F);

    int maxCols = mat.cols;
    int maxRows = mat.rows;

    /* get gradients in each border */
    /* first col */
    cv::Mat col = (-mat.col(0) + mat.col(1))/(float)spacing;
    col.copyTo(grad(cvRect(0,0,1,maxRows)));

    /*  last col */
    col = (-mat.col(maxCols-2) + mat.col(maxCols-1))/(float)spacing;
    col.copyTo(grad(cvRect(maxCols-1,0,1,maxRows)));

    /* centered elements */
    cv::Mat centeredMat = mat(cvRect(0,0,maxCols-2,maxRows));
    cv::Mat offsetMat = mat(cvRect(2,0,maxCols-2,maxRows));
    cv::Mat resultCenteredMat = (-centeredMat + offsetMat)/(((float)spacing)*2.0);

    resultCenteredMat.copyTo(grad(cvRect(1,0,maxCols-2, maxRows)));
    return grad;
}

