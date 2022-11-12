#include <iostream>
#include <set>
#include <vector>
#include <map>
#include <fstream>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include<cmath>
#include <ctime>
using namespace std;
using namespace cv;
#define IMAGE_FOLDER "./imgs/"//图片保存的路径(这是张正有的标定靶所在的图片位置)
#define THREED_FOLDER "3d/"//这是3D标靶照片所在的位置(待矫正的图片)
#define RESULT_FOLDER "./result/" //提取的角点后绘制的图像位置
#define REMAP_FOLDER "./remap/" //图像经过畸变矫正后存储的位置
int main()
{
	//读取张正友图片的文件夹
	string path = IMAGE_FOLDER;
	vector<String> image_file;//该数组存储的是图片的位置信息
	glob(path, image_file);

	//读取3d标靶的文件夹
	path = THREED_FOLDER;
	vector<String> three_file;
	glob(path, three_file);


	cout << "图片总数是：" << image_file.size() << endl;
	Size patternsize(7, 7); //number of centers
	float Dis = 20.0;//该值是标定靶中两个圆心相距的距离  单位是mm
	//生成自己的世界坐标系
	vector<Point3f> worldPoints;
	for (int k = 0; k < patternsize.height; k++)
	{
		for (int j = 0; j < patternsize.width; j++)
		{
			worldPoints.push_back(Point3f(j, k, 0)*Dis);
		}
	}
	//自己生成的世界坐标系
	Size imagesize;
	vector < vector<Point2f>> image_points;
	vector < vector<Point3f>> object_points;
	for (int i = 0; i < image_file.size(); i++)
	{
		Mat src = imread(image_file[i]);
		imagesize.height = src.rows;
		imagesize.width = src.cols;

		//imshow("测试图", src);

		Mat gray;
		cvtColor(src, gray, CV_BGR2GRAY);
		vector<Point2f> centers; //this will be filled by the detected centers
		bool patternfound = findCirclesGrid(gray, patternsize, centers, CALIB_CB_SYMMETRIC_GRID);
		if (patternfound)
		{
			cout << image_file[i] << "寻找角点成功!" << endl;
			drawChessboardCorners(src, patternsize, Mat(centers), patternfound);
			image_points.push_back(centers);
			//imshow("result", src);
			imwrite(RESULT_FOLDER + to_string(i + 1) + ".jpg", src);
			//cout << "得到的点的坐标是: " << centers << endl;
			//imshow("找到了", src);

			//cout << "生成的世界坐标系是：" << endl << worldPoints << endl;
			object_points.push_back(worldPoints);
		}
		else
			cout << image_file[i] << "角点寻找失败" << endl;
	}
    clock_t start, finish;
	//开始计算标定
	//cameraMateix 属于相机内参   disCoeffs 属于相机的畸变系数
	Mat cameraMatrix, distCoeffs;
	vector<Mat> rvecs, tvecs;
    start = clock();
	double error = calibrateCamera(object_points, image_points, imagesize, cameraMatrix, distCoeffs, rvecs, tvecs, 0);
    finish = clock();
	cout << "标定完成！" <<"运行时间:" << double(finish - start) / CLOCKS_PER_SEC <<endl;
	cout << "内参矩阵是：" << endl << cameraMatrix << endl;
	cout << "误差是：" << error << endl;
	cout << "畸变系数是：" << distCoeffs << endl;
    cout << "待校正图片总数是：" << three_file.size() << endl;
	//开始进行图像校正
	for (int i = 0; i < three_file.size(); i++)
	{
		Mat src = imread(three_file[i]);
        cout<<REMAP_FOLDER + three_file[i]<<endl;
		Mat dst;
		undistort(src, dst, cameraMatrix, distCoeffs);
		imwrite(REMAP_FOLDER + three_file[i], dst);
		//cout << "校正成功！" << endl;
	}

	cout << "纠正完成" << endl;
	waitKey(0);
	return 0;
}
//#include <iostream>
//#include <fstream>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//
//using namespace std;
//using namespace cv;
//
//int main(int argc, char **argv) {
//
//    ifstream fin("calibimage.txt");            //读取标定图片的路径，与cpp程序在同一路径下
//    if (!fin)                                  //检测是否读取到文件
//    {
//        cerr<<"没有找到文件"<<endl;
//    }
//    ofstream fout("calibration_result.txt");   //输出结果保存在此文本文件下
//    //依次读取每一幅图片，从中提取角点
//    cout<<"开始提取角点……"<<endl;
//    int image_count = 0;                       //图片数量
//    Size image_size;                           //图片尺寸
//    Size board_size = Size(7,7);               //标定板每行每列角点个数，共7*7个角点
//    vector<Point2f> image_points_buf;          //缓存每幅图检测到的角点
//    vector<vector<Point2f>> image_points_seq;  //用一个二维数组保存检测到的所有角点
//    string filename;                           //申明一个文件名的字符串
//
//    while (getline(fin,filename))              //逐行读取，将行读入字符串
//    {
//        image_count++;
//        cout<<"image_count = "<<image_count<<endl;
//        //读入图片
//        Mat imageInput=imread(filename);
//
//        image_size.height = imageInput.rows;//图像的高对应着行数
//        image_size.width = imageInput.cols; //图像的宽对应着列数
//        cout<<"image_size.width = "<<image_size.width<<endl;
//        cout<<"image_size.height = "<<image_size.height<<endl;
//
//        //角点检测
//        if (findChessboardCorners(imageInput, board_size, image_points_buf) == 0)
//        {
//            cout<<"can not find the corners "<<endl;
//            exit(1);
//        }
//        else
//        {
//            Mat view_gray;                      //存储灰度图的矩阵
//            cvtColor(imageInput, view_gray, CV_RGB2GRAY);//将RGB图转化为灰度图
//            //亚像素精确化（两种方法）
//            find4QuadCornerSubpix(view_gray, image_points_buf, Size(5,5));
//            //cornerSubPix(view_gray,image_points_buf,Size(5,5));
//            image_points_seq.push_back(image_points_buf);//保存亚像素角点
//            //在图中画出角点位置
//            drawChessboardCorners(view_gray, board_size, image_points_buf, true);//将角点连线
//            imshow("Camera calibration", view_gray);
//            waitKey(100);                         //等待按键输入
//        }
//    }
//    //输出图像数目
//    int total = image_points_seq.size();
//    cout<<"total = "<<total<<endl;
//    int CornerNum = board_size.width*board_size.height;//一幅图片中的角点数
//    //以第一幅图片为例，下同
//    cout<<"第一副图片的角点数据:"<<endl;
//    for (int i=0; i<CornerNum; i++)
//    {
//        cout<<"x= "<<image_points_seq[0][i].x<<" ";
//        cout<<"y= "<<image_points_seq[0][i].y<<" ";
//        cout<<endl;
//    }
//    cout<<"角点提取完成!\n";
//
//    //开始相机标定
//    cout<<"开始标定……"<<endl;
//    Size square_size = Size(20,20);              //每个小方格实际大小
//    vector<vector<Point3f>> object_points;         //保存角点的三维坐标
//    Mat cameraMatrix = Mat(3,3,CV_32FC1,Scalar::all(0));//内参矩阵3*3
//    Mat distCoeffs = Mat(1,5,CV_32FC1,Scalar::all(0));//畸变矩阵1*5
//    vector<Mat> rotationMat;                       //旋转矩阵
//    vector<Mat> translationMat;                    //平移矩阵
//    //初始化角点三维坐标
//    int i,j,t;
//    for (t=0; t<image_count; t++)
//    {
//        vector<Point3f> tempPointSet;
//        for (i=0; i<board_size.height; i++)       //行
//        {
//            for (j=0;j<board_size.width;j++)      //列
//            {
//                Point3f realpoint;
//                realpoint.x = i*square_size.width;
//                realpoint.y = j*square_size.height;
//                realpoint.z = 0;
//                tempPointSet.push_back(realpoint);
//            }
//        }
//        object_points.push_back(tempPointSet);
//    }
//    vector<int> point_counts;
//    for (i=0; i<image_count; i++)
//    {
//        point_counts.push_back(board_size.width*board_size.height);
//    }
//    //标定
//    calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs,rotationMat, translationMat,0);   //拥有八个参数的标定函数，不过一句话搞定
//    cout<<"标定完成！"<<endl;
//
//    //对标定结果进行评价
//    double total_err = 0.0;                      //所有图像平均误差总和
//    double err = 0.0;                            //每幅图像的平均误差
//    vector<Point2f> image_pointsre;              //重投影点
//    cout<<"\t每幅图像的标定误差：\n";
//    fout<<"每幅图像的标定误差：\n";
//    for (i=0; i<image_count; i++)
//    {
//        vector<Point3f> tempPointSet = object_points[i];
//        //通过之前标定得到的相机内外参，对三维点进行重投影
//        projectPoints(tempPointSet, image_pointsre, rotationMat[i], translationMat[i], cameraMatrix, distCoeffs);
//        //计算两者之间的误差
//        vector<Point2f> tempImagePoint = image_points_seq[i];
//        Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);//变为1*20的矩阵
//        Mat image_pointsreMat = Mat(1, image_pointsre.size(), CV_32FC2);
//        for (int j = 0 ; j < tempImagePoint.size(); j++)
//        {
//            tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
//            image_pointsreMat.at<Vec2f>(0,j) = Vec2f(image_pointsre[j].x, image_pointsre[j].y);
//        }
//        err = norm(image_pointsreMat, tempImagePointMat, NORM_L2);
//        total_err += err/=  point_counts[i];
//        cout<<"第"<<i+1<<"幅图像的平均误差为： "<<err<<"像素"<<endl;
//        fout<<"第"<<i+1<<"幅图像的平均误差为： "<<err<<"像素"<<endl;
//    }
//    cout<<"总体平均误差为： "<<total_err/image_count<<"像素"<<endl;
//    fout<<"总体平均误差为： "<<total_err/image_count<<"像素"<<endl;
//    cout<<"评价完成！"<<endl;
//
//    //将标定结果写入txt文件
//    cout<<"开始保存结果……"<<endl;
//    Mat rotate_Mat = Mat(3,3,CV_32FC1, Scalar::all(0));//保存旋转矩阵
//    fout<<"相机内参数矩阵："<<endl;
//    fout<<cameraMatrix<<endl<<endl;
//    fout<<"畸变系数：\n";
//    fout<<distCoeffs<<endl<<endl<<endl;
//    for (int i=0; i<image_count; i++)
//    {
//        Rodrigues(rotationMat[i], rotate_Mat); //将旋转向量通过罗德里格斯公式转换为旋转矩阵
//        fout<<"第"<<i+1<<"幅图像的旋转矩阵为："<<endl;
//        fout<<rotate_Mat<<endl;
//        fout<<"第"<<i+1<<"幅图像的平移向量为："<<endl;
//        fout<<translationMat[i]<<endl<<endl;
//    }
//    cout<<"保存完成"<<endl;
//    fout<<endl;
//
//    return 0;
//}
//
