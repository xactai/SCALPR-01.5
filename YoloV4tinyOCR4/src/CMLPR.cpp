
#include <iostream>
#include <baseapi.h>
#include <allheaders.h>

#include "core/core.hpp"
#include "highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "CMLPR.h"

using namespace cv;
using namespace std;

Mat RGBToGray(Mat rgb)
{
	int stride = 3;
	Mat gray = Mat::zeros(rgb.size(), CV_8UC1);

	for (size_t i = 0; i < rgb.rows; i++)
	{
		for (size_t j = 0; j < rgb.cols * stride; j += stride)
		{
			auto r = rgb.at<uchar>(i, j);
			auto g = rgb.at<uchar>(i, j + 1);
			auto b = rgb.at<uchar>(i, j + 2);

			gray.at<uchar>(i, j / stride) = (r + g + b) / stride;
		}
	}

	return gray;
}

Mat RGBToBinary(Mat rgb, int threshold = 128)
{
	int stride = 3;
	Mat binary = Mat::zeros(rgb.size(), CV_8UC1);

	for (size_t i = 0; i < rgb.rows; i++)
	{
		for (size_t j = 0; j < rgb.cols * stride; j += stride)
		{
			auto r = rgb.at<uchar>(i, j);
			auto g = rgb.at<uchar>(i, j + 1);
			auto b = rgb.at<uchar>(i, j + 2);
			auto result = (r + g + b) / stride;

			if (result > threshold)
				binary.at<uchar>(i, j / stride) = 255;
		}
	}

	return binary;
}

Mat GrayToBinary(Mat gray, int threshold = 128)
{
	Mat binary = Mat::zeros(gray.size(), CV_8UC1);

	for (size_t i = 0; i < gray.rows; i++)
	{
		for (size_t j = 0; j < gray.cols; j += 1)
		{
			if (gray.at<uchar>(i, j) > threshold)
				binary.at<uchar>(i, j) = 255;
		}
	}

	return binary;
}

Mat GrayInversion(Mat gray)
{
	Mat inversion = Mat::zeros(gray.size(), CV_8UC1);

	for (size_t i = 0; i < gray.rows; i++)
	{
		for (size_t j = 0; j < gray.cols; j += 1)
		{
			inversion.at<uchar>(i, j) = 255 - gray.at<uchar>(i, j);
		}
	}

	return inversion;
}

Mat GrayStep(Mat gray, int minThreshold = 80, int maxThreshold = 140)
{
	Mat step = Mat::zeros(gray.size(), CV_8UC1);

	for (size_t i = 0; i < gray.rows; i++)
	{
		for (size_t j = 0; j < gray.cols; j++)
		{
			auto temp = gray.at<uchar>(i, j);

			if (temp >= minThreshold && temp <= maxThreshold)
				step.at<uchar>(i, j) = 255;
		}
	}
	return step;
}

Mat GrayAverage3x3(Mat gray)
{
	Mat average = Mat::zeros(gray.size(), CV_8UC1);

	for (int i = 1; i < gray.rows - 1; i++)
	{
		for (int j = 1; j < gray.cols - 1; j++)
		{
			for (int q = -1; q < 2; q++)
			{
				for (int z = -1; z < 2; z++)
				{
					average.at<uchar>(i, j) += (gray.at<uchar>(i + q, j + z)) / 9;
				}
			}
		}
	}
	return average;
}

int GetSum(Mat& gray, int n, int i, int j)
{
	int sum = 0;
	auto th = ((n - 1) / 2);
	for (int q = -th; q <= th; q++)
	{
		for (int z = -th; z <= th; z++)
		{
			sum += gray.at<uchar>(i + q, j + z);
		}
	}
	return sum;
}

Mat AverageNxN(Mat gray, int n)
{
	Mat average = Mat::zeros(gray.size(), CV_8UC1);

	for (int i = 1; i < gray.rows - 1; i++)
	{
		for (int j = 1; j < gray.cols - 1; j++)
		{
			auto sum = GetSum(gray, n, i, j);
			average.at<uchar>(i, j) += sum / (n * n);
		}
	}
	return average;
}

Mat Avg(Mat Grey, int neighbourSize)
{
	Mat AvgImg = Mat::zeros(Grey.size(), CV_8UC1);
	int totalPix = pow(2 * neighbourSize + 1, 2);
	for (int i = neighbourSize; i < Grey.rows - neighbourSize; i++)
	{
		for (int j = neighbourSize; j < Grey.cols - neighbourSize; j++)
		{
			int sum = 0;
			int count = 0;
			for (int ii = -neighbourSize; ii <= neighbourSize; ii++)
			{
				for (int jj = -neighbourSize; jj <= neighbourSize; jj++)
				{
					count++;
					sum += Grey.at<uchar>(i + ii, j + jj);
				}
			}
			AvgImg.at<uchar>(i, j) = sum / count;
		}
	}

	return AvgImg;
}

Mat Blur(Mat Grey, int neighbourSize, float neighbourWeight = 1.f) {
	Mat blurImg = Mat::zeros(Grey.size(), CV_8UC1);

	int totalPix = pow(2 * neighbourSize + 1, 2);
	for (int i = neighbourSize; i < Grey.rows - neighbourSize; i++)
	{
		for (int j = neighbourSize; j < Grey.cols - neighbourSize; j++)
		{
			int sum = 0;
			int count = 0;
			for (int ii = -neighbourSize; ii <= neighbourSize; ii++)
			{
				for (int jj = -neighbourSize; jj <= neighbourSize; jj++)
				{
					count++;
					sum += (int)(Grey.at<uchar>(i + ii, j + jj) * neighbourWeight);
				}
			}
			sum -= (int)(Grey.at<uchar>(i, j) * (neighbourWeight));
			sum += (int)(Grey.at<uchar>(i, j) * (1 - neighbourWeight) * count);
			blurImg.at<uchar>(i, j) = sum / count;
		}
	}

	return blurImg;
}

Mat Max(Mat Grey, int neighbourSize)
{
	Mat img = Mat::zeros(Grey.size(), CV_8UC1);
	for (int i = neighbourSize; i < Grey.rows - neighbourSize; i++)
	{
		for (int j = neighbourSize; j < Grey.cols - neighbourSize; j++)
		{
			int max = -1;
			for (int ii = -neighbourSize; ii <= neighbourSize; ii++)
			{
				for (int jj = -neighbourSize; jj <= neighbourSize; jj++)
				{
					int pixel = Grey.at<uchar>(i + ii, j + jj);
					if (pixel > max)
						max = pixel;
				}
			}
			img.at<uchar>(i, j) = max;
		}
	}
	return img;
}

Mat Min(Mat Grey, int neighbourSize)
{
	Mat img = Mat::zeros(Grey.size(), CV_8UC1);
	for (int i = neighbourSize; i < Grey.rows - neighbourSize; i++)
	{
		for (int j = neighbourSize; j < Grey.cols - neighbourSize; j++)
		{
			int min = 255;
			for (int ii = -neighbourSize; ii <= neighbourSize; ii++)
			{
				for (int jj = -neighbourSize; jj <= neighbourSize; jj++)
				{
					int pixel = Grey.at<uchar>(i + ii, j + jj);
					if (pixel < min)
						min = pixel;
				}
			}
			img.at<uchar>(i, j) = min;
		}
	}
	return img;
}

Mat Edge(Mat Grey, int th)
{
	Mat EdgeImg = Mat::zeros(Grey.size(), CV_8UC1);
	for (int i = 1; i < Grey.rows - 1; i++)
	{
		for (int j = 1; j < Grey.cols - 1; j++)
		{
			int AvgL = (Grey.at<uchar>(i - 1, j - 1) + Grey.at<uchar>(i, j - 1) + Grey.at<uchar>(i + 1, j - 1)) / 3;
			int AvgR = (Grey.at<uchar>(i - 1, j + 1) + Grey.at<uchar>(i, j + 1) + Grey.at<uchar>(i + 1, j + 1)) / 3;
			if (abs(AvgL - AvgR) > th)
				EdgeImg.at<uchar>(i, j) = 255;


		}
	}

	return EdgeImg;


}


Mat Dialation(Mat edge, int neighbourSize)
{

	Mat dialation = Mat::zeros(edge.size(), CV_8UC1);

	for (int i = neighbourSize; i < edge.rows - neighbourSize; i++)
	{
		for (int j = neighbourSize; j < edge.cols - neighbourSize; j++)
		{
			bool shouldBreak = false;

			for (int ii = -neighbourSize; ii < neighbourSize; ii++)
			{
				for (int jj = -neighbourSize; jj < neighbourSize; jj++)
				{
					auto isNeighbourWhite = edge.at<uchar>(i + ii, j + jj) == 255;
					if (isNeighbourWhite)
					{
						dialation.at<uchar>(i, j) = 255;
						shouldBreak = true;
						break;
					}

				}

				if (shouldBreak) { break; }
			}
		}
	}

	return dialation;
}

Mat ErosionWithLimit(Mat edge, int neighbourSize)
{

	Mat erosion = Mat::zeros(edge.size(), CV_8UC1);

	for (int i = neighbourSize; i < edge.rows - neighbourSize; i++)
	{
		for (int j = neighbourSize; j < edge.cols - neighbourSize; j++)
		{
			int blackNeighbors = 0;
			erosion.at<uchar>(i, j) = edge.at<uchar>(i, j);
			bool shouldBreak = false;

			for (int ii = -neighbourSize; ii <= neighbourSize; ii++)
			{
				for (int jj = -neighbourSize; jj <= neighbourSize; jj++)
				{
					auto isNeighbourBlack = edge.at<uchar>(i + ii, j + jj) == 0;
					if (isNeighbourBlack)
					{
						blackNeighbors++;

					}
				}
			}
			if (blackNeighbors > 3)
			{
				erosion.at<uchar>(i, j) = 0;
			}
		}
	}

	return erosion;
}

Mat ErosionHomam(Mat Edge, int windowsize) {
	Mat ErodedImg = Mat::zeros(Edge.size(), CV_8UC1);
	for (int i = windowsize; i < Edge.rows - windowsize; i++) {
		for (int j = windowsize; j < Edge.cols - windowsize; j++) {
			ErodedImg.at<uchar>(i, j) = Edge.at<uchar>(i, j);
			for (int p = -windowsize; p <= windowsize; p++) {
				for (int q = -windowsize; q <= windowsize; q++) {
					if (Edge.at<uchar>(i + p, j + q) == 0) {
						ErodedImg.at<uchar>(i, j) = 0;

					}
				}
			}
		}
	}
	return ErodedImg;
}

Mat AddBorder(Mat edge, int borderX, int borderY)
{

	Mat retImg = Mat::zeros(edge.size(), CV_8UC1);


	for (int i = 0; i < edge.rows; i++)
	{
		for (int j = 0; j < edge.cols; j++)
		{

			retImg.at<uchar>(i, j) = 255;

		}
	}

	for (int i = borderY; i < edge.rows - borderY; i++)
	{
		for (int j = borderX; j < edge.cols - borderX; j++)
		{

			retImg.at<uchar>(i, j) = edge.at<uchar>(i, j);

		}
	}

	return retImg;
}


int ContrastValue(int value, float intensity = 3.5f)
{
	float f = value;
	f /= 255.f;

	f = pow(f, intensity);
	f *= 255.f;
	return (int)f;
}

Mat ContrastImg(Mat img, float intensity = 3.5f)
{
	Mat retval = Mat::zeros(img.size(), CV_8UC1);
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			int val = ContrastValue(img.at<uchar>(i, j), intensity);
			img.at<uchar>(i, j) = val;
		}
	}
	return img;
}

Mat EqHist(Mat gray)
{
	Mat eqImg = Mat::zeros(gray.size(), CV_8UC1);

	int count[256] = { 0 };
	for (size_t i = 0; i < gray.rows; i++)
	{
		for (size_t j = 0; j < gray.cols; j++)
		{
			count[gray.at<uchar>(i, j)]++;
		}
	}

	float prob[256] = { 0.0 };
	for (size_t i = 0; i < 256; i++)
	{
		prob[i] = (float)count[i] / (float)(gray.rows * gray.cols);
	}

	float accprob[256] = { 0.0 };
	accprob[0] = prob[0];
	for (size_t i = 1; i < 256; i++)
	{
		accprob[i] = prob[i] + accprob[i - 1];
	}

	float newValue[256] = { 0.0 };
	for (size_t i = 0; i < 256; i++)
	{
		newValue[i] = 255 * accprob[i];
	}

	for (size_t i = 0; i < gray.rows; i++)
	{
		for (size_t j = 0; j < gray.cols; j++)
		{
			eqImg.at<uchar>(i, j) = newValue[gray.at<uchar>(i, j)];
		}
	}
	return eqImg;
}

int OTSU(Mat Grey)
{
	int count[256] = { 0 };
	for (int i = 0; i < Grey.rows; i++)
		for (int j = 0; j < Grey.cols; j++)
			count[Grey.at<uchar>(i, j)]++;


	// prob
	float prob[256] = { 0.0 };
	for (int i = 0; i < 256; i++)
		prob[i] = (float)count[i] / (float)(Grey.rows * Grey.cols);

	// accprob
	float theta[256] = { 0.0 };
	theta[0] = prob[0];
	for (int i = 1; i < 256; i++)
		theta[i] = prob[i] + theta[i - 1];

	float meu[256] = { 0.0 };
	for (int i = 1; i < 256; i++)
		meu[i] = i * prob[i] + meu[i - 1];

	float sigma[256] = { 0.0 };
	for (int i = 0; i < 256; i++)
		sigma[i] = pow(meu[255] * theta[i] - meu[i], 2) / (theta[i] * (1 - theta[i]));

	int index = 0;
	float maxVal = 0;
	for (int i = 0; i < 256; i++)
	{
		if (sigma[i] > maxVal)
		{
			maxVal = sigma[i];
			index = i;
		}
	}

	return index + 30;
}

void showAll()
{
	vector<Mat> images;
	for (int i = 1; i < 21; i++)
	{
		images.push_back(imread(format("..\\Dataset\\%d.jpg", i)));
	}

	Mat img;
	img = imread("..\\Dataset\\8.jpg");

	auto gray = RGBToGray(img);

	for (int i = 0; i < 20; i++)
	{
		Mat image = images[i];
		image = RGBToGray(image);
		Mat avg = AverageNxN(image, 1);
		Mat edge = Edge(avg, 50);
		Mat eroded = ErosionWithLimit(edge, 1);
		Mat dilated = Dialation(eroded, 5);

		Mat DilatedImgCpy;
		DilatedImgCpy = dilated.clone();
		vector<vector<Point>> contours1;
		vector<Vec4i> hierachy1;
		findContours(dilated, contours1, hierachy1, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
		Mat dst = Mat::zeros(gray.size(), CV_8UC3);

		Rect rect;
		Mat plate;
		Scalar black = CV_RGB(0, 0, 0);
		for (int ii = 0; ii < contours1.size(); ii++)
		{
			rect = boundingRect(contours1[i]);

			auto ratio = (float)rect.width / (float)rect.height;

			auto tooTall = rect.height > 100;
			auto tooWide = rect.width < 70 || rect.width > 400;
			auto  outsideFocusX = rect.x < 0.15 * DilatedImgCpy.cols || rect.x > 0.85 * DilatedImgCpy.cols;
			auto  outsideFocusY = rect.y < 0.3 * DilatedImgCpy.rows || rect.y > 0.85 * DilatedImgCpy.rows;
			if (tooTall || tooWide || outsideFocusX || outsideFocusY || ratio < 1.5f)
			{
				drawContours(DilatedImgCpy, contours1, i, black, -1, 8, hierachy1);
			}
			else
			{
				plate = gray(rect);
			}

		}
		string title = std::to_string(i);
		imshow(title, dilated);

		title += title;
		if (plate.cols != 0 && plate.rows != 0)
			imshow(title, plate);
	}

}

float WhiteToBlackRatio(Mat image)
{
	float sum = 0;
	float pixels = image.rows * image.cols;

	for (int i = 0; i < image.rows; ++i)
	{
		for (int j = 0; j < image.cols; ++j)
		{
			int value = image.at<uchar>(i, j);
			if (value == 255)
			{
				sum++;
			}
		}
	}
	return sum / pixels;
}

Mat LocateLicensePlate(Mat image)
{

	Mat gray = RGBToGray(image);

	if (gray.cols > 1600)
	{
		Mat compressed = Mat::zeros(image.rows / 2, image.cols / 2, CV_8UC1);

		for (int i = 0, ii = 0; i < gray.rows; i += 2, ii++)
		{
			for (int j = 0, jj = 0; j < gray.cols; j += 2, jj++)
			{
				compressed.at<uchar>(ii, jj) = gray.at<uchar>(i, j);
			}
		}
		gray = compressed;
	}

	auto average = AverageNxN(gray, 1);

	auto edge = Edge(average, 50);

	auto erosion = ErosionWithLimit(edge, 1);

	auto dialation = Dialation(erosion, 5);


	Mat DilatedImgCpy;
	DilatedImgCpy = dialation.clone();
	vector<vector<Point>> contours1;
	vector<Vec4i> hierachy1;
	findContours(dialation, contours1, hierachy1, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
	Mat dst = Mat::zeros(gray.size(), CV_8UC3);


	Rect rect;
	Mat plate;
	vector<Mat> plates;
	Scalar black = CV_RGB(0, 0, 0);
	for (int i = 0; i < contours1.size(); i++)
	{
		rect = boundingRect(contours1[i]);
		float wBRatio = WhiteToBlackRatio(DilatedImgCpy(rect));

		bool tooMuchBlack = wBRatio <= 0.6f;
		auto ratio = (float)rect.width / (float)rect.height;
		auto tooTall = rect.height > 100;
		auto wrongWidth = rect.width < 52 || rect.width > 400;
		auto outsideFocusX = rect.x < 0.15 * DilatedImgCpy.cols || rect.x > 0.85 * DilatedImgCpy.cols;
		auto outsideFocusY = rect.y < 0.3 * DilatedImgCpy.rows || rect.y > 0.9 * DilatedImgCpy.rows;
		if (tooTall || wrongWidth || outsideFocusX || outsideFocusY || ratio < 1.5f || tooMuchBlack)
		{
			drawContours(DilatedImgCpy, contours1, i, black, -1, 8, hierachy1);
		}
		else
		{
			plate = gray(rect);
			plates.push_back(plate);
		}
	}

	return plate;
}


Mat UpScaleImage(Mat img)
{
	Mat reSizeImg = Mat::zeros(img.size() * 2, CV_8UC1);

	for (size_t i = 0; i < img.rows; i++)
	{
		for (size_t j = 0; j < img.cols; j++)
		{
			int ii = i * 2;
			int jj = j * 2;
			reSizeImg.at<uchar>(ii, jj) = img.at<uchar>(i, j);
			reSizeImg.at<uchar>(ii + 1, jj) = img.at<uchar>(i, j);
			reSizeImg.at<uchar>(ii, jj + 1) = img.at<uchar>(i, j);
			reSizeImg.at<uchar>(ii + 1, jj - 1) = img.at<uchar>(i, j);
		}
	}
	return reSizeImg;
}

Mat PlateOperation1(Mat plate)
{
	plate = UpScaleImage(plate);
	plate = ContrastImg(plate);
	plate = Blur(plate, 1, 0.2f);

	plate = GrayToBinary(plate);

	plate = AddBorder(plate, ((float)plate.cols * 0.02f), ((float)plate.rows * 0.05f));

	return plate;
}

Mat PlateOperation2(Mat plate)
{
	plate = GrayInversion(plate);
	plate = ErosionWithLimit(plate, 1);
	plate = ErosionWithLimit(plate, 1);

	return plate;
}

Mat PlateOperation3(Mat plate)
{
	plate = GrayInversion(plate);
	plate = ErosionHomam(plate, 1);
	plate = Dialation(plate, 1);
	plate = GrayInversion(plate);

	return plate;
}

string ProcessLicensePlate(Mat plate, int index)
{
	string str = "NO LETTERS DETECTED!";

	imshow(to_string(index + 1), plate);
	tesseract::TessBaseAPI* api = new tesseract::TessBaseAPI();

	api->Init("..\\tessdata", "eng");

	api->SetVariable("tessedit_char_whitelist", "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789");

	cvtColor(plate, plate, COLOR_BGR2BGRA);
	api->SetImage(plate.data, plate.cols, plate.rows, 4, 4 * plate.cols);

	char* outText = api->GetUTF8Text();
	if (outText != nullptr)
	{
		str = outText;
		str.erase(remove(str.begin(), str.end(), '('), str.end());
		str.erase(remove(str.begin(), str.end(), ')'), str.end());
		str.erase(remove(str.begin(), str.end(), '|'), str.end());
		str.erase(remove(str.begin(), str.end(), ' '), str.end());
		str.erase(remove(str.begin(), str.end(), '.'), str.end());
		str.erase(remove(str.begin(), str.end(), '['), str.end());
		str.erase(remove(str.begin(), str.end(), ']'), str.end());
		str.erase(remove(str.begin(), str.end(), '{'), str.end());
		str.erase(remove(str.begin(), str.end(), '}'), str.end());
		str.erase(remove(str.begin(), str.end(), '"'), str.end());
		str.erase(remove(str.begin(), str.end(), '_'), str.end());
		str.erase(remove(str.begin(), str.end(), '	'), str.end());
		str.erase(remove(str.begin(), str.end(), '\n'), str.end());
	}
	api->End();
	delete[] outText;

	return toUpperCase(str);
}

int main()
{

	vector<Mat> images;
	for (int i = 1; i < 21; i++)
	{
		images.push_back(imread(format("..\\Dataset\\%d.jpg", i)));
	}

	vector<string> plateText{
		"CBC6466",	//1
		"NAV5969",	//2
		"WA5008C",	//3
		"WCV9605",	//4
		"RK8255",	//5
		"BDF1490",	//6
		"W3701V",	//7
		"PGE523",	//8
		"WGD2542",	//9
		"WWP9229",	//10
		"BHM9492",	//11
		"BKQ9784",	//12
		"WXA2198",	//13
		"WSY8789",	//14
		"WVM757",	//15
		"WWQ3817",	//16
		"JLP911",	//17
		"A550RGY",	//18
		"WRP525",	//19
		"AHD6131"	//20
	};


	std::vector<Mat(*)(Mat)> plateOperations{ &PlateOperation1,&PlateOperation2,&PlateOperation3 };

	int correctPlatesFound = 0;
	for (size_t i = 0; i < images.size(); i++)
	{
		Mat plate = LocateLicensePlate(images[i]);
		string str = "PLATE NOT FOUND!";
		string match = " not a match!";

		if (plate.cols != 0 && plate.rows != 0)
		{
			auto count = 0;
			for (size_t j = 0; j < plateOperations.size(); j++)
			{
				plate = plateOperations[j](plate);
				str = ProcessLicensePlate(plate, i);

				if (str == plateText[i])
				{
					match = "---> FOUND MATCH";
					correctPlatesFound++;
					break;
				}
			}
		}
		cout << i + 1 << ": " << str << match << "\n" << endl;
	}
	cout << "Found " << correctPlatesFound << " /20" << endl;

	waitKey();
}