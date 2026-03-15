/***************************************************************************************
 ***                                                                                 ***
 ***  Copyright (c) 2022, Lucid Vision Labs, Inc.                                    ***
 ***                                                                                 ***
 ***  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     ***
 ***  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       ***
 ***  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    ***
 ***  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         ***
 ***  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  ***
 ***  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  ***
 ***  SOFTWARE.                                                                      ***
 ***                                                                                 ***
 ***************************************************************************************/


// #include "stdafx.h"
#include <stdio.h>
#include "ArenaApi.h"
// #include "SaveApi.h"

#define TAB1 "  "
#define TAB2 "    "

#include <iostream>
#include <fstream>
#include <sstream> //std::stringstream

#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Helios RGB: Overlay
//    This example demonstrates color overlay over 3D image, part 3 - Overlay:
//    With the system calibrated, we can now remove the calibration target from
//    the scene and grab new images with the Helios and Triton cameras, using the
//    calibration result to find the RGB color for each 3D point measured with
//    the Helios. Based on the output of solvePnP we can project the 3D points
//    measured by the Helios onto the RGB camera image using the OpenCV function
//    projectPoints. Grab a Helios image with the GetHeliosImage()
//    function(output: xyz_mm) and a Triton RGB image with the
//    GetTritionRGBImage() function(output: triton_rgb). The following code shows
//    how to project the Helios xyz points onto the Triton image, giving a(row,
//    col) position for each 3D point. We can sample the Triton image at
//    that(row, col) position to find the 3D point’s RGB value.

// =-=-=-=-=-=-=-=-=-
// =-=- SETTINGS =-=-
// =-=-=-=-=-=-=-=-=-

// image timeout
#define TIMEOUT 200

// orientation values file name
#define FILE_NAME_IN "/root/hanyang_matching_source/src/hanyang_matching_process/src/orientation.yml"

// file name
// #define FILE_NAME_OUT "Images\\Cpp_HLTRGB_3_Overlay.ply"

// =-=-=-=-=-=-=-=-=-
// =-=- HELPERS -=-=-
// =-=-=-=-=-=-=-=-=-
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_scan(new pcl::PointCloud<pcl::PointXYZRGBA>);
std_msgs::msg::Header helios_header;
std_msgs::msg::Header img_header;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_results_publisher_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_results_publisher_;


// helper function
void getImageHLT(Arena::IDevice* pHeliosDevice, Arena::IImage** ppOutImage, cv::Mat& xyz_mm, size_t& width, size_t& height, double& xyz_scale_mm, double& x_offset_mm, double& y_offset_mm, double& z_offset_mm)
{

	// get node values that will be changed in order to return their values at
	// the end of the example
	GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pHeliosDevice->GetNodeMap(), "PixelFormat");
	GenICam::gcstring operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pHeliosDevice->GetNodeMap(), "Scan3dOperatingMode");
	GenICam::gcstring exposureTimeInitial = Arena::GetNodeValue<GenICam::gcstring>(pHeliosDevice->GetNodeMap(), "ExposureTimeSelector");
	GenICam::gcstring conversionGainInitial = Arena::GetNodeValue<GenICam::gcstring>(pHeliosDevice->GetNodeMap(), "ConversionGain");
	int64_t imageAccumulationInitial = Arena::GetNodeValue<int64_t>(pHeliosDevice->GetNodeMap(), "Scan3dImageAccumulation");
	bool spatialFilterInitial = Arena::GetNodeValue<bool>(pHeliosDevice->GetNodeMap(), "Scan3dSpatialFilterEnable");
	bool confidenceThresholdInitial = Arena::GetNodeValue<bool>(pHeliosDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable");

	// Read the scale factor and offsets to convert from unsigned 16-bit values 
	//    in the Coord3D_ABCY16 pixel format to coordinates in mm
	GenApi::INodeMap* node_map = pHeliosDevice->GetNodeMap();
	xyz_scale_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateScale");
	Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateA");
	x_offset_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");
	Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateB");
	y_offset_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");
	Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateC");
	z_offset_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");

	// set pixel format
	std::cout << TAB1 << "Set Coord3D_ABCY16 to pixel format\n";

	Arena::SetNodeValue<GenICam::gcstring>(pHeliosDevice->GetNodeMap(), "PixelFormat", "Coord3D_ABCY16");

	// set operating mode distance
	std::cout << TAB1 << "Set 3D operating mode to Distance3000mm\n";
	Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dOperatingMode", "Distance3000mmSingleFreq");

	// set exposure time
	std::cout << TAB1 << "Set time selector to Exp1000Us\n";

	Arena::SetNodeValue<GenICam::gcstring>(pHeliosDevice->GetNodeMap(), "ExposureTimeSelector", "Exp1000Us");

	// set gain
	std::cout << TAB1 << "Set conversion gain to low\n";

	Arena::SetNodeValue<GenICam::gcstring>(pHeliosDevice->GetNodeMap(), "ConversionGain", "Low");

	// Set image accumulation
	std::cout << TAB1 << "Set image accumulation to 4\n";

	Arena::SetNodeValue<int64_t>(pHeliosDevice->GetNodeMap(), "Scan3dImageAccumulation", 4);

	// Enable spatial filter
	std::cout << TAB1 << "Enable spatial filter\n";

	Arena::SetNodeValue<bool>(pHeliosDevice->GetNodeMap(), "Scan3dSpatialFilterEnable", true);

	// Enable confidence threshold
	std::cout << TAB1 << "Enable confidence threshold\n";

	Arena::SetNodeValue<bool>(pHeliosDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable", true);

	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(pHeliosDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

	// enable stream packet resend
	Arena::SetNodeValue<bool>(pHeliosDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);


	pHeliosDevice->StartStream();
	Arena::IImage* pHeliosImage = pHeliosDevice->GetImage(2000);

	// copy image because original will be delited after function call
	Arena::IImage* pCopyImage = Arena::ImageFactory::Copy(pHeliosImage);
	*ppOutImage = pCopyImage;

	width = pHeliosImage->GetWidth();
	height = pHeliosImage->GetHeight();

	xyz_mm = cv::Mat((int)height, (int)width, CV_32FC3);

	const uint16_t* input_data = reinterpret_cast<const uint16_t*>(pHeliosImage->GetData());

	cloud_scan->clear();
    cloud_scan->height = height;
    cloud_scan->width = width;
    helios_header.stamp.sec =
        static_cast<uint32_t>(pHeliosImage->GetTimestampNs() / 1000000000);
    helios_header.stamp.nanosec =
        static_cast<uint32_t>(pHeliosImage->GetTimestampNs() % 1000000000);
    // helios_header.frame_id = std::to_string(pHeliosImage->GetFrameId());
	helios_header.frame_id = "map";

	for (unsigned int ir = 0; ir < height; ++ir)
	{
		for (unsigned int ic = 0; ic < width; ++ic)
		{
			// Get unsigned 16 bit values for X,Y,Z coordinates
			ushort x_u16 = input_data[0];
			ushort y_u16 = input_data[1];
			ushort z_u16 = input_data[2];
			pcl::PointXYZRGBA point;

			// Convert 16-bit X,Y,Z to float values in mm
			xyz_mm.at<cv::Vec3f>(ir, ic)[0] = (float)(x_u16 * xyz_scale_mm + x_offset_mm);
			xyz_mm.at<cv::Vec3f>(ir, ic)[1] = (float)(y_u16 * xyz_scale_mm + y_offset_mm);
			xyz_mm.at<cv::Vec3f>(ir, ic)[2] = (float)(z_u16 * xyz_scale_mm + z_offset_mm);
			point.x = (float)((x_u16 * xyz_scale_mm + x_offset_mm));
            point.y = (float)((y_u16 * xyz_scale_mm + y_offset_mm)); 
            point.z = (float)((z_u16 * xyz_scale_mm + z_offset_mm)); 
            cloud_scan -> points.push_back(point);
			input_data += 4;
		}
	}
	pHeliosDevice->RequeueBuffer(pHeliosImage);
	pHeliosDevice->StopStream();

	// return nodes to their initial values
	Arena::SetNodeValue<bool>(pHeliosDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable", confidenceThresholdInitial);
	Arena::SetNodeValue<bool>(pHeliosDevice->GetNodeMap(), "Scan3dSpatialFilterEnable", spatialFilterInitial);
	Arena::SetNodeValue<int64_t>(pHeliosDevice->GetNodeMap(), "Scan3dImageAccumulation", imageAccumulationInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pHeliosDevice->GetNodeMap(), "ConversionGain", conversionGainInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pHeliosDevice->GetNodeMap(), "ExposureTimeSelector", exposureTimeInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pHeliosDevice->GetNodeMap(), "Scan3dOperatingMode", operatingModeInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pHeliosDevice->GetNodeMap(), "PixelFormat", pixelFormatInitial);
	std::cout << TAB1 << "Nodes were set back to initial values\n";

}


void getImageTRI(Arena::IDevice* pDeviceTriton, Arena::IImage** ppOutImage, cv::Mat& triton_rgb)
{
	#if defined(_WIN32)
		Arena::SetNodeValue<GenICam::gcstring>(pDeviceTriton->GetNodeMap(), "PixelFormat", "RGB8");
	#elif defined(__linux__)
		Arena::SetNodeValue<GenICam::gcstring>(pDeviceTriton->GetNodeMap(), "PixelFormat", "BGR8");
	#endif

	pDeviceTriton->StartStream();
	Arena::IImage* pImage = pDeviceTriton->GetImage(2000);

	// copy image because original will be delited after function call
	Arena::IImage* pCopyImage = Arena::ImageFactory::Copy(pImage);
	*ppOutImage = pCopyImage;

	size_t triHeight, triWidth;
	triHeight = pImage->GetHeight();
	triWidth = pImage->GetWidth();

	img_header.stamp.sec =
        static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
    img_header.stamp.nanosec =
        static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
    img_header.frame_id = std::to_string(pImage->GetFrameId());

	triton_rgb = cv::Mat((int)triHeight, (int)triWidth, CV_8UC3);
	memcpy(triton_rgb.data, pImage->GetData(), triHeight * triWidth * 3);

	pCopyImage = NULL;
	delete pCopyImage;

	pDeviceTriton->RequeueBuffer(pImage);
	pDeviceTriton->StopStream();
}


void OverlayColorOnto3DAndSave(Arena::IDevice* pDeviceTRI, Arena::IDevice* pDeviceHLT)
{
	// get node values that will be changed in order to return their values at
	// the end of the example
	GenICam::gcstring pixelFormatInitialTRI = Arena::GetNodeValue<GenICam::gcstring>(pDeviceTRI->GetNodeMap(), "PixelFormat");
	GenICam::gcstring pixelFormatInitialHLT = Arena::GetNodeValue<GenICam::gcstring>(pDeviceHLT->GetNodeMap(), "PixelFormat");

	// Read in camera matrix, distance coefficients, and rotation and translation vectors
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::Mat rotationVector;
	cv::Mat translationVector;

	cv::FileStorage fs(FILE_NAME_IN, cv::FileStorage::READ);
	
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;
	fs["rotationVector"] >> rotationVector;
	fs["translationVector"] >> translationVector;

	fs.release();

	// Get an image from Helios 2
	std::cout << TAB1 << "Get and prepare HLT image\n";

	Arena::IImage* pImageHLT = nullptr;
	cv::Mat imageMatrixXYZ;
	size_t width = 0;
	size_t height = 0;
	double scale;
	double offsetX, offsetY, offsetZ;

	getImageHLT(
		pDeviceHLT, 
		&pImageHLT, 
		imageMatrixXYZ, 
		width, 
		height, 
		scale, 
		offsetX, 
		offsetY, 
		offsetZ);

	// cv::imwrite(FILE_NAME_OUT "XYZ.jpg", imageMatrixXYZ);

	// Get an image from Triton
	std::cout << TAB1 << "Get and prepare TRI image\n";

	Arena::IImage* pImageTRI = nullptr;
	cv::Mat imageMatrixRGB;

	getImageTRI(
		pDeviceTRI,
		&pImageTRI,
		imageMatrixRGB);

	// cv::imwrite(FILE_NAME_OUT "RGB.jpg", imageMatrixRGB);

	// Overlay RGB color data onto 3D XYZ points
	std::cout << TAB1 << "Overlay the RGB color data onto the 3D XYZ points\n";

	// reshape image matrix
	std::cout << TAB2 << "Reshape XYZ matrix\n";
	
	int size = imageMatrixXYZ.rows * imageMatrixXYZ.cols;
	cv::Mat xyzPoints = imageMatrixXYZ.reshape(3, size);
	
	// project points
	std::cout << TAB2 << "Project points\n";

	cv::Mat projectedPointsTRI;
	
	cv::projectPoints(
		xyzPoints, 
		rotationVector, 
		translationVector, 
		cameraMatrix, 
		distCoeffs, 
		projectedPointsTRI);

	// loop through projected points to access RGB data at those points
	std::cout << TAB2 << "Get values at projected points\n";

	uint8_t* pColorData = new uint8_t[width * height * 3];

	std::cout<< TAB1 << "RGB Image Size: " <<"ROWS: "<< imageMatrixXYZ.size<<"\n";

	cv::Mat resizedrgb = cv::Mat(480, 640, CV_32FC3);
	int rowindx = 0;
	int colindx = -1;

	for (int i = 0; i < width * height; i++)
	{
		unsigned int colTRI = (unsigned int)std::round(projectedPointsTRI.at<cv::Vec2f>(i)[0]);
		unsigned int rowTRI = (unsigned int)std::round(projectedPointsTRI.at<cv::Vec2f>(i)[1]);

		// only handle appropriate points
		if (rowTRI < 0 ||
			colTRI < 0 ||
			rowTRI >= static_cast<unsigned int>(imageMatrixRGB.rows) ||
			colTRI >= static_cast<unsigned int>(imageMatrixRGB.cols)){
			uchar R = 255;
			uchar G = 255;
			uchar B = 255;
			if(colindx>=639){
				colindx = -1;		
				rowindx++;	
			}
			colindx++;
			//std::cout<<"RESIZEDRGB: ("<<rowindx<<", "<<colindx<<") "<<(float)(B) << (float)(G)<< (float)(R)<<"out\n";
			resizedrgb.at<cv::Vec3f>(rowindx, colindx)[0] = (float)(R);
			resizedrgb.at<cv::Vec3f>(rowindx, colindx)[1] = (float)(G);
			resizedrgb.at<cv::Vec3f>(rowindx, colindx)[2] = (float)(B);

			pColorData[i * 3 + 0] = B;
			pColorData[i * 3 + 1] = G;
			pColorData[i * 3 + 2] = R;

			continue;
			}

		// access corresponding XYZ and RGB data
		uchar R = imageMatrixRGB.at<cv::Vec3b>(rowTRI, colTRI)[0];
		uchar G = imageMatrixRGB.at<cv::Vec3b>(rowTRI, colTRI)[1];
		uchar B = imageMatrixRGB.at<cv::Vec3b>(rowTRI, colTRI)[2];
		
		float X = imageMatrixXYZ.at<cv::Vec3f>(i)[0];
		float Y = imageMatrixXYZ.at<cv::Vec3f>(i)[1];
		float Z = imageMatrixXYZ.at<cv::Vec3f>(i)[2];

		// grab RGB data to save colored .ply
		pColorData[i * 3 + 0] = B;
		pColorData[i * 3 + 1] = G;
		pColorData[i * 3 + 2] = R;

		if(colindx>=639){
			colindx = -1;		
			rowindx++;	
		}
		colindx++;
		//std::cout<<"RESIZEDRGB: ("<<rowindx<<", "<<colindx<<") "<<(float)(B) << (float)(G)<< (float)(R)<<"\n";
		resizedrgb.at<cv::Vec3f>(rowindx, colindx)[0] = (float)(R);
		resizedrgb.at<cv::Vec3f>(rowindx, colindx)[1] = (float)(G);
		resizedrgb.at<cv::Vec3f>(rowindx, colindx)[2] = (float)(B);				

	}
	cv::Mat newimg;
	sensor_msgs::msg::PointCloud2 cloud_msg;

	resizedrgb.convertTo(newimg, CV_8UC3, 1.0, 0.0);
	sensor_msgs::msg::Image::ConstPtr pub_img = cv_bridge::CvImage(img_header, "bgr8", newimg).toImageMsg();
	image_results_publisher_->publish(*pub_img);
	std::cout << TAB1 << " Resized Image size: "  << resizedrgb.size<<"\n";
	usleep(1000);
	pcl::toROSMsg(*cloud_scan, cloud_msg);
    cloud_msg.header = helios_header;
    point_results_publisher_->publish(cloud_msg);
	image_results_publisher_->publish(*pub_img);
	
	// cv::imwrite(FILE_NAME_OUT "RESIZEDRGB.jpg", resizedrgb);

	// Save result
	// std::cout << TAB1 << "Save image to " << FILE_NAME_OUT << "\n";

/*****************************************************************
	// prepare to save
	Save::ImageParams params(
		pImageHLT->GetWidth(),
		pImageHLT->GetHeight(),
		pImageHLT->GetBitsPerPixel());

	Save::ImageWriter plyWriter(
		params,
		FILE_NAME_OUT);

	// save .ply with color data
	bool filterPoints = true;
	bool isSignedPixelFormat = false;

	plyWriter.SetPly(
		".ply", 
		filterPoints, 
		isSignedPixelFormat, 
		scale, 
		offsetX, 
		offsetY, 
		offsetZ);

	plyWriter.Save(pImageHLT->GetData(), pColorData);
*******************************************************************/

	// return nodes to their initial values
	Arena::SetNodeValue<GenICam::gcstring>(pDeviceTRI->GetNodeMap(), "PixelFormat", pixelFormatInitialTRI);
	Arena::SetNodeValue<GenICam::gcstring>(pDeviceHLT->GetNodeMap(), "PixelFormat", pixelFormatInitialHLT);

	// clean up
	pColorData = NULL;
	delete[] pColorData;
	Arena::ImageFactory::Destroy(pImageHLT);
	Arena::ImageFactory::Destroy(pImageTRI);
}

// =-=-=-=-=-=-=-=-=-
// =- PREPARATION -=-
// =- & CLEAN UP =-=-
// =-=-=-=-=-=-=-=-=-

bool isApplicableDeviceTriton(Arena::DeviceInfo deviceInfo)
{
	// color triton camera needed
	return ((deviceInfo.ModelName().find("PHX") != GenICam::gcstring::npos) && (deviceInfo.ModelName().find("-C") != GenICam::gcstring::npos));
}

bool isApplicableDeviceHelios2(Arena::DeviceInfo deviceInfo)
{
	return ((deviceInfo.ModelName().find("HLT") != GenICam::gcstring::npos) || (deviceInfo.ModelName().find("HTP") != GenICam::gcstring::npos) \
		|| (deviceInfo.ModelName().find("HTW") != GenICam::gcstring::npos));
}

// void convert_pointcloud_msg(Arena::IImage* pImageHLT, uint8_t* pColorData, sensor_msgs::msg::PointCloud2& cloud_rgb_msg)
// {
// 	cloud_rgb_msg.header.stamp.sec =
//         static_cast<uint32_t>(pImageHLT->GetTimestampNs() / 1000000000);
//     cloud_rgb_msg.header.stamp.nanosec =
//         static_cast<uint32_t>(pImageHLT->GetTimestampNs() % 1000000000);
//     cloud_rgb_msg.header.frame_id = std::to_string(pImageHLT->GetFrameId());

// 	cloud_rgb_msg.height = pImageHLT->GetHeight();
// 	cloud_rgb_msg.width = pImageHLT->GetWidth();
// 	cloud_rgb_msg.is_bigendian = pImageHLT->GetPixelEndianness() ==
//                              	 Arena::EPixelEndianness::PixelEndiannessBig;
// }





int main(int argc, char **argv)
{
	// flag to track when an exception has been thrown
	bool exceptionThrown = false;

	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("helios_publishing_node");

	std::cout << "Cpp_HLTRGB_3_Overlay\n";
	image_results_publisher_ = node->create_publisher<sensor_msgs::msg::Image>("/helios_pub/resizedimages",1);
	point_results_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/helios_pub/points",1);

	try
	{
		std::ifstream ifile;
		ifile.open(FILE_NAME_IN);
		if (!ifile)
		{
			std::cout << "File '" << FILE_NAME_IN << "' not found\nPlease run examples 'Cpp_HLTRGB_1_Calibration' and 'Cpp_HLTRGB_2_Orientation' prior to this one\nPress enter to complete\n";
			std::getchar();
			return 0;
		}

		// prepare example
		Arena::ISystem* pSystem = Arena::OpenSystem();
		pSystem->UpdateDevices(100);
		std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
		if (deviceInfos.size() == 0)
		{
			std::cout << "\nNo camera connected\nPress enter to complete\n";
			std::getchar();
			return 0;
		}

		Arena::IDevice* pDeviceTRI = nullptr;
		Arena::IDevice* pDeviceHLT = nullptr;
		for (auto& deviceInfo : deviceInfos)
		{
			if (!pDeviceTRI && isApplicableDeviceTriton(deviceInfo))
			{
				pDeviceTRI = pSystem->CreateDevice(deviceInfo);

				// enable stream auto negotiate packet size
				Arena::SetNodeValue<bool>(
					pDeviceTRI->GetTLStreamNodeMap(),
					"StreamAutoNegotiatePacketSize",
					true);

				// enable stream packet resend
				Arena::SetNodeValue<bool>(
					pDeviceTRI->GetTLStreamNodeMap(),
					"StreamPacketResendEnable",
					true);
			}
			else if (isApplicableDeviceTriton(deviceInfo))
			{
				throw std::logic_error("too many Triton devices connected");
			}
			else if (!pDeviceHLT && isApplicableDeviceHelios2(deviceInfo))
			{
				pDeviceHLT = pSystem->CreateDevice(deviceInfo);

				// enable stream auto negotiate packet size
				Arena::SetNodeValue<bool>(
					pDeviceHLT->GetTLStreamNodeMap(),
					"StreamAutoNegotiatePacketSize",
					true);

				// enable stream packet resend
				Arena::SetNodeValue<bool>(
					pDeviceHLT->GetTLStreamNodeMap(),
					"StreamPacketResendEnable",
					true);
			}
			else if (isApplicableDeviceHelios2(deviceInfo))
			{
				throw std::logic_error("too many Helios 2 devices connected");
			}
		}

		if (!pDeviceTRI)
			throw std::logic_error("No applicable Triton devices");

		if (!pDeviceHLT)
			throw std::logic_error("No applicable Helios 2 devices");

		// run example
		if (pDeviceTRI && pDeviceHLT)
		{
			std::cout << "Commence example\n\n";
			OverlayColorOnto3DAndSave(pDeviceTRI, pDeviceHLT);
			std::cout << "\nExample complete\n";
		}

		if (pDeviceTRI)
			pSystem->DestroyDevice(pDeviceTRI);
		if (pDeviceHLT)
			pSystem->DestroyDevice(pDeviceHLT);

		Arena::CloseSystem(pSystem);
	}
	catch (GenICam::GenericException& ge)
	{
		std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
		exceptionThrown = true;
	}
	catch (std::exception& ex)
	{
		std::cout << "\nStandard exception thrown: " << ex.what() << "\n";
		exceptionThrown = true;
	}
	catch (...)
	{
		std::cout << "\nUnexpected exception thrown\n";
		exceptionThrown = true;
	}
	rclcpp::shutdown();
	if (exceptionThrown)
		return -1;
	else
		return 0;
}
