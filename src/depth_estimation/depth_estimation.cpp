#include <ros/ros.h>

/* STL */
# include <vector>
/* Include opencv2 */
# include <opencv2/core/core.hpp>
# include <opencv2/features2d/features2d.hpp>
# include <opencv2/highgui/highgui.hpp>

/* Include CvBridge, Image Transport, Image msg */
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/* OpenCV Window Name */
static const std::string OPENCV_WINDOW="Depth Information";

/* Topics */
static const std::string IMAGE_TOPIC = "/vehicle/camera1/image_raw";
static const std::string PUBLISH_TOPIC = "/vehicle/camera1/depth_estimation";

class ImageConverter
{
	private:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	image_transport::Publisher image_pub;
	cv_bridge::CvImagePtr previous_cv_ptr;

	public:
	ImageConverter() : it(nh)
	{
		/* Subscribe to input video feed and publish output video feed */
		image_sub=it.subscribe(IMAGE_TOPIC, 1, &ImageConverter::imageCb, this);
		image_pub=it.advertise(PUBLISH_TOPIC, 1);
		/* Open an output window */
		ROS_INFO_STREAM("NOW OPEN CV WINDOW!");
		cv::namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void sort_matches_increasing(std::vector< cv::DMatch >& matches)
	{
		for (int i = 0; i < matches.size(); i++)
		{
			for (int j = 0; j < matches.size() - 1; j++)
			{
				if (matches[j].distance > matches[j + 1].distance)
				{
					auto temp = matches[j];
					matches[j] = matches[j + 1];
					matches[j + 1] = temp;
				}
			}
		}
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			/* Copy the image */
			cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		/* Now cv_ptr points to the image, start OpenCV part */
		if (this->previous_cv_ptr==nullptr)
		{
			/* If there is no image stored */
			this->previous_cv_ptr = cv_ptr;
			ROS_INFO_STREAM("No previous image.");
			return;
		}
		else{
			/* Step 1: Detect the keypoints using ORB Detector */
			cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();

			// Finding key points.
			std::vector< cv::KeyPoint > keypoints_base_image;
			std::vector< cv::KeyPoint > keypoints_locate_image;

			// Find keypoints.
			detector->detect(previous_cv_ptr->image, keypoints_base_image);
			detector->detect(cv_ptr->image, keypoints_locate_image);
			
			detector.release();

			// Find descriptors
			cv::Mat descriptors_base_image;
			cv::Mat descriptors_locate_image;

			cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
			extractor->compute(previous_cv_ptr->image, keypoints_base_image, descriptors_base_image);
			extractor->compute(cv_ptr->image, keypoints_locate_image, descriptors_locate_image);

			extractor.release();

			// Create Brute-Force Matcher. Other Algorithms are 'non-free'.
			cv::BFMatcher brue_force_matcher = cv::BFMatcher(cv::NORM_HAMMING, true);

			// Vector where matches will be stored.
			std::vector< cv::DMatch > matches;

			// Find matches and store in matches vector.
			brue_force_matcher.match((const cv::OutputArray)descriptors_base_image, (const cv::OutputArray)descriptors_locate_image,  matches);

			// Sort them in order of their distance
			sort_matches_increasing(matches);

			// Only  use the first 10 matches
			cv::Mat output_image;

			cv::drawMatches(
				previous_cv_ptr->image, keypoints_base_image,
				cv_ptr->image, keypoints_locate_image,
				matches,
				output_image);

			// Update cv_bridge pointer 
			this->previous_cv_ptr=cv_ptr;
			cv::imshow(OPENCV_WINDOW, output_image);
			cv::waitKey();
		}
	}
};

// Main function
int main(int argc, char** argv){
	// Initialize the ROS Node "roscpp_example"
	ros::init(argc, argv, "depth_estimation");
	ROS_INFO_STREAM("Creating Image Converter Object!");
  ImageConverter ic;
  ros::spin();
  return 0;
}
