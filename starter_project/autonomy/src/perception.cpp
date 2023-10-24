#include "perception.hpp"

// ROS Headers, ros namespace
#include <ros/init.h>
#include <cmath>

int main(int argc, char** argv) {
    ros::init(argc, argv, "starter_project_perception"); // Our node name (See: http://wiki.ros.org/Nodes)

    [[maybe_unused]] mrover::Perception perception;

    // "spin" blocks until our node dies
    // It listens for new messages and calls our subscribed functions with them
    ros::spin();

    return EXIT_SUCCESS;
}

namespace mrover {

    Perception::Perception() : mNodeHandle{}, mImageTransport{mNodeHandle} {
        // Subscribe to camera image messages
        // Every time another node publishes to this topic we will be notified
        // Specifically the callback we passed will be invoked
        mImageSubscriber = mImageTransport.subscribe("camera/right/image", 1, &Perception::imageCallback, this);

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        // TODO: uncomment me!
        mTagPublisher = mNodeHandle.advertise<StarterProjectTag>("tag", 1);

        mTagDetectorParams = cv::aruco::DetectorParameters::create();
        mTagDictionary = cv::aruco::getPredefinedDictionary(0);
    }

    void Perception::imageCallback(sensor_msgs::ImageConstPtr const& image) {
        // Create a cv::Mat from the ROS image message
        // Note this does not copy the image data, it is basically a pointer
        // Be careful if you extend its lifetime beyond this function
        cv::Mat cvImage{static_cast<int>(image->height), static_cast<int>(image->width),
                        CV_8UC3, const_cast<uint8_t*>(image->data.data())};
        // Detect tags in the image pixels
        findTagsInImage(cvImage, mTags);
        // Select the tag that is closest to the middle of the screen
        StarterProjectTag tag = selectTag(mTags);
        // Publish the message to our topic so navigation or others can receive it
        publishTag(tag);
    }

    void Perception::findTagsInImage(cv::Mat const& image, std::vector<StarterProjectTag>& tags) { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, mTagIds, and mTagDetectorParams member variables already defined!
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions
        std::vector<std::vector<cv::Point2f>> rejectedCandidates;

        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams, rejectedCandidates);
        tags.clear(); // Clear old tags in output vector

        for (size_t i=0; i < mTagCorners.size(); i++) {
            float closeness = getClosenessMetricFromTagCorners(image, mTagCorners[i]);
            std::pair<float, float> center = getCenterFromTagCorners(mTagCorners[i]);
            //StarterProjectTag data = {mTagIds[i], center.first, center.second, closeness};
            StarterProjectTag tag;
            tag.tagId = mTagIds[i];
            tag.xTagCenterPixel=center.first - (float) image.cols/2;
            tag.yTagCenterPixel=-(center.second - (float) image.rows/2);
            tag.closenessMetric=closeness;
            tags.push_back(tag);
        }
    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        float minDistance;
        float distance;
        int index = 0;
        float x = tags[0].xTagCenterPixel;
        float y = tags[0].yTagCenterPixel;
        minDistance= (x*x) + (y*y);
        for (size_t i=1; i<tags.size(); i++){
            x=tags[i].xTagCenterPixel;
            y=tags[i].yTagCenterPixel;
            distance= sqrt((x*x) + (y*y));
            if (distance<minDistance){
                index = (int) i;
            }
        }
        StarterProjectTag center;
        center = tags[index];
        
        return center;
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        // TODO: implement me!
        float xside = tagCorners[1].x - tagCorners[0].x;
        float yside = tagCorners[3].y - tagCorners[0].y;
        float tagarea = xside * yside;
        float imgarea = (float) image.rows * (float) image.cols;
        //to get height, image.rows, and to get width, image.cols
        float closeness = 1- (tagarea / imgarea); //close to 1 is far, close to 0 is close
        return closeness;
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        float xcenter = (tagCorners[1].x + tagCorners[0].x) / 2;
        float ycenter = (tagCorners[3].y + tagCorners[0].y) / 2;
        std::pair<float, float> center = {xcenter, ycenter};
        return center;
    }

} // namespace mrover
