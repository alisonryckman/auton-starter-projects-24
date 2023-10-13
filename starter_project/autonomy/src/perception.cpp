#include "perception.hpp"
#include "mrover/StarterProjectTag.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "std_msgs/String.h"

// ROS Headers, ros namespace
#include <opencv2/core/types.hpp>
#include <ros/init.h>
#include <sstream>

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

        tags.clear(); // Clear old tags in output vector

        //get help

        
        cv::aruco::detectMarkers(image,mTagDictionary,mTagCorners,mTagIds,mTagDetectorParams);

        

        for (int i = 0; i < mTagIds.size(); i++) {
            StarterProjectTag tag;
            tag.tagId = mTagIds[i];
            std::pair<float,float> center = getCenterFromTagCorners(mTagCorners[i]);
            //express each tag center to be relative to the center of the image
            tag.xTagCenterPixel = (center.first - (float)image.cols/2) / (float)image.cols;
            tag.yTagCenterPixel = (center.second - (float)image.cols/2) / (float)image.cols;
            tag.closenessMetric = getClosenessMetricFromTagCorners(image, mTagCorners[i]);
            tags.push_back(tag);
        }

    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        //when closeness metric is closer to zero, closer to camera
        if (!tags.empty()) {
            if (tags.size() == 1) {
                // if only one tag then return it
                return tags[0];
            }
            int min_index = 0;
            double min_distance = sqrt(pow(tags[0].xTagCenterPixel,2) + pow(tags[0].yTagCenterPixel,2));

            for (int i = 0; i < tags.size(); i++) {
                if (min_distance > (sqrt(pow(tags[i].xTagCenterPixel,2) + pow(tags[i].yTagCenterPixel,2)))) {

                    min_distance = sqrt(pow(tags[i].xTagCenterPixel,2) + pow(tags[i].yTagCenterPixel,2));
                    min_index = i;

                }
            }

            return tags[min_index];

        } else {
            // if no tag is seen, return with a false (-1) indicator
            StarterProjectTag tag;
            tag.tagId = -1;
            return tag;
        }
        
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        // TODO: implement me!
        
        //mTagPublisher
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        // TODO: implement me!
        float imageSize = (float)image.cols * (float)image.rows;

        //find three corners of tag to calculate area from
        cv::Point2f topLeft = tagCorners[0];
        cv::Point2f topRight = tagCorners[1];
        cv::Point2f bottomLeft = tagCorners[2];

        //calculate area of tag
        float tagWidth = topRight.x - topLeft.x;
        float tagHeight = bottomLeft.y - topLeft.y;
        ROS_INFO("tag width: %f tag height: %f", tagWidth,tagHeight);

        //metric is the ratio between the tag area and total image area
        float metric = abs(tagWidth * tagHeight) / (float)imageSize;
        //metric goes from 0 to 1 where 0 means really close


        return 1 - metric;
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        int xSum = 0;
        int ySum = 0;

        // the center is the sum of the x and y coordinates of the corners
        for (cv::Point2f corner : tagCorners) {
            xSum += (int)corner.x;
            ySum += (int)corner.y;
        }
        std::pair<float,float> center(xSum,ySum);

        return center;
    }

} // namespace mrover
