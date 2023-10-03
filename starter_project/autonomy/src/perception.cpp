#include "perception.hpp"
#include "mrover/StarterProjectTag.h"

// ROS Headers, ros namespace
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <ros/init.h>

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

        tags.clear(); // Clear old tags in output vector

        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams); // populate tag data from OpenCV method

        StarterProjectTag currentTag; // current tag. we're going to assign values to this tag and then append it to the tags vector
        for (int i = 0; i < static_cast<int>(mTagIds.size()); i++) {
            std::pair<float, float> centerPoint = getCenterFromTagCorners(mTagCorners[i]); // center location of current tag
            currentTag.tagId = mTagIds[i];
            currentTag.xTagCenterPixel = centerPoint.first - (static_cast<float>(image.rows) / 2);
            currentTag.yTagCenterPixel = centerPoint.second - (static_cast<float>(image.cols) / 2);
            currentTag.closenessMetric = getClosenessMetricFromTagCorners(image, mTagCorners[i]);
            tags.push_back(currentTag); // append current tag to vector of all tags
        }
    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) {         // NOLINT(*-convert-member-functions-to-static)
        double minDist = sqrt(pow(tags[0].xTagCenterPixel, 2) + pow(tags[0].yTagCenterPixel, 2)); // we need an initial minimum to iterate against;
                                                                                                  // let this be the first tag in the vector

        double currDist;      // initialize current distance vector
        int bestTagIndex = 0; // initialize best tag index variable
        for (int i = 1; i < static_cast<int>(tags.size()); i++) {

            currDist = sqrt(pow(tags[i].xTagCenterPixel, 2) + pow(tags[i].yTagCenterPixel, 2)); // compute euclidean distance to current tag center from middle of camera image

            // if our current tag's euclidean distance from the center of the camera view is less than the previous minimum distance, set it as the new minimum and update the best index
            if (currDist < minDist) {
                minDist = currDist;
                bestTagIndex = i;
            }
        }

        return {tags[bestTagIndex]}; // use best index variable to return best tag
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        // boiler plate publish code, see http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        mTagPublisher.publish(tag);
        // report center point and closeness metric for selected tag
        ROS_INFO("Center Point X Coordinate: %f", tag.xTagCenterPixel);
        ROS_INFO("Center Point Y Coordinate: %f", tag.yTagCenterPixel);
        ROS_INFO("Closeness Metric: %f", tag.closenessMetric);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)

        // getting height of image
        int height = image.rows;
        int width = image.cols;
        auto imArea = static_cast<float>(height * width);

        // computing max and minimum x and y coordinates
        // approximates tag as a rectangle
        float xMax = tagCorners[1].x;
        float yMax = tagCorners[2].y;
        float xMin = tagCorners[0].x;
        float yMin = tagCorners[0].y;
        float tagArea = (xMax - xMin) * (yMax - yMin);

        // closeness is tag area divided by image area. this is pretty small for practicable distances, so it could be scaled by some factor to make the values more human-tractable, but
        // it's not strictly necessary
        float closeness = tagArea / imArea;

        return closeness;
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // initialize variables
        float xCent = 0;
        float yCent = 0;
        auto numPoints = static_cast<float>(tagCorners.size()); // number of points

        // compute sum of x-components and y-components
        for (int i = 0; i < static_cast<int>(numPoints); i++) {
            xCent += tagCorners[i].x;
            yCent += tagCorners[i].y;
        }

        // compute centroid coordinates by dividing by number of points (averaging)
        xCent /= numPoints;
        yCent /= numPoints;

        // generating pair
        std::pair<float, float> centerPoint = {xCent, yCent};
        return centerPoint;
    }

} // namespace mrover
