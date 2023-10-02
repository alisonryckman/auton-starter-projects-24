#include "perception.hpp"

// Boost headers, boost namespace
// Boost attempts to fill in the gaps in the C++ standard library
// Use sparingly - only when the C++ standard library is insufficient
#include <boost/range/combine.hpp>

#include <opencv2/imgproc.hpp>

// ROS Headers, ros namespace
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

        // I only did this since I have OpenCV 4.8.1 on my machine not 4.2.0, they broke some API compatibility...
        mTagDetectorParams = cv::makePtr<cv::aruco::DetectorParameters>();
        mTagDictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(0));
    }

    void Perception::imageCallback(sensor_msgs::ImageConstPtr const& message) {
        // Create a cv::Mat from the ROS image message
        // Note this does not copy the image data, it is basically a pointer
        // Be careful if you extend its lifetime beyond this function
        cv::Mat image{static_cast<int>(message->height), static_cast<int>(message->width),
                      CV_8UC3, const_cast<uint8_t*>(message->data.data())};

        // Detect tags in the image pixels
        findTagsInImage(image, mTags);

        // Select the tag that is closest to the middle of the screen
        std::optional<StarterProjectTag> tag = selectTag(image.size(), mTags);
        // Can convert an optional to a bool to check if it has a value
        if (!tag) return;

        // Get the underlying value from the optional since we confirmed it exists
        publishTag(tag.value());
    }

    void Perception::findTagsInImage(cv::Mat const& image, std::vector<StarterProjectTag>& tags) { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, mTagIds, and mTagDetectorParams member variables already defined!
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector

        // Inputs are image, mTagDictionary, mTagDetectorParams
        // Outputs are mTagCorners, mTagIds
        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams);

        // Want to use std::ranges::views::zip(mTagIds, mTagCorners) instead of boost::combine,
        // but this fails with libstdc++13 and clang-16, see: https://github.com/llvm/llvm-project/issues/49620
        int id{};
        std::vector<cv::Point2f> corners;
        for (auto const& tuple: boost::combine(mTagIds, mTagCorners)) {
            boost::tie(id, corners) = tuple;

            auto [x, y] = getCenterFromTagCorners(corners);
            float closenessMetric = getClosenessMetricFromTagCorners(image, corners);

            StarterProjectTag tag;
            tag.tagId = id;
            tag.xTagCenterPixel = x;
            tag.yTagCenterPixel = y;
            tag.closenessMetric = closenessMetric;
            // Ideally we would use tags.emplace_back here,
            // this would call the constructor directly in the vector's memory,
            // however ROS does not provide us with a constructor!
            tags.push_back(tag);
        }
    }

    // I modified this function signature.
    // We need a size so we can find the middle of the image.
    // Return an optional as when there are zero tags there is no notion of "selecting."
    std::optional<StarterProjectTag> Perception::selectTag(cv::Size const& imageSize, std::vector<StarterProjectTag> const& tags) { // NOLINT(*-convert-member-functions-to-static)
        if (tags.empty()) return std::nullopt;

        // See: https://en.cppreference.com/w/cpp/algorithm/ranges/min_element
        // The lambda argument first "projects" each element (tag), meaning it transforms it into a new type.
        // In this case that is a double representing the distance from the image center to the tag center.
        // Then we use std::less to compare the projected elements to find the minimum.
        // Finally, we dereference the iterator since we know at least one element exists.
        auto imageCenter = static_cast<cv::Point2f>(imageSize / 2);
        return *std::ranges::min_element(tags, std::less{}, [&](StarterProjectTag const& tag) {
            cv::Point2f tagCenter{tag.xTagCenterPixel, tag.yTagCenterPixel};
            // The "norm" is the magnitude of the vector from the image center to the tag center.
            // This is the same as the distance between the two points.
            // You can confirm this by writing out the equations.
            return cv::norm(imageCenter - tagCenter);
        });
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        // When in doubt do not reinvent the wheel!
        // Explore all the functions that libraries have to offer... https://docs.opencv.org/4.2.0/dd/d49/tutorial_py_contour_features.html
        // A "contour" is a set of points that form a shape. Such as our tag corners.
        double tagArea = cv::contourArea(tagCorners);
        double imageArea = image.size().area();
        // Idea: How much of the screen a tag takes up is a good approximation for how close we are to the tag
        return static_cast<float>(tagArea / imageArea);
    }

    // I modified this to return a OpenCV point
    cv::Point2f Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // See: https://docs.opencv.org/4.2.0/d8/d23/classcv_1_1Moments.html
        cv::Moments moments = cv::moments(tagCorners);
        auto centerOfMassX = static_cast<float>(moments.m10 / moments.m00);
        auto centerOfMassY = static_cast<float>(moments.m01 / moments.m00);
        return {centerOfMassX, centerOfMassY};

        // ALTERNATIVE SOLUTION
        // This finds the average of all the corners which is roughly the center.
        // std::reduce finds the sum, and then we divide by the count.
        return std::reduce(tagCorners.begin(), tagCorners.end()) / static_cast<float>(tagCorners.size());
    }

} // namespace mrover
