#ifndef _READ_YAML_PARAM_
#define _READ_YAML_PARAM_

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <string>

class MyData
{
    public:
        MyData();
        // explicit MyData::MyData(int) : A(97), X(CV_PI), id("mydata1234") // explicit to avoid implicit conversion
        // {}
        void write(cv::FileStorage& fs) const;                        //Write serialization for this class
        void read(const cv::FileNode& node);                          //Read serialization for this class
    public:   // Data Members
        int A;
        double X;
        std::string id;
};

static void read(const cv::FileNode& node, MyData& x, const MyData& default_value = MyData());

void read_param();

#endif
