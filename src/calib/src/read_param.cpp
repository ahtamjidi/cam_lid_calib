#include <calib/read_param.h>
#include <iostream>
    
    MyData::MyData() : A(0), X(0), id()
    {}
    void MyData::write(cv::FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{" << "A" << A << "X" << X << "id" << id << "}";
    }
    void MyData::read(const cv::FileNode& node)                          //Read serialization for this class
    {
        A = (int)node["A"];
        X = (double)node["X"];
        id = (std::string)node["id"];
    }


// class MyData
// {
// public:
//     MyData() : A(0), X(0), id()
//     {}
//     explicit MyData(int) : A(97), X(CV_PI), id("mydata1234") // explicit to avoid implicit conversion
//     {}
//     void write(cv::FileStorage& fs) const                        //Write serialization for this class
//     {
//         fs << "{" << "A" << A << "X" << X << "id" << id << "}";
//     }
//     void read(const cv::FileNode& node)                          //Read serialization for this class
//     {
//         A = (int)node["A"];
//         X = (double)node["X"];
//         id = (std::string)node["id"];
//     }
// public:   // Data Members
//     int A;
//     double X;
//     std::string id;
// };

static void read(const cv::FileNode& node, MyData& x, const MyData& default_value){
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

/////////////////////////////////////////////////////
void read_param()
    {//read
        std::cout << std::endl << "Reading: " << std::endl;
        cv::FileStorage fs;
        std::string filename("asdfaf");
        fs.open(filename, cv::FileStorage::READ);
        int itNr;
        //fs["iterationNr"] >> itNr;
        itNr = (int) fs["iterationNr"];
        std::cout << itNr;
        if (!fs.isOpened())
        {
            std::cerr << "Failed to open " << filename << std::endl;
            // help(av);
            // return 1;
        }
        cv::FileNode n = fs["strings"];                         // Read string sequence - Get node
        if (n.type() != cv::FileNode::SEQ)
        {
            std::cerr << "strings is not a sequence! FAIL" << std::endl;
            // return 1;
        }
        cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
        for (; it != it_end; ++it)
            std::cout << (std::string)*it << std::endl;
        n = fs["Mapping"];                                // Read mappings from a sequence
        std::cout << "Two  " << (int)(n["Two"]) << "; ";
        std::cout << "One  " << (int)(n["One"]) << std::endl << std::endl;
        MyData m;
        cv::Mat R, T;
        fs["R"] >> R;                                      // Read cv::Mat
        fs["T"] >> T;
        fs["MyData"] >> m;                                 // Read your own structure_
        std::cout << std::endl
            << "R = " << R << std::endl;
        std::cout << "T = " << T << std::endl << std::endl;
        // std::cout << "MyData = " << endl << m << std::endl << std::endl;
        //Show default behavior for non existing nodes
        std::cout << "Attempt to read NonExisting (should initialize the data structure with its default).";
        fs["NonExisting"] >> m;
        // std::cout << std::endl << "NonExisting = " << std::endl << m << std::endl;
    }
//////////////////////////////////////////////
