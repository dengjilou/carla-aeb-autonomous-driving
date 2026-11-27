#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <Eigen/Geometry>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Mat = Eigen::Matrix4f;
namespace fs = std::filesystem;

Cloud::Ptr read_scan(const fs::path& path, float voxel_size) {
    std::ifstream input(path, std::ios::binary);
    if (!input || fs::file_size(path) % 16) throw std::runtime_error("Invalid scan " + path.string());
    Cloud::Ptr raw(new Cloud), filtered(new Cloud);
    float xyzi[4];
    while (input.read(reinterpret_cast<char*>(xyzi), sizeof(xyzi))) {
        const float r2 = xyzi[0]*xyzi[0] + xyzi[1]*xyzi[1];
        if (r2 >= 4 && r2 <= 40*40 && xyzi[2] > -2.5f && xyzi[2] < 4)
            raw->emplace_back(xyzi[0], xyzi[1], xyzi[2]);
    }
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setLeafSize(voxel_size,voxel_size,voxel_size);
    vox.setInputCloud(raw);
    vox.filter(*filtered);
    if (filtered->size()<30) throw std::runtime_error("Too few points in " + path.string());
    return filtered;
}

struct Match { Mat prev_from_curr; bool ok; double fitness; };
Match match(Cloud::ConstPtr curr, Cloud::ConstPtr prev, const Mat& guess) {
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
    icp.setInputSource(curr);
    icp.setInputTarget(prev);
    icp.setMaxCorrespondenceDistance(2.5);
    icp.setMaximumIterations(25);
    icp.setTransformationEpsilon(1e-7);
    icp.setEuclideanFitnessEpsilon(1e-5);
    Cloud aligned;
    icp.align(aligned,guess);
    return {icp.getFinalTransformation(),icp.hasConverged(),icp.getFitnessScore()};
}

int main(int argc, char** argv) try {
    if (argc==2 && std::string(argv[1])=="--self-test") {
        Cloud::Ptr curr(new Cloud), prev(new Cloud);
        std::mt19937 gen(7);
        std::uniform_real_distribution<float> coord(-4,4);
        for(int i=0;i<500;++i) curr->emplace_back(coord(gen),coord(gen),coord(gen));
        Eigen::Affine3f known = Eigen::Translation3f(.2f,-.15f,.1f) *
                                  Eigen::AngleAxisf(.05f,Eigen::Vector3f::UnitZ());
        pcl::transformPointCloud(*curr,*prev,known.matrix());
        const auto est=match(curr,prev,Mat::Identity());
        const float err=(est.prev_from_curr-known.matrix()).cwiseAbs().maxCoeff();
        std::cout<<"T_previous_from_current max element error = "<<err<<"\n";
        return est.ok && err<1e-4 ? 0:1;
    }
    const bool segment=argc==8 && std::string(argv[4])=="--segment";
    if(argc<4 || (argc>6 && !segment)) throw std::runtime_error(
        "odometry SCANS identity|motion OUTPUT.csv [frame_count [--wait-for-download] | --segment first last voxel_m]");
    const fs::path scans(argv[1]);
    const std::string mode(argv[2]);
    if (mode!="identity" && mode!="motion") throw std::runtime_error("Unknown initial guess");
    const bool await_download=argc==6 && std::string(argv[5])=="--wait-for-download";
    if(argc==6 && !await_download) throw std::runtime_error("Unknown download option");
    const int first=segment?std::stoi(argv[5]):0;
    const float voxel_size=segment?std::stof(argv[7]):1.f;
    if(first<0 || !std::isfinite(voxel_size) || voxel_size<=0) throw std::runtime_error("Invalid segment settings");
    int n=segment?std::stoi(argv[6])-first+1:(argc>=5?std::stoi(argv[4]):0);
    if(argc==4) for(auto& file:fs::directory_iterator(scans)) n+=file.path().extension()==".bin";
    if(n<2) throw std::runtime_error("Expected at least two scans");
    std::ofstream out(argv[3]);
    out.exceptions(std::ios::failbit|std::ios::badbit);
    out<<"frame,points,converged,registration_ms,frame_ms,fitness";
    for(int r=0;r<3;++r) for(int c=0;c<4;++c) out<<",T"<<r<<c;
    out<<"\n"<<std::setprecision(12);
    Eigen::Isometry3d world_from_scan=Eigen::Isometry3d::Identity();
    Mat last=Mat::Identity();
    Cloud::Ptr prev;
    for(int i=first;i<first+n;++i) {
        std::ostringstream name;
        name<<std::setw(6)<<std::setfill('0')<<i<<".bin";
        auto path=scans/name.str();
        if(await_download) {
            const auto deadline=std::chrono::steady_clock::now()+std::chrono::minutes(15);
            while(!fs::exists(path) && std::chrono::steady_clock::now()<deadline)
                std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        const auto frame_start=std::chrono::steady_clock::now();
        auto curr=read_scan(path,voxel_size);
        Match result{Mat::Identity(),true,0};
        auto start=std::chrono::steady_clock::now();
        if(prev) result=match(curr,prev,mode=="motion"?last:Mat::Identity());
        double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        // Retain the previous pose on failure and include that frame in evaluation.
        last=result.ok?result.prev_from_curr:Mat::Identity();
        Eigen::Quaterniond q(last.block<3,3>(0,0).cast<double>());
        Eigen::Isometry3d step=Eigen::Isometry3d::Identity();
        step.linear()=q.normalized().toRotationMatrix();
        step.translation()=last.block<3,1>(0,3).cast<double>();
        world_from_scan=world_from_scan*step;
        const double frame_ms=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-frame_start).count();
        out<<i<<","<<curr->size()<<","<<result.ok<<","<<ms<<","<<frame_ms<<","<<result.fitness;
        for(int r=0;r<3;++r) for(int c=0;c<4;++c) out<<","<<world_from_scan.matrix()(r,c);
        out<<"\n";
        prev=curr;
    }
    return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<"\n";return 1;}
