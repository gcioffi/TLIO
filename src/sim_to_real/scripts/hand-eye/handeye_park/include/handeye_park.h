#pragma once

#include <fstream>
#include <iostream>
#include <filesystem>

#include <Eigen/Dense>
#include <sophus/se3.hpp>


std::vector<Sophus::SE3d> loadPoses(const std::string& path)
{
  std::vector<Sophus::SE3d> poses;

  // debug
  std::cout << "path: " << path << "\n";
  // end

  std::ifstream data(path.c_str());

  if (data)
  {
    std::string line;

    while(std::getline(data, line))
    {
      std::stringstream line_stream(line);
      double t;
      Eigen::Vector3d p;
      Eigen::Quaterniond q;
      Eigen::Vector3d v;
      line_stream >> t >> q.w() >> q.x() >> q.y() >> q.z() 
      >> p.x() >> p.y() >> p.z() 
      >> v.x() >> v.y() >> v.z();

      // generate pose
      Sophus::SE3d pose(q, p);

      // add to vector
      poses.push_back(pose);
    }
  }

  return poses;
}


class HandEyePark
{
    public:
    HandEyePark() {};
    ~HandEyePark() {};
    
    void loadData(const std::string &hand_poses_path, const std::string &eye_poses_path)
    {
        hand_poses_ = loadPoses(hand_poses_path);
        eye_poses_ = loadPoses(eye_poses_path);
    }
    
    // methods
    Sophus::SE3d run()
    {
        const int n = hand_poses_.size();
        
        Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3*n, 3);
        Eigen::VectorXd b_A = Eigen::VectorXd::Zero(3*n);
        Eigen::VectorXd b_B = Eigen::VectorXd::Zero(3*n);

        for (int i=1; i<n; i++)
        {
            // compute A
            const Eigen::Matrix3d A0 = hand_poses_[i-1].rotationMatrix();
            const Eigen::Matrix3d A1 = hand_poses_[i].rotationMatrix();
            const Eigen::Matrix3d A = A0.transpose() * A1;

            // compute B
            const Eigen::Matrix3d B0 = eye_poses_[i-1].rotationMatrix();
            const Eigen::Matrix3d B1 = eye_poses_[i].rotationMatrix();
            const Eigen::Matrix3d B = B0.transpose() * B1;


            // matrix log
            const Eigen::Vector3d alpha = Sophus::SO3d(A).log();
            const Eigen::Vector3d beta = Sophus::SO3d(B).log();

            // build M
            M += beta * alpha.transpose();

            // build C
            C.block<3, 3>(3*(i-1), 0) = Eigen::Matrix3d::Identity() - A;

            // build b_A and b_B
            b_A.segment<3>(3*(i-1)) = A0.transpose() * (hand_poses_[i].translation() - hand_poses_[i-1].translation());
            b_B.segment<3>(3*(i-1)) = B0.transpose() * (eye_poses_[i].translation() - eye_poses_[i-1].translation());
        }
        
        // compute rotation
        Eigen::EigenSolver<Eigen::MatrixXd> es(M.transpose() * M);
        Eigen::Matrix3d Lambda = Eigen::Vector3d(sqrt(1./es.eigenvalues().real()[0]), sqrt(1./es.eigenvalues().real()[1]), sqrt(1./es.eigenvalues().real()[2])).asDiagonal();
        Eigen::Matrix3d V = es.eigenvectors().real();
        Eigen::Matrix3d X = V * Lambda * V.inverse() * M.transpose();

        // compute translation
        Eigen::VectorXd d = Eigen::VectorXd::Zero(3*n);
        for (int i=0; i<n; i++)
        {
            d.segment<3>(3*i) = b_A.segment<3>(3*i) - X*b_B.segment<3>(3*i);
        }
        Eigen::Vector3d b = (C.transpose()*C).inverse() * C.transpose()*d;

        return Sophus::SE3d(X, b);
    };
    
    private:
    std::vector<Sophus::SE3d> hand_poses_; // sim poses
    std::vector<Sophus::SE3d> eye_poses_; // real poses
};

