#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <Eigen/Core>

using namespace std;

struct PnPData {
    double x, y, z, u, v;
};

string float2str(float num);
float str2float(string str);

double str2double(string str);
string double2str(double num);

int str2int(string str);
string int2str(int num);

string long2str(long num);

void getIntrinsic(const string path, vector<float> &intrinsic);
void getDistortion(const string path, vector<float> &distortion);
void getExtrinsic(const string path, vector<float> &extrinsic);
void writeExt(const string path, const Eigen::Matrix3d &rot, const Eigen::Vector3d &t);
void getData(const string lidar_path, const string photo_path, vector<PnPData> &pData);
void rotation2angle(Eigen::Matrix4d rot);

string configPath = "config";

// convert a int to a string
string int2str(int num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[float2str] - Code error" << endl;
        exit(0);
    }
}

int str2int(string str) {
    int d;
    stringstream sin(str);
    if(sin >> d) {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to int" << endl;
    exit(0);
}

string float2str(float num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[float2str] - Code error" << endl;
        exit(0);
    }
}

float str2float(string str) {
    float d;
    stringstream sin(str);
    if(sin >> d) {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to float" << endl;
    exit(0);
}

string double2str(double num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[double2str] - Code error" << endl;
        exit(0);
    }
}

double str2double(string str) {
    double d;
    stringstream sin(str);
    if(sin >> d) {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to double" << endl;
    exit(0);
}

string long2str(long num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[long2str] - Code error" << endl;
        exit(0);
    }
}

void getIntrinsic(const string path, vector<float> &intrinsic) {
    ifstream inFile;
    inFile.open(path);
    if (!inFile.is_open()) {
        cout << "Can not open file " << path << endl; 
        exit(1);
    }

    string lineStr;
    getline(inFile, lineStr);
    for (uint i = 0; i < 3; ++i) {
        getline(inFile, lineStr);
        stringstream line(lineStr);
        string str;
        
        line >> str;
        intrinsic.push_back(str2double(str));
        line >> str;
        intrinsic.push_back(str2double(str));
        line >> str;
        intrinsic.push_back(str2double(str));
    }
}

void getDistortion(const string path, vector<float> &distortion) {
    ifstream inFile;
    inFile.open(path);
    if (!inFile.is_open()) {
        cout << "Can not open file " << path << endl; 
        exit(1);
    }
    string lineStr;
    for (uint i = 0; i < 6; ++i) {
        getline(inFile, lineStr);
    }
    
    getline(inFile, lineStr);
    stringstream line(lineStr);
    string str;
        
    line >> str;
    distortion.push_back(str2double(str));
    line >> str;
    distortion.push_back(str2double(str));
    line >> str;
    distortion.push_back(str2double(str));
    line >> str;
    distortion.push_back(str2double(str));
    line >> str;
    distortion.push_back(str2double(str));
}

void getExtrinsic(const string path, vector<float> &extrinsic) {
    ifstream inFile;
    inFile.open(path);
    if (!inFile.is_open()) {
        cout << "Can not open file " << path << endl; 
        exit(1);
    }

    string lineStr;
    getline(inFile, lineStr);
    for (uint i = 0; i < 3; ++i) {
        getline(inFile, lineStr);
        stringstream line(lineStr);
        string str;
        
        line >> str;
        extrinsic.push_back(str2double(str));
        line >> str;
        extrinsic.push_back(str2double(str));
        line >> str;
        extrinsic.push_back(str2double(str));
        line >> str;
        extrinsic.push_back(str2double(str));
    }
}

void rotation2angle(Eigen::Matrix4d rot) {
    double sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if(!singular) {
        x = atan2(rot(2, 1), rot(2, 2)) * 180 / M_PI;
        y = atan2(-rot(2, 0), sy) * 180 / M_PI;
        z = atan2(rot(1, 0), rot(0, 0)) * 180 / M_PI;
    }
    else {
        x = atan2(-rot(1, 2), rot(1, 1)) * 180 / M_PI;
        y = atan2(-rot(2, 0), sy) * 180 / M_PI;
        z = 0;
    }
    cout << x << " " << y << " " << z << endl << endl; // roll pitch yaw
}

void writeExt(const string path, const Eigen::Matrix3d &rot, const Eigen::Vector3d &t) {
    string filename = path;
    ofstream outfile(filename.c_str());
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else {
        outfile << "extrinsic" << endl;
        outfile << rot(0, 0) << "  " << rot(0, 1) << "  " << rot(0, 2) << "  " << t[0] << endl;
        outfile << rot(1, 0) << "  " << rot(1, 1) << "  " << rot(1, 2) << "  " << t[1] << endl;
        outfile << rot(2, 0) << "  " << rot(2, 1) << "  " << rot(2, 2) << "  " << t[2] << endl;
        outfile << "0  0  0  1" << endl;
    }
}

void getData(const string lidar_path, const string photo_path, vector<PnPData> &pData) {
    ifstream inFile_lidar;
    ifstream inFile_photo;

    inFile_lidar.open(lidar_path);
    inFile_photo.open(photo_path);
    string lineStr_lidar;
    string lineStr_photo;

    while(getline(inFile_lidar, lineStr_lidar) && getline(inFile_photo, lineStr_photo)) {
        if (lineStr_lidar.size() > 10 && lineStr_photo.size() > 10) {  // ignore the index
            PnPData pp;
            string str;
            stringstream line_lidar(lineStr_lidar);
            stringstream line_photo(lineStr_photo);

            line_lidar >> str;
            pp.x = str2double(str);
            cout << pp.x << " ";

            line_lidar >> str;
            pp.y = str2double(str);
            cout << pp.y << " ";

            line_lidar >> str;
            pp.z = str2double(str);
            cout << pp.z << " ";

            line_photo >> str;
            pp.u = str2double(str);
            cout << pp.u << " ";

            line_photo >> str;
            pp.v = str2double(str);
            cout << pp.v << " ";

            pData.push_back(pp);
            cout << endl;
        }
        else if(lineStr_lidar.size() < 1 && lineStr_photo.size() < 1) {  // stop reading the data when there is an empty line
            break;
        }
        else if ((lineStr_lidar.size() < 10 && lineStr_photo.size() > 10) || (lineStr_lidar.size() > 10 && lineStr_photo.size() < 10)) {
            cout << lineStr_lidar.size() << " " << lineStr_photo.size();
            cout << "Lidar data and photo data not aligned!" << endl;
            exit(1);
        }
    }
    inFile_lidar.close();
    inFile_photo.close();
}

#endif // COMMON_H