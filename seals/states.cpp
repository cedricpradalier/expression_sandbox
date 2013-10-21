
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "states.h"

#include <Eigen/Core>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace cerise;


bool DataLine::load(const std::string & line) {
    char buffer[line.size()];
    double f=0.0;
    int n=0;
    const char * lp = line.c_str();
    std::vector<double> dline;
    while (sscanf(lp, " %s%n", buffer, &n)==1) {
        if (sscanf(buffer,"%le",&f)==1) {
            dline.push_back(f);
        } else {
            dline.push_back(NAN);
        }
        lp += n;
    }
    if (dline.size() < 19) {
        LOG(ERROR) << "Not enough data filed on input line\n";
        return false;
    }
    timestamp = dline[0] + dline[1];
    depth = dline[8];
    vel = dline[9];
    vpred = dline[17];
    has_vel = round(dline[10]) > 0.5;
    dive_status = round(dline[18]);
    a[0] = dline[2];
    a[1] = dline[3];
    a[2] = dline[4];
    m[0] = dline[5];
    m[1] = dline[6];
    m[2] = dline[7];
    // conversion from ENU to NED (easier to manage with the compass and
    // accelerometer data in this convention)
    xyz[0] = dline[12];
    xyz[1] = dline[11];
    xyz[2] = -dline[13];
    // minus sign to convert from ENU to NED
    if (isnan(dline[14])) {
        rpy[0] = rpy[1] = rpy[2] = 0.0;
    } else {
        rpy[0] = -dline[14]*180./M_PI;
        rpy[1] = -dline[15]*180./M_PI;
        rpy[2] = -dline[16]*180./M_PI;
    }
    dive_id = 0;
    if (dline.size()>19) {
        dive_id = round(dline[19]);
    }
    return true;
}

bool DataLine::load_raw(const std::string & line) {
    char buffer[line.size()];
    double f=0.0;
    int n=0;
    const char * lp = line.c_str();
    std::vector<double> dline;
    while (sscanf(lp, " %s%n", buffer, &n)==1) {
        if (sscanf(buffer,"%le",&f)==1) {
            dline.push_back(f);
        } else {
            dline.push_back(NAN);
        }
        lp += n;
    }
    if (dline.size() < 11) {
        LOG(ERROR) << "Not enough data filed on input line\n";
        return false;
    }
    timestamp = dline[0] + dline[1];
    depth = dline[2];
    vel = dline[10];
    vpred = dline[10];
    has_vel = true;
    dive_status = -1;
    a[0] = dline[7];
    a[1] = dline[8];
    a[2] = dline[9];
    m[0] = dline[4];
    m[1] = dline[5];
    m[2] = dline[6];
    // conversion from ENU to NED (easier to manage with the compass and
    // accelerometer data in this convention)
    xyz[0] = 0.0;
    xyz[1] = 0.0;
    xyz[2] = -depth;
    // minus sign to convert from ENU to NED
    double na = sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    rpy[0] = -asin(a[0]/na);
    rpy[1] = atan2(a[1]/na,-a[2]/na);
    Eigen::Matrix3f roll; roll << 
        1, 0, 0,
        0, cos(rpy[0]), -sin(rpy[0]),
        0, sin(rpy[0]),  cos(rpy[0]);
    Eigen::Matrix3f pitch; pitch << 
         cos(rpy[1]), 0, sin(rpy[1]),
        0, 1, 0,
        -sin(rpy[1]), 0, cos(rpy[1]);
    Eigen::Vector3f M; M << m[0],m[1],m[2];
    Eigen::Vector3f Mr = pitch * roll * M;
    double compass = -atan2(Mr(1),Mr(0));
    rpy[2] = M_PI/2 - compass;
    // back to degrees
    for (int i=0;i<3;i++) {
        rpy[i] *= 180./M_PI;
    }
    dive_id = 0;
    return true;
}

bool GPSLine::load(const std::string & line) {
    double f=0.0;
    int n=0;
    const char * lp = line.c_str();
    std::vector<double> dline;
    while (sscanf(lp, " %le%n", &f, &n)==1) {
        lp += n;
        dline.push_back(f);
    }
    if (dline.size() < 12) {
        LOG(ERROR) << "Not enough data filed on gps input line\n";
        return false;
    }
    timestamp = dline[0] + dline[1];
    latitude = dline[2];
    longitude = dline[3];
    index = round(dline[4]) - 1;
    error = dline[5];
    easting = dline[6];
    northing = dline[7];
    zone = round(dline[8]);
    B[0] = dline[9];
    B[1] = dline[10];
    B[2] = dline[11];
    return true;
}

OptimisedOrientation::OptimisedOrientation() {
    for (size_t i=0;i<5;i++) {
        state[i] = 0.0;
    }
    rotation = state;
    propulsion = state + 4;
}

OptimisedOrientation::OptimisedOrientation(const OptimisedOrientation & oo) {
    dl_index = oo.dl_index;
    for (size_t i=0;i<5;i++) {
        state[i] = oo.state[i];
    }
    rotation = state;
    propulsion = state + 4;
}

bool OptimisedOrientation::load(const std::string & line) {
    double f=0.0;
    int n=0;
    const char * lp = line.c_str();
    std::vector<double> dline;
    while (sscanf(lp, " %le%n", &f, &n)==1) {
        lp += n;
        dline.push_back(f);
    }
    if (dline.size() < 6) {
        LOG(ERROR) << "Not enough data filed on orientation line\n";
        return false;
    }
    dl_index = (size_t)(round(dline[0]));
    for (size_t i=0;i<4;i++) rotation[i] = dline[i+1];
    propulsion[0] = dline[5];
    return true;
}

std::string OptimisedOrientation::save() const {
    char buffer[1024];
    snprintf(buffer,1023,"%d %e %e %e %e %e",(int)dl_index,
            rotation[0],rotation[1],rotation[2],rotation[3],propulsion[0]);
    return std::string(buffer);
}

OptimisedOrientationSequence::OptimisedOrientationSequence() {
    Bscale = common_parameters;
    Kdepth = common_parameters + 3;
    MagField = common_parameters + 4;
};

OptimisedOrientationSequence::OptimisedOrientationSequence(const OptimisedOrientationSequence & oos) {
    Bscale = common_parameters;
    Kdepth = common_parameters + 3;
    MagField = common_parameters + 4;
    for (size_t i=0;i<4;i++) {
        common_parameters[i] = oos.common_parameters[i];
    }
    input_file = oos.input_file;
    states = oos.states;
}

void OptimisedOrientationSequence::initialise(const std::string & source_file, const std::vector<DataLine> & lines) 
{
    input_file = source_file;
    Bscale[0] = 1.0;
    Bscale[1] = 1.0;
    Bscale[2] = 1.0;
    Kdepth[0] = 2.0/600.0;
    states.clear();
    double r = lines[0].rpy[0]*M_PI/180.;
    double p = lines[0].rpy[1]*M_PI/180.;
    Eigen::Matrix3f roll; roll << 
        1, 0, 0,
        0, cos(r), sin(r),
        0, -sin(r),  cos(r);
    Eigen::Matrix3f pitch; pitch << 
         cos(p), 0, -sin(p),
        0, 1, 0,
        sin(p), 0, cos(p);
    Eigen::Vector3f M; M << lines[0].m[0],lines[0].m[1],lines[0].m[2];
    Eigen::Vector3f Mr = pitch * roll * M;
    MagField[0] = Mr(0);
    MagField[1] = Mr(1);
    MagField[2] = Mr(2);
    
    for (size_t i=0;i<lines.size();i++) {
        const DataLine & dl(lines[i]);
        OptimisedOrientation oo;
        double mat[9] = {1,0,0,0,1,0,0,0,1};
        oo.dl_index = i;
        ceres::EulerAnglesToRotationMatrix<double>(dl.rpy,3,mat);
        double log[3];
        ceres::RotationMatrixToAngleAxis<double>(mat,log);
        ceres::AngleAxisToQuaternion<double>(log,oo.rotation);
        states.push_back(oo);
    }
}

bool OptimisedOrientationSequence::load(const std::string & filename)
{
    states.clear();
    FILE * fp = fopen(filename.c_str(),"r");
    while (!feof(fp)) {
        char line[4096] = {0,};
        if (fgets(line,4095, fp)!= NULL) {
            std::string l(line);
            if (l.substr(0,5) == "#Inpu") {
                char iname[1024];
                sscanf(line,"#Input %s",iname);
                input_file = iname;
            } else if (l.substr(0,5) == "#Bsca") {
                sscanf(line,"#Bscale %le %le %le",Bscale+0,Bscale+1,Bscale+2);
            } else if (l.substr(0,5) == "#MagF") {
                sscanf(line,"#MagField %le %le %le",MagField+0,MagField+1,MagField+2);
            } else if (l.substr(0,5) == "#Kdep") {
                sscanf(line,"#Kdepth %le",Kdepth);
            } else if (line[0] == '#') {
                continue;
            } else {
                OptimisedOrientation oo;
                if (oo.load(l)) {
                    states.push_back(oo);
                }
            }
        }
    }
    return true;
}

bool OptimisedOrientationSequence::save(const std::string & filename) const 
{
    FILE * fp = fopen(filename.c_str(),"w");
    if (!fp) {
        LOG(ERROR) << "Can't write file: '"<<filename<<"'\n";
        return false;
    }
    fprintf(fp,"#Input %s\n",input_file.c_str());
    fprintf(fp,"#MagField %e %e %e\n",MagField[0],MagField[1],MagField[2]);
    fprintf(fp,"#Bscale %e %e %e\n",Bscale[0],Bscale[1],Bscale[2]);
    fprintf(fp,"#Kdepth %e\n",Kdepth[0]);
    fprintf(fp,"#Header index  R0 R1 R2 [R3] Prop\n");
    for (size_t i=0;i<states.size();i++) {
        fprintf(fp,"%s\n",states[i].save().c_str());
    }
    fclose(fp);
    return true;
}

