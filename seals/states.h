#ifndef ES_STATES_H
#define ES_STATES_H

#include <string>
#include <vector>

namespace cerise {

    struct DataLine {
        double timestamp;
        double depth;
        bool has_vel;
        int dive_status;
        size_t dive_id;
        double vel;
        double vpred; // Initial velocity guess
        double a[3];
        double m[3];
        double xyz[3]; // Incremental position
        double rpy[3]; // Euler angles in degree

        bool load(const std::string & line) ;
        bool load_raw(const std::string & line) ;
    };

    struct GPSLine {
        double timestamp;
        double latitude;
        double longitude;
        int index;
        double error;
        double northing;
        double easting;
        int zone; // UTM Zone
        double B[3]; // Magnetic field

        bool load(const std::string & line) ;
    };


    struct OptimisedOrientation {
        size_t dl_index; // link to dataline index
        double state[5];
        double *rotation;
        double *propulsion;
        OptimisedOrientation(); 
        OptimisedOrientation(const OptimisedOrientation & oo); 

        bool load(const std::string & line);

        std::string save() const;
        
    };

    struct OptimisedOrientationSequence {
        std::string input_file;
        double common_parameters[7];
        double *Bscale; // 3D vector of magnetometer gains
        double *Kdepth; // Buoyancy factor: measure accel = G + Kdepth*depth*[0;0;1]
        double *MagField; // Magnetic field
        std::vector<OptimisedOrientation> states;

        OptimisedOrientationSequence();
        OptimisedOrientationSequence(const OptimisedOrientationSequence & oos); 

        void initialise(const std::string & source_file, const std::vector<DataLine> & lines);

        bool load(const std::string & filename);

        bool save(const std::string & filename) const;
    };

};



#endif // ES_STATES_H
