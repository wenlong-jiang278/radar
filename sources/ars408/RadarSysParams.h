#ifndef USER_RADARSYSPARAMS_H
#define USER_RADARSYSPARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double             rng_delta;
    double             vel_delta;
    double             az_delta_deg;
    unsigned short int trk_capt_delay;
    unsigned short int trk_drop_delay;
    unsigned short int trk_fps;
    short int          trk_fov_az_right;
    short int          trk_fov_az_left;
    short int          trk_fov_ev_down;
    short int          trk_fov_ev_up;
    unsigned short int trk_nf_thres;
} radar_sys_params_t;

typedef struct {
    double MatrixTransCoordinate[6];
    double Elev_Factor_P[6];
    double Radar_Longitude;
    double Radar_Latitude;
    float  TrueNorthDeflectAngle;
} RadarCoordinateInfodef;

#ifdef __cplusplus
}
#endif

#endif
