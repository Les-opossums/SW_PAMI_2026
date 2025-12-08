#ifndef LOCALIZATION_H
#define LOCALIZATION_H

// table configuration
#define TABLE_SIZE_X 1000.0f  // in mm
#define TABLE_SIZE_Y 2000.0f  // in mm

#define LOC_TOLERANCE_MM 300.0f  // in mm

#define LD19_MAX_PTS_SCAN 1200

typedef struct {
    float x;      // in mm
    float y;      // in mm
    float theta;  // in radians
    bool valid;
} RobotPose;    

typedef struct {
    LD19DataPoint points[LD19_MAX_PTS_SCAN];
    uint16_t count;
} LIDARScan;

RobotPose Loc_ProcessScan(const LIDARScan* scan, RobotPose* prev_pose);
#endif // LOCALIZATION_H