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


RobotPose Loc_ProcessScan(const LD19DataPointHandler* scan, RobotPose* prev_pose);
#endif // LOCALIZATION_H