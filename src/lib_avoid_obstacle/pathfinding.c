#include "../PAMI_2026.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Internal state for the current goal
static Point2D current_goal = {0.0f, 0.0f};

// Helper: Limit magnitude of a vector
static void limit_magnitude(float* x, float* y, float max_val) {
    float mag_sq = (*x * *x) + (*y * *y);
    if (mag_sq > (max_val * max_val)) {
        float mag = sqrtf(mag_sq);
        *x = (*x / mag) * max_val;
        *y = (*y / mag) * max_val;
    }
}

void Path_Init(void) {
    current_goal.x = 0.0f;
    current_goal.y = 0.0f;
}

void Path_SetGoal(float x, float y) {
    current_goal.x = x;
    current_goal.y = y;
}



VelocityCommand Path_GetRepulsionVector(const LD19DataPointHandler* scan, float goal_vx, float goal_vy) {
    VelocityCommand rep = {goal_vx, goal_vy, 0.0f, false};

    if (scan == NULL || scan->index == 0) return rep;

    float g_norm = sqrtf(goal_vx * goal_vx + goal_vy * goal_vy);
    if (g_norm < 0.01f) return rep;

    float norm_gx = goal_vx / g_norm;
    float norm_gy = goal_vy / g_norm;

    static const float field_radius     = 300.0f;
    static const float avoid_radius     = 80.0f;
    static const float MIN_AVOID_SPEED  = 0.15f;
    static const float HYSTERESIS       = 250.0f;
    static const int   EXIT_DELAY       = 40;
    static const int   OPPOSITE_DELAY   = 3;
    static const float OPPOSITE_THRESH  = 8.0f;
    static const int   FLIP_COOLDOWN    = 20;   // cycles de gel après un flip

    float min_d         = field_radius;
    float cross_sum     = 0.0f;
    float closest_cross = 0.0f;
    int   found         = 0;
    int   count         = 0;

    for (int i = 0; i < scan->index; i++) {
        float d = scan->points[i].distance;
        if (d < 10.0f || d >= field_radius) continue;

        float px  = scan->points[i].x;
        float py  = scan->points[i].y;
        float dot = px * norm_gx + py * norm_gy;
        if (dot < -30.0f) continue;

        float cross = norm_gx * py - norm_gy * px;
        cross_sum += cross;
        count++;

        if (d < min_d) {
            min_d         = d;
            closest_cross = cross;
            found         = 1;
        }
    }

    if (count > 0) cross_sum /= (float)count;

    static float swirl_sign     =  1.0f;
    static int   is_avoiding    =  0;
    static int   free_cycles    =  0;
    static float filtered_cross =  0.0f;
    static float prev_min_d     =  300.0f;
    static int   opposite_count =  0;
    static int   flip_cooldown  =  0;   // NEW

    if (found) {
        free_cycles = 0;

        float t_raw = (field_radius - min_d) / (field_radius - avoid_radius);
        if (t_raw < 0.0f) t_raw = 0.0f;
        if (t_raw > 1.0f) t_raw = 1.0f;

        // Décrémenter le cooldown
        if (flip_cooldown > 0) flip_cooldown--;

        // Détection obstacle opposé — uniquement si pas en cooldown
        if (is_avoiding && t_raw > 0.20f && flip_cooldown == 0) {
            bool closest_opposite = (swirl_sign > 0.0f && closest_cross < -OPPOSITE_THRESH) ||
                                    (swirl_sign < 0.0f && closest_cross >  OPPOSITE_THRESH);
            if (closest_opposite) {
                opposite_count++;
                if (opposite_count >= OPPOSITE_DELAY) {
                    is_avoiding    = 0;
                    filtered_cross = 0.0f;
                    opposite_count = 0;
                    prev_min_d     = field_radius;
                    flip_cooldown  = FLIP_COOLDOWN;  // geler après reset
                }
            } else {
                opposite_count = 0;
            }
        } else if (flip_cooldown > 0) {
            opposite_count = 0;  // remettre à zéro pendant le cooldown
        }

        // Reset par saut de distance
        if (is_avoiding && prev_min_d < 150.0f && min_d > 220.0f) {
            is_avoiding    = 0;
            filtered_cross = 0.0f;
            opposite_count = 0;
            flip_cooldown  = FLIP_COOLDOWN;
        }
        prev_min_d = min_d;

        if (!is_avoiding) {
            filtered_cross = closest_cross;
            if (fabsf(closest_cross) < 5.0f) {
                swirl_sign = +1.0f;
            } else {
                swirl_sign = (closest_cross >= 0.0f) ? +1.0f : -1.0f;
            }
            is_avoiding = 1;
        } else {
            filtered_cross = filtered_cross * 0.80f + cross_sum * 0.20f;

            if (flip_cooldown == 0) {
                if (swirl_sign > 0.0f && filtered_cross < -HYSTERESIS) {
                    swirl_sign     = -1.0f;
                    filtered_cross = cross_sum;
                    opposite_count = 0;
                    flip_cooldown  = FLIP_COOLDOWN;
                } else if (swirl_sign < 0.0f && filtered_cross > +HYSTERESIS) {
                    swirl_sign     = +1.0f;
                    filtered_cross = cross_sum;
                    opposite_count = 0;
                    flip_cooldown  = FLIP_COOLDOWN;
                }
            }
        }

        float t = t_raw * sqrtf(t_raw);  // t^1.5

        float deflection = swirl_sign * (M_PI * 0.50f) * t;

        float effective_norm = g_norm;
        if (effective_norm < MIN_AVOID_SPEED && t > 0.15f) {
            effective_norm = MIN_AVOID_SPEED;
        }

        float eff_vx = norm_gx * effective_norm;
        float eff_vy = norm_gy * effective_norm;
        float cos_d  = cosf(deflection);
        float sin_d  = sinf(deflection);

        rep.vx = eff_vx * cos_d - eff_vy * sin_d;
        rep.vy = eff_vx * sin_d + eff_vy * cos_d;

        printf("DBG avoid: min_d=%.0f t=%.2f angle=%.1fdeg sign=%.0f ic=%.0f cc=%.0f fc=%.0f opp=%d cd=%d\n",
               min_d, t, deflection * 180.0f / M_PI, swirl_sign, cross_sum, closest_cross, filtered_cross, opposite_count, flip_cooldown);

    } else {
        prev_min_d     = field_radius;
        opposite_count = 0;
        if (flip_cooldown > 0) flip_cooldown--;
        free_cycles++;
        if (free_cycles >= EXIT_DELAY) {
            is_avoiding    = 0;
            filtered_cross = 0.0f;
            swirl_sign     = +1.0f;
            flip_cooldown  = 0;
        }
    }

    return rep;
}


VelocityCommand Path_ComputeVelocity(RobotPose current_pose, const LD19DataPointHandler* scan) {
    VelocityCommand cmd = {0.0f, 0.0f, 0.0f, false};

    float dx_global = current_goal.x - current_pose.x;
    float dy_global = current_goal.y - current_pose.y;
    float distance_to_goal = sqrtf(dx_global*dx_global + dy_global*dy_global);

    if (distance_to_goal < PF_GOAL_TOLERANCE) {
        cmd.reached = true;
        return cmd;
    }

    float cos_theta = cosf(current_pose.theta);
    float sin_theta = sinf(current_pose.theta);
    float dx_local =  dx_global * cos_theta + dy_global * sin_theta;
    float dy_local = -dx_global * sin_theta + dy_global * cos_theta;

    // Vecteur attraction limité en vitesse
    float F_att_x = dx_local * PF_ATTRACTIVE_GAIN;
    float F_att_y = dy_local * PF_ATTRACTIVE_GAIN;
    limit_magnitude(&F_att_x, &F_att_y, PF_MAX_SPEED);

    // Path_GetRepulsionVector REMPLACE le vecteur — pas d'addition
    VelocityCommand rep = Path_GetRepulsionVector(scan, F_att_x, F_att_y);

    cmd.vx    = rep.vx;
    cmd.vy    = rep.vy;
    cmd.omega = 0.0f;

    return cmd;
}