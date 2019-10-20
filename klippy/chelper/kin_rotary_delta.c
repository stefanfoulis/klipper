// Rotary delta kinematics stepper pulse time generation
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct rotary_stepper {
    struct stepper_kinematics sk;
    double cos, sin, shoulder_radius, shoulder_height;
    double upper_arm2, arm2_diff;
};

// Inverse kinematics based on the following two formulas:
//   elbow_x**2 + elbow_y**2 = upper_arm**2
//   (effector_x - elbow_x)**2 + (effector_y - elbow_y)**2 + effector_z**2
//       = lower_arm**2

static double
rotary_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    struct rotary_stepper *rs = container_of(sk, struct rotary_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    // Rotate and shift axes to an origin at shoulder joint with upper
    // arm constrained to xy plane and x aligned to shoulder platform.
    double sjz = c.y * rs->cos - c.x * rs->sin;
    double sjx = c.x * rs->cos + c.y * rs->sin - rs->shoulder_radius;
    double sjy = c.z - rs->shoulder_height;
    // Determine constants such that: sj_elbow_y = c1 - c2*sj_elbow_x
    double inv_sjy = 1. / sjy;
    double c1 = .5 * inv_sjy * (sjx*sjx + sjy*sjy + sjz*sjz + rs->arm2_diff);
    double c2 = sjx * inv_sjy;
    // Calculate scaled elbow coordinates via quadratic equation.
    double scale = c2*c2 + 1.0;
    double sj_scaled_elbow_x = c1*c2 + sqrt(scale*rs->upper_arm2 - c1*c1);
    double sj_scaled_elbow_y = c1*scale - c2*sj_scaled_elbow_x;
    // Calculate angle in radians
    return atan2(sj_scaled_elbow_y, sj_scaled_elbow_x);
}

struct stepper_kinematics * __visible
rotary_delta_stepper_alloc(double shoulder_radius, double shoulder_height
                           , double angle, double upper_arm, double lower_arm)
{
    struct rotary_stepper *rs = malloc(sizeof(*rs));
    memset(rs, 0, sizeof(*rs));
    rs->cos = cos(angle);
    rs->sin = sin(angle);
    rs->shoulder_radius = shoulder_radius;
    rs->shoulder_height = shoulder_height;
    rs->upper_arm2 = upper_arm * upper_arm;
    rs->arm2_diff = rs->upper_arm2 - lower_arm * lower_arm;
    rs->sk.calc_position_cb = rotary_stepper_calc_position;
    rs->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &rs->sk;
}
