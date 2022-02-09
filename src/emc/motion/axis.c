
#include "axis.h"
#include "emcmotcfg.h"      // EMCMOT_MAX_AXIS
#include "rtapi.h"
#include "rtapi_math.h"
#include "simple_tp.h"

#define _(s) (s)

typedef struct {
    double pos_cmd;                 /* commanded axis position */
    double teleop_vel_cmd;          /* comanded axis velocity */
    double max_pos_limit;           /* upper soft limit on axis pos */
    double min_pos_limit;           /* lower soft limit on axis pos */
    double vel_limit;               /* upper limit of axis speed */
    double acc_limit;               /* upper limit of axis accel */
    simple_tp_t teleop_tp;          /* planner for teleop mode motion */

    int old_ajog_counts;            /* prior value, used for deltas */
    int kb_ajog_active;             /* non-zero during a keyboard jog */
    int wheel_ajog_active;          /* non-zero during a wheel jog */
    int locking_joint;              /* locking_joint number, -1 ==> notused */

    double ext_offset_vel_limit;    /* upper limit of axis speed for ext offset */
    double ext_offset_acc_limit;    /* upper limit of axis accel for ext offset */
    int old_eoffset_counts;
    simple_tp_t ext_offset_tp;      /* planner for external coordinate offsets*/
} emcmot_axis_t;

typedef struct {
    hal_float_t *pos_cmd;           /* RPI: commanded position */
    hal_float_t *teleop_vel_cmd;    /* RPI: commanded velocity */
    hal_float_t *teleop_pos_cmd;    /* RPI: teleop traj planner pos cmd */
    hal_float_t *teleop_vel_lim;    /* RPI: teleop traj planner vel limit */
    hal_bit_t   *teleop_tp_enable;  /* RPI: teleop traj planner is running */

    hal_s32_t   *ajog_counts;       /* WPI: jogwheel position input */
    hal_bit_t   *ajog_enable;       /* RPI: enable jogwheel */
    hal_float_t *ajog_scale;        /* RPI: distance to jog on each count */
    hal_float_t *ajog_accel_fraction;  /* RPI: to limit wheel jog accel */
    hal_bit_t   *ajog_vel_mode;     /* RPI: true for "velocity mode" jogwheel */
    hal_bit_t   *kb_ajog_active;    /* RPI: executing keyboard jog */
    hal_bit_t   *wheel_ajog_active; /* RPI: executing handwheel jog */

    hal_bit_t   *eoffset_enable;
    hal_bit_t   *eoffset_clear;
    hal_s32_t   *eoffset_counts;
    hal_float_t *eoffset_scale;
    hal_float_t *external_offset;
    hal_float_t *external_offset_requested;
} axis_hal_t;

typedef struct {
    axis_hal_t axis[EMCMOT_MAX_AXIS];   /* data for each axis */
} axis_hal_data_t;

static emcmot_axis_t axis_array[EMCMOT_MAX_AXIS];
static axis_hal_data_t *hal_data = NULL;


void axis_init_all(void)
{
    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis_array[n].locking_joint = -1;
    }
}

void axis_initialize_external_offsets(void)
{
    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        *(hal_data->axis[n].external_offset) = 0;
        *(hal_data->axis[n].external_offset_requested) = 0;
        axis_array[n].ext_offset_tp.pos_cmd  = 0;
        axis_array[n].ext_offset_tp.curr_pos = 0;
        axis_array[n].ext_offset_tp.curr_vel = 0;
    }
}


static int export_axis(int mot_comp_id, char c, axis_hal_t * addr)
{
    int retval=0, msg;
    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_WARN);

    retval += hal_pin_bit_newf(  HAL_IN, &(addr->ajog_enable),        mot_comp_id,"axis.%c.jog-enable", c);
    retval += hal_pin_float_newf(HAL_IN, &(addr->ajog_scale),         mot_comp_id,"axis.%c.jog-scale", c);
    retval += hal_pin_s32_newf(  HAL_IN, &(addr->ajog_counts),        mot_comp_id,"axis.%c.jog-counts", c);
    retval += hal_pin_bit_newf(  HAL_IN, &(addr->ajog_vel_mode),      mot_comp_id,"axis.%c.jog-vel-mode", c);
    retval += hal_pin_bit_newf(  HAL_OUT,&(addr->kb_ajog_active),     mot_comp_id,"axis.%c.kb-jog-active", c);
    retval += hal_pin_bit_newf(  HAL_OUT,&(addr->wheel_ajog_active),  mot_comp_id,"axis.%c.wheel-jog-active", c);
    retval += hal_pin_float_newf(HAL_IN, &(addr->ajog_accel_fraction),mot_comp_id,"axis.%c.jog-accel-fraction", c);

    *addr->ajog_accel_fraction = 1.0; // fraction of accel for wheel ajogs

    rtapi_set_msg_level(msg);
    return retval;
}

int axis_init_hal_io(int mot_comp_id)
{
    hal_data = hal_malloc(sizeof(axis_hal_data_t));
    if (!hal_data) {
        rtapi_print_msg(RTAPI_MSG_ERR, _("MOTION: axis_hal_data hal_malloc() failed\n"));
        return -1;
    }

    // export axis pins and parameters
    int retval=0;
    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        char c = "xyzabcuvw"[n];
        retval += hal_pin_float_newf(HAL_OUT, &hal_data->axis[n].pos_cmd,
                                     mot_comp_id, "axis.%c.pos-cmd", c);
        retval += hal_pin_float_newf(HAL_OUT, &hal_data->axis[n].teleop_vel_cmd,
                                     mot_comp_id, "axis.%c.teleop-vel-cmd", c);
        retval += hal_pin_float_newf(HAL_OUT, &hal_data->axis[n].teleop_pos_cmd,
                                     mot_comp_id, "axis.%c.teleop-pos-cmd", c);
        retval += hal_pin_float_newf(HAL_OUT, &hal_data->axis[n].teleop_vel_lim,
                                     mot_comp_id, "axis.%c.teleop-vel-lim",  c);
        retval += hal_pin_bit_newf(  HAL_OUT, &hal_data->axis[n].teleop_tp_enable,
                                     mot_comp_id, "axis.%c.teleop-tp-enable",c);
        retval += hal_pin_bit_newf(  HAL_IN, &hal_data->axis[n].eoffset_enable,
                                     mot_comp_id, "axis.%c.eoffset-enable", c);
        retval += hal_pin_bit_newf(  HAL_IN, &hal_data->axis[n].eoffset_clear,
                                     mot_comp_id, "axis.%c.eoffset-clear", c);
        retval += hal_pin_s32_newf(  HAL_IN, &hal_data->axis[n].eoffset_counts,
                                     mot_comp_id, "axis.%c.eoffset-counts", c);
        retval += hal_pin_float_newf(HAL_IN, &hal_data->axis[n].eoffset_scale,
                                     mot_comp_id, "axis.%c.eoffset-scale", c);
        retval += hal_pin_float_newf(HAL_OUT, &hal_data->axis[n].external_offset,
                                     mot_comp_id, "axis.%c.eoffset", c);
        retval += hal_pin_float_newf(HAL_OUT, &hal_data->axis[n].external_offset_requested,
                                     mot_comp_id, "axis.%c.eoffset-request", c);

        retval += export_axis(mot_comp_id, c, &hal_data->axis[n]);
        if (retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, _("MOTION: axis %c pin/param export failed <%d>\n"), c,retval);
            return -1;
        }
    }

    return 0;
}

void axis_output_to_hal(double *pcmd_p[])
{
    // output axis info to HAL for scoping, etc
    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        *(hal_data->axis[n].teleop_vel_cmd)    = axis_array[n].teleop_vel_cmd;
        *(hal_data->axis[n].teleop_pos_cmd)    = axis_array[n].teleop_tp.pos_cmd;
        *(hal_data->axis[n].teleop_vel_lim)    = axis_array[n].teleop_tp.max_vel;
        *(hal_data->axis[n].teleop_tp_enable)  = axis_array[n].teleop_tp.enable;
        *(hal_data->axis[n].kb_ajog_active)    = axis_array[n].kb_ajog_active;
        *(hal_data->axis[n].wheel_ajog_active) = axis_array[n].wheel_ajog_active;

        // hal pins: axis.L.pos-cmd reported without applied offsets:
        *(hal_data->axis[n].pos_cmd) = *pcmd_p[n]
                                     - axis_array[n].ext_offset_tp.curr_pos;
     }
}


void axis_set_max_pos_limit(int axis_num, double maxLimit)
{
    axis_array[axis_num].max_pos_limit = maxLimit;
}

void axis_set_min_pos_limit(int axis_num, double minLimit)
{
    axis_array[axis_num].min_pos_limit = minLimit;
}

void axis_set_vel_limit(int axis_num, double vel)
{
    axis_array[axis_num].vel_limit = vel;
}

void axis_set_acc_limit(int axis_num, double acc)
{
    axis_array[axis_num].acc_limit = acc;
}

void axis_set_ext_offset_vel_limit(int axis_num, double vel)
{
    axis_array[axis_num].ext_offset_vel_limit = vel;
}

void axis_set_ext_offset_acc_limit(int axis_num, double acc)
{
    axis_array[axis_num].ext_offset_acc_limit = acc;
}

void axis_set_locking_joint(int axis_num, int joint)
{
    axis_array[axis_num].locking_joint = joint;
}


double axis_get_min_pos_limit(int axis_num)
{
    return axis_array[axis_num].min_pos_limit;
}

double axis_get_max_pos_limit(int axis_num)
{
    return axis_array[axis_num].max_pos_limit;
}

double axis_get_vel_limit(int axis_num)
{
    return axis_array[axis_num].vel_limit;
}

double axis_get_acc_limit(int axis_num)
{
    return axis_array[axis_num].acc_limit;
}

double axis_get_teleop_vel_cmd(int axis_num)
{
    return axis_array[axis_num].teleop_vel_cmd;
}

int axis_get_locking_joint(int axis_num)
{
    return axis_array[axis_num].locking_joint;
}

double axis_get_compound_velocity(void)
{
    double v2 = 0.0;

    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        if (axis_array[n].teleop_tp.active)
            v2 += axis_array[n].teleop_vel_cmd * axis_array[n].teleop_vel_cmd;
    }

    if (v2 > 0.0)
        return sqrt(v2);
    return 0.0;
}

EmcPose axis_get_ext_offset_curr_pos()
{
    EmcPose p;
    p.tran.x = axis_array[0].ext_offset_tp.curr_pos;
    p.tran.y = axis_array[1].ext_offset_tp.curr_pos;
    p.tran.z = axis_array[2].ext_offset_tp.curr_pos;
    p.a      = axis_array[3].ext_offset_tp.curr_pos;
    p.b      = axis_array[4].ext_offset_tp.curr_pos;
    p.c      = axis_array[5].ext_offset_tp.curr_pos;
    p.u      = axis_array[6].ext_offset_tp.curr_pos;
    p.v      = axis_array[7].ext_offset_tp.curr_pos;
    p.w      = axis_array[8].ext_offset_tp.curr_pos;
    return p;
}


void axis_jog_cont(int n, double vel, long servo_period)
{
    double ext_offset_epsilon = TINY_DP(axis_array[n].ext_offset_tp.max_acc, servo_period);
    if (axis_array[n].ext_offset_tp.enable
        && (fabs(*(hal_data->axis[n].external_offset)) > ext_offset_epsilon)) {
        /* here: set pos_cmd to a big number so that with combined
        *        teleop jog plus external offsets the soft limits
        *        can always be reached
        */
        if (vel > 0.0) {
            axis_array[n].teleop_tp.pos_cmd =  1e12; // 1T halscope limit
        } else {
            axis_array[n].teleop_tp.pos_cmd = -1e12; // 1T halscope limit
        }
    } else {
        if (vel > 0.0) {
            axis_array[n].teleop_tp.pos_cmd = axis_array[n].max_pos_limit;
        } else {
            axis_array[n].teleop_tp.pos_cmd = axis_array[n].min_pos_limit;
        }
    }

    axis_array[n].teleop_tp.max_vel = fabs(vel);
    axis_array[n].teleop_tp.max_acc = axis_array[n].acc_limit;
    axis_array[n].kb_ajog_active = 1;
    axis_array[n].teleop_tp.enable = 1;
}

void axis_jog_incr(int n, double offset, double vel, long servo_period)
{
    double tmp1;
    double ext_offset_epsilon = TINY_DP(axis_array[n].ext_offset_tp.max_acc, servo_period);
    if (vel > 0.0) {
        tmp1 = axis_array[n].teleop_tp.pos_cmd + offset;
    } else {
        tmp1 = axis_array[n].teleop_tp.pos_cmd - offset;
    }

    // a fixed epsilon is used here for convenience
    // it is not the same as the epsilon used as a stopping
    // criterion in control.c
    if (axis_array[n].ext_offset_tp.enable
        && (fabs(*(hal_data->axis[n].external_offset)) > ext_offset_epsilon)) {
        // external_offsets: soft limit enforcement is in control.c
    } else {
        if (tmp1 > axis_array[n].max_pos_limit) { return; }
        if (tmp1 < axis_array[n].min_pos_limit) { return; }
    }

    axis_array[n].teleop_tp.pos_cmd = tmp1;
    axis_array[n].teleop_tp.max_vel = fabs(vel);
    axis_array[n].teleop_tp.max_acc = axis_array[n].acc_limit;
    axis_array[n].kb_ajog_active = 1;
    axis_array[n].teleop_tp.enable = 1;
}

void axis_jog_abs(int n, double offset, double vel)
{
    double tmp1;

    axis_array[n].kb_ajog_active = 1;
    // TELEOP JOG_ABS
    if (axis_array[n].wheel_ajog_active) { return; }
    if (vel > 0.0) {
        tmp1 = axis_array[n].teleop_tp.pos_cmd + offset;
    } else {
        tmp1 = axis_array[n].teleop_tp.pos_cmd - offset;
    }
    if (tmp1 > axis_array[n].max_pos_limit) { return; }
    if (tmp1 < axis_array[n].min_pos_limit) { return; }
    axis_array[n].teleop_tp.pos_cmd = tmp1;
    axis_array[n].teleop_tp.max_vel = fabs(vel);
    axis_array[n].teleop_tp.max_acc = axis_array[n].acc_limit;
    axis_array[n].kb_ajog_active = 1;
    axis_array[n].teleop_tp.enable = 1;
}

void axis_jog_abort(int n)
{
    axis_array[n].teleop_tp.enable = 0;
}

void axis_jog_abort_all(void)
{
    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis_jog_abort(n);
    }
}

bool axis_jog_immediate_stop_all(void)
{
    int aborted = false;

    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        if (axis_array[n].teleop_tp.enable) {
            aborted = true;
        }
        axis_array[n].teleop_tp.enable = 0;
        axis_array[n].teleop_tp.curr_vel = 0.0;
    }
    return aborted;
}


void axis_handle_jogwheels(bool motion_teleop_flag, bool motion_enable_flag, bool homing_is_active)
{
    int n;
    int new_ajog_counts, delta;
    double distance, pos, stop_dist;
    static int first_pass = 1; /* used to set initial conditions */

    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        double aaccel_limit;
        // disallow accel bogus fractions
        if (   (*(hal_data->axis[n].ajog_accel_fraction) > 1)
            || (*(hal_data->axis[n].ajog_accel_fraction) < 0) ) {
            aaccel_limit = axis_array[n].acc_limit;
        } else {
            aaccel_limit = *(hal_data->axis[n].ajog_accel_fraction) * axis_array[n].acc_limit;
        }

        new_ajog_counts = *(hal_data->axis[n].ajog_counts);
        delta = new_ajog_counts - axis_array[n].old_ajog_counts;
        axis_array[n].old_ajog_counts = new_ajog_counts;
        if ( first_pass ) { continue; }
        if ( delta == 0 ) {
            //just update counts
            continue;
        }
        if (!motion_teleop_flag) {
            axis_array[n].teleop_tp.enable = 0;
            return;
        }
        if (!motion_enable_flag)                     { continue; }
        if ( *(hal_data->axis[n].ajog_enable) == 0 ) { continue; }
        if (homing_is_active)                        { continue; }
        if (axis_array[n].kb_ajog_active)            { continue; }

        if (axis_array[n].locking_joint >= 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
            "Cannot wheel jog a locking indexer AXIS_%c\n",
            "XYZABCUVW"[n]);
            continue;
        }

        distance = delta * *(hal_data->axis[n].ajog_scale);
        pos = axis_array[n].teleop_tp.pos_cmd + distance;
        if ( *(hal_data->axis[n].ajog_vel_mode) ) {
            double v = axis_array[n].vel_limit;
            /* compute stopping distance at max speed */
            stop_dist = v * v / ( 2 * aaccel_limit);
            /* if commanded position leads the actual position by more
               than stopping distance, discard excess command */
            if ( pos > axis_array[n].pos_cmd + stop_dist ) {
                pos = axis_array[n].pos_cmd + stop_dist;
            } else if ( pos < axis_array[n].pos_cmd - stop_dist ) {
                pos = axis_array[n].pos_cmd - stop_dist;
            }
        }
        if (pos > axis_array[n].max_pos_limit) { break; }
        if (pos < axis_array[n].min_pos_limit) { break; }
        axis_array[n].teleop_tp.pos_cmd = pos;
        axis_array[n].teleop_tp.max_vel = axis_array[n].vel_limit;
        axis_array[n].teleop_tp.max_acc = aaccel_limit;
        axis_array[n].wheel_ajog_active = 1;
        axis_array[n].teleop_tp.enable  = 1;
    }
    first_pass = 0;
}


void axis_sync_teleop_tp_to_carte_pos(int extfactor, double *pcmd_p[])
{
    // expect extfactor =  -1 || 0 || +1
    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        axis_array[n].teleop_tp.curr_pos = *pcmd_p[n]
                                         + extfactor * axis_array[n].ext_offset_tp.curr_pos;
    }
}

void axis_sync_carte_pos_to_teleop_tp(int extfactor, double *pcmd_p[])
{
    // expect extfactor =  -1 || 0 || +1
    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        *pcmd_p[n] = axis_array[n].teleop_tp.curr_pos
                   + extfactor * axis_array[n].ext_offset_tp.curr_pos;
    }
}

void axis_apply_ext_offsets_to_carte_pos(int extfactor, double *pcmd_p[])
{
    // expect extfactor =  -1 || 0 || +1
    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        *pcmd_p[n] = *pcmd_p[n]
                   + extfactor * axis_array[n].ext_offset_tp.curr_pos;
    }
}



hal_bit_t axis_plan_external_offsets(double servo_period, bool motion_enable_flag, bool all_homed)
{
    static int first_pass = 1;
    int new_eoffset_counts, delta;
    static int last_eoffset_enable[EMCMOT_MAX_AXIS];
    double ext_offset_epsilon;
    hal_bit_t eoffset_active;

    eoffset_active = 0;

    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        // coord,teleop updates done in get_pos_cmds()
        axis_array[n].ext_offset_tp.max_vel = axis_array[n].ext_offset_vel_limit;
        axis_array[n].ext_offset_tp.max_acc = axis_array[n].ext_offset_acc_limit;

        new_eoffset_counts       = *(hal_data->axis[n].eoffset_counts);
        delta                    = new_eoffset_counts - axis_array[n].old_eoffset_counts;
        axis_array[n].old_eoffset_counts = new_eoffset_counts;

        *(hal_data->axis[n].external_offset)  = axis_array[n].ext_offset_tp.curr_pos;
        axis_array[n].ext_offset_tp.enable = 1;
        if ( first_pass ) {
            *(hal_data->axis[n].external_offset) = 0;
            continue;
        }

        // Use stopping criterion of simple_tp.c:
        ext_offset_epsilon = TINY_DP(axis_array[n].ext_offset_tp.max_acc, servo_period);
        if (fabs(*(hal_data->axis[n].external_offset)) > ext_offset_epsilon) {
            eoffset_active = 1;
        }
        if ( !*(hal_data->axis[n].eoffset_enable) ) {
            axis_array[n].ext_offset_tp.enable = 0;
            // Detect disabling of eoffsets:
            //   At very high accel, simple planner may terminate with
            //   a larger position value than occurs at more realistic accels.
            if (last_eoffset_enable[n]
                && (fabs(*(hal_data->axis[n].external_offset)) > ext_offset_epsilon)
                && motion_enable_flag
                && axis_array[n].ext_offset_tp.enable) {
                rtapi_print_msg(RTAPI_MSG_NONE,
                           "*** Axis_%c External Offset=%.4g eps=%.4g\n"
                           "*** External Offset disabled while NON-zero\n"
                           "*** To clear: re-enable & zero or use Machine-Off\n",
                           "XYZABCUVW"[n],
                           *(hal_data->axis[n].external_offset),
                           ext_offset_epsilon);
            }
            last_eoffset_enable[n] = 0;
            continue; // Note: if   not eoffset_enable
                      //       then planner disabled and no pos_cmd updates
                      //       useful for eoffset_pid hold
        }
        last_eoffset_enable[n] = 1;
        if (*(hal_data->axis[n].eoffset_clear)) {
            axis_array[n].ext_offset_tp.pos_cmd             = 0;
            *(hal_data->axis[n].external_offset_requested) = 0;
            continue;
        }
        if (delta == 0)           { continue; }
        if (!all_homed)           { continue; }
        if (!motion_enable_flag)  { continue; }

        axis_array[n].ext_offset_tp.pos_cmd   += delta *  *(hal_data->axis[n].eoffset_scale);
        *(hal_data->axis[n].external_offset_requested) = axis_array[n].ext_offset_tp.pos_cmd;
    } // for n
    first_pass = 0;

    return eoffset_active;
}

/* Return -1 if over negative limit, 1 if over positive limit, or 0 if in range
** *failing_axis_no first axis letter failing
*/
int axis_check_constraint(EmcPose pos, int* failing_axis_no) {

// special case due to initializations for unused axis letters:
#define eps  1e-308
#define SPECIAL_CASE(target,ano) \
    (   (fabs(target) < eps) \
     && (fabs(axis_array[ano].min_pos_limit) < eps) \
     && (fabs(axis_array[ano].max_pos_limit) < eps) )  ? (1) : (0)

// see pull request #1047
#define UNDER_LIMIT(target,ano) \
    (target < (axis_array[ano].min_pos_limit - 1e-12)) ? (1) : (0)
#define OVER_LIMIT(target,ano) \
    (target > (axis_array[ano].max_pos_limit + 1e-12)) ? (1) : (0)

    if (   SPECIAL_CASE(pos.tran.x,0)
        && SPECIAL_CASE(pos.tran.y,1)
        && SPECIAL_CASE(pos.tran.z,2)
        && SPECIAL_CASE(pos.a,     3)
        && SPECIAL_CASE(pos.b,     4)
        && SPECIAL_CASE(pos.c,     5)
        && SPECIAL_CASE(pos.u,     6)
        && SPECIAL_CASE(pos.v,     7)
        && SPECIAL_CASE(pos.w,     8) ) return 0;

    int ano;
    ano=0; if (UNDER_LIMIT(pos.tran.x,ano)) { *failing_axis_no = ano; return -1;}
    ano=1; if (UNDER_LIMIT(pos.tran.y,ano)) { *failing_axis_no = ano; return -1;}
    ano=2; if (UNDER_LIMIT(pos.tran.z,ano)) { *failing_axis_no = ano; return -1;}
    ano=3; if (UNDER_LIMIT(pos.a     ,ano)) { *failing_axis_no = ano; return -1;}
    ano=4; if (UNDER_LIMIT(pos.b     ,ano)) { *failing_axis_no = ano; return -1;}
    ano=5; if (UNDER_LIMIT(pos.c     ,ano)) { *failing_axis_no = ano; return -1;}
    ano=6; if (UNDER_LIMIT(pos.u     ,ano)) { *failing_axis_no = ano; return -1;}
    ano=7; if (UNDER_LIMIT(pos.v     ,ano)) { *failing_axis_no = ano; return -1;}
    ano=8; if (UNDER_LIMIT(pos.w     ,ano)) { *failing_axis_no = ano; return -1;}

    ano=0; if (OVER_LIMIT (pos.tran.x,ano)) { *failing_axis_no = ano; return  1;}
    ano=1; if (OVER_LIMIT (pos.tran.y,ano)) { *failing_axis_no = ano; return  1;}
    ano=2; if (OVER_LIMIT (pos.tran.z,ano)) { *failing_axis_no = ano; return  1;}
    ano=3; if (OVER_LIMIT (pos.a     ,ano)) { *failing_axis_no = ano; return  1;}
    ano=4; if (OVER_LIMIT (pos.b     ,ano)) { *failing_axis_no = ano; return  1;}
    ano=5; if (OVER_LIMIT (pos.c     ,ano)) { *failing_axis_no = ano; return  1;}
    ano=6; if (OVER_LIMIT (pos.u     ,ano)) { *failing_axis_no = ano; return  1;}
    ano=7; if (OVER_LIMIT (pos.v     ,ano)) { *failing_axis_no = ano; return  1;}
    ano=8; if (OVER_LIMIT (pos.w     ,ano)) { *failing_axis_no = ano; return  1;}

    return 0;
#undef SPECIAL_CASE
#undef UNDER_LIMIT
#undef OVER_LIMIT
}


int axis_update_coord_with_bound(double *pcmd_p[], double servo_period)
{
    int ans = 0;
    double save_pos_cmd[EMCMOT_MAX_AXIS];
    double save_offset_cmd[EMCMOT_MAX_AXIS];

    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        save_pos_cmd[n]     = *pcmd_p[n];
        save_offset_cmd[n]  = axis_array[n].ext_offset_tp.pos_cmd;
        simple_tp_update(&(axis_array[n].ext_offset_tp), servo_period);
    }
    axis_apply_ext_offsets_to_carte_pos(+1, pcmd_p); // add external offsets

    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        //workaround: axis letters not in [TRAJ]COORDINATES
        //            have min_pos_limit == max_pos_lim == 0
        if ( (0 == axis_array[n].max_pos_limit) && (0 == axis_array[n].min_pos_limit) ) {
            continue;
        }
        if (axis_array[n].ext_offset_tp.curr_pos == 0) {
           continue; // don't claim violation if no offset
        }

        if (*pcmd_p[n] >= axis_array[n].max_pos_limit) {
            // hold carte_pos_cmd at the limit:
            *pcmd_p[n]  = axis_array[n].max_pos_limit;
            // stop growth of offsetting position:
            axis_array[n].ext_offset_tp.curr_pos = axis_array[n].max_pos_limit
                                                 - save_pos_cmd[n];
            if (axis_array[n].ext_offset_tp.pos_cmd > save_offset_cmd[n]) {
                axis_array[n].ext_offset_tp.pos_cmd = save_offset_cmd[n];
            }
            axis_array[n].ext_offset_tp.curr_vel = 0;
            ans++;
            continue;
        }
        if (*pcmd_p[n] <= axis_array[n].min_pos_limit) {
            *pcmd_p[n]  = axis_array[n].min_pos_limit;
            axis_array[n].ext_offset_tp.curr_pos = axis_array[n].min_pos_limit
                                                 - save_pos_cmd[n];
            if (axis_array[n].ext_offset_tp.pos_cmd < save_offset_cmd[n]) {
                axis_array[n].ext_offset_tp.pos_cmd = save_offset_cmd[n];
            }
            axis_array[n].ext_offset_tp.curr_vel = 0;
            ans++;
        }
    }
    if (ans > 0) { return 1; }
    return 0;
}


static int update_teleop_with_check(int n, simple_tp_t *the_tp, double servo_period)
{
    // 'the_tp' is the planner to update
    // the tests herein apply to the sum of the offsets for both
    // planners (teleop_tp and ext_offset_tp)
    double save_curr_pos;
    save_curr_pos = the_tp->curr_pos;
    simple_tp_update(the_tp, servo_period);

    //workaround: axis letters not in [TRAJ]COORDINATES
    //            have min_pos_limit == max_pos_lim == 0
    if  ( (0 == axis_array[n].max_pos_limit) && (0 == axis_array[n].min_pos_limit) ) {
        return 0;
    }
    if  ( (axis_array[n].ext_offset_tp.curr_pos + axis_array[n].teleop_tp.curr_pos)
          >= axis_array[n].max_pos_limit) {
        // positive error, restore save_curr_pos
        the_tp->curr_pos = save_curr_pos;
        the_tp->curr_vel = 0;
        return 1;
    }
    if  ( (axis_array[n].ext_offset_tp.curr_pos + axis_array[n].teleop_tp.curr_pos)
           <= axis_array[n].min_pos_limit) {
        // negative error, restore save_curr_pos
        the_tp->curr_pos = save_curr_pos;
        the_tp->curr_vel = 0;
        return 1;
    }
    return 0;
}

int axis_calc_motion(double servo_period)
{
    int violated_teleop_limit = 0;
    int n;
    for (n = 0; n < EMCMOT_MAX_AXIS; n++) {
        // teleop_tp.max_vel is always positive
        if (axis_array[n].teleop_tp.max_vel > axis_array[n].vel_limit) {
            axis_array[n].teleop_tp.max_vel = axis_array[n].vel_limit;
        }
        if (update_teleop_with_check(n, &(axis_array[n].teleop_tp), servo_period)) {
            violated_teleop_limit = 1;
        } else {
            axis_array[n].teleop_vel_cmd = axis_array[n].teleop_tp.curr_vel;
            axis_array[n].pos_cmd = axis_array[n].teleop_tp.curr_pos;
        }

        if (!axis_array[n].teleop_tp.active) {
            axis_array[n].kb_ajog_active = 0;
            axis_array[n].wheel_ajog_active = 0;
        }

        if (axis_array[n].ext_offset_tp.enable) {
            if (update_teleop_with_check(n, &(axis_array[n].ext_offset_tp), servo_period)) {
                violated_teleop_limit = 1;
            }
        }
    }
    return violated_teleop_limit;
}
