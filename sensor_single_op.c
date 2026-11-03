#include "sensor_signal.h"

#include <stddef.h>
#include <string.h>

/*
 * Optimized 22 x 32 pressure-array processor.
 * Public API and data structures remain unchanged.
 */

#ifndef SENSOR_IS_BAD_PIXEL
#define SENSOR_IS_BAD_PIXEL(index) (0u)
#endif

#ifndef SENSOR_NOISE_FLOOR
#define SENSOR_NOISE_FLOOR 0u
#endif

#if SENSOR_NOISE_FLOOR > 255u
#error "SENSOR_NOISE_FLOOR must be in the range 0..255"
#endif

#define CALIBRATION_FRAMES              20u
#define DETECT_MARGIN                   4u
#define ROW_ACTIVE_MIN                  3u
#define CONSECUTIVE_ROWS_MIN            13u
#define LONG_ROWS_MIN                   17u
#define TOP3_FRACTION_MAX               30u
#define THIN_TOP3_FRACTION_MAX          33u
#define VOTE_THRESHOLD                  4u
#define LEAVE_FRAMES                    50u
#define BOOT_PERSON_PRESSURE_SUM        5000u

#define MOTION_DIFF_AVERAGE_ENTER       2u
#define MOTION_DIFF_AVERAGE_EXIT        1u
#define MOTION_CHANGED_POINTS_ENTER     12u
#define MOTION_CHANGED_POINTS_EXIT      8u
#define MOTION_ENTER_FRAMES             2u
#define MOTION_STABLE_FRAMES            4u

/* Temporary statistics for one half of the sensor array. */
typedef struct
{
    uint16_t column_sum[SENSOR_HALF_COLS];
    uint16_t top_rows[3];
    uint16_t valid_points;
    uint32_t pressure_sum;
    uint8_t current_active_rows;
    uint8_t max_consecutive_rows;
} HalfAccumulator;


/* Filter one sensor sample. */
static uint8_t filter_sample(uint32_t index, uint8_t raw)
{
    (void)index;

    if (SENSOR_IS_BAD_PIXEL(index)) {
        return 0u;
    }

    if (raw <= (uint8_t)SENSOR_NOISE_FLOOR) {
        return 0u;
    }

    return raw;
}


/* Maintain the three largest row sums. */
static void insert_top3(HalfAccumulator *acc, uint16_t row_sum)
{
    if (row_sum >= acc->top_rows[0]) {
        acc->top_rows[2] = acc->top_rows[1];
        acc->top_rows[1] = acc->top_rows[0];
        acc->top_rows[0] = row_sum;
    }
    else if (row_sum >= acc->top_rows[1]) {
        acc->top_rows[2] = acc->top_rows[1];
        acc->top_rows[1] = row_sum;
    }
    else if (row_sum > acc->top_rows[2]) {
        acc->top_rows[2] = row_sum;
    }
}


/* Calculate statistics for one row. */
static void calc_row(HalfAccumulator *acc,
                     uint16_t row_sum,
                     uint8_t active_points)
{
    insert_top3(acc, row_sum);

    if (active_points >= ROW_ACTIVE_MIN) {
        if (acc->current_active_rows < 255u) {
            ++acc->current_active_rows;
        }

        if (acc->current_active_rows > acc->max_consecutive_rows) {
            acc->max_consecutive_rows = acc->current_active_rows;
        }
    }
    else {
        acc->current_active_rows = 0u;
    }
}


/* Calculate features for one half of the sensor array. */
static void calc_half(const HalfAccumulator *acc,
                      SensorHalfResult *out)
{
    uint16_t max_column = 0u;
    uint32_t top3_sum;
    uint32_t i;

    for (i = 0u; i < SENSOR_HALF_COLS; ++i) {
        if (acc->column_sum[i] > max_column) {
            max_column = acc->column_sum[i];
        }
    }

    out->pressure_sum = acc->pressure_sum;

    out->average_pressure = (acc->valid_points == 0u)
        ? 0u
        : (uint16_t)(acc->pressure_sum / acc->valid_points);

    out->max_column_sum = max_column;
    out->max_consecutive_rows = acc->max_consecutive_rows;

    top3_sum = (uint32_t)acc->top_rows[0]
             + (uint32_t)acc->top_rows[1]
             + (uint32_t)acc->top_rows[2];

    out->top3_row_fraction_percent = (acc->pressure_sum == 0u)
        ? 100u
        : (uint8_t)((top3_sum * 100u) / acc->pressure_sum);
}


/* Generate a detection vote. */
static uint8_t make_vote(const SensorHalfResult *half,
                         uint16_t threshold)
{
    const uint8_t fraction_limit =
        (half->max_consecutive_rows >= LONG_ROWS_MIN)
        ? THIN_TOP3_FRACTION_MAX
        : TOP3_FRACTION_MAX;

    return (half->average_pressure > threshold &&
            half->max_consecutive_rows >= CONSECUTIVE_ROWS_MIN &&
            half->top3_row_fraction_percent <= fraction_limit)
        ? 1u : 0u;
}


/* Count positive votes. */
static uint8_t sum_votes(const uint8_t votes[SENSOR_VOTE_WINDOW])
{
    uint8_t sum = 0u;
    uint32_t i;

    for (i = 0u; i < SENSOR_VOTE_WINDOW; ++i) {
        sum = (uint8_t)(sum + (votes[i] != 0u ? 1u : 0u));
    }

    return sum;
}


/* Update the occupied state of one side. */
static void update_side(uint8_t *occupied,
                        uint8_t *leave_count,
                        uint8_t current_vote,
                        uint8_t vote_sum)
{
    if (*occupied == 0u) {
        *leave_count = 0u;

        if (vote_sum >= VOTE_THRESHOLD) {
            *occupied = 1u;
        }
    }
    else if (current_vote != 0u) {
        *leave_count = 0u;
    }
    else {
        if (*leave_count < LEAVE_FRAMES) {
            ++(*leave_count);
        }

        if (*leave_count >= LEAVE_FRAMES) {
            *occupied = 0u;
            *leave_count = 0u;
        }
    }
}


/* Update motion detection state. */
static void update_motion(SensorProcessor *processor,
                          SensorResult *result,
                          uint32_t difference_sum,
                          uint16_t changed_points)
{
    result->motion_changed_points = changed_points;

    result->motion_average_difference =
        (uint16_t)(difference_sum / SENSOR_FRAME_SIZE);

    if (processor->in_motion == 0u) {
        processor->motion_stable_count = 0u;

        if (result->motion_average_difference >=
                MOTION_DIFF_AVERAGE_ENTER &&
            changed_points >= MOTION_CHANGED_POINTS_ENTER) {

            if (processor->motion_enter_count < MOTION_ENTER_FRAMES) {
                ++processor->motion_enter_count;
            }

            if (processor->motion_enter_count >= MOTION_ENTER_FRAMES) {
                processor->in_motion = 1u;
                processor->motion_enter_count = 0u;
            }
        }
        else {
            processor->motion_enter_count = 0u;
        }
    }
    else {
        processor->motion_enter_count = 0u;

        if (result->motion_average_difference <
                MOTION_DIFF_AVERAGE_EXIT ||
            changed_points < MOTION_CHANGED_POINTS_EXIT) {

            if (processor->motion_stable_count < MOTION_STABLE_FRAMES) {
                ++processor->motion_stable_count;
            }

            if (processor->motion_stable_count >= MOTION_STABLE_FRAMES) {
                processor->in_motion = 0u;
                processor->motion_stable_count = 0u;
            }
        }
        else {
            processor->motion_stable_count = 0u;
        }
    }

    result->in_motion = processor->in_motion;
}


/* Initialize the sensor processor. */
void SensorProcessor_Init(SensorProcessor *processor)
{
    if (processor == NULL) {
        return;
    }

    memset(processor, 0, sizeof(*processor));

    processor->threshold_left = 10u;
    processor->threshold_right = 10u;
}


/* Process one complete sensor frame. */
SensorStatus SensorProcessor_Process(
    SensorProcessor *processor,
    const uint8_t frame[SENSOR_FRAME_SIZE],
    SensorResult *result)
{
    HalfAccumulator left_acc;
    HalfAccumulator right_acc;

    uint32_t difference_sum = 0u;
    uint16_t changed_points = 0u;

    uint32_t row;
    uint32_t col;
    uint32_t frame_sum;

    uint8_t left_votes;
    uint8_t right_votes;

    const uint8_t had_previous = (processor != NULL)
        ? processor->previous_frame_valid : 0u;

    if (processor == NULL || frame == NULL || result == NULL) {
        return SENSOR_STATUS_BAD_ARGUMENT;
    }

    memset(result, 0, sizeof(*result));
    memset(&left_acc, 0, sizeof(left_acc));
    memset(&right_acc, 0, sizeof(right_acc));

    /* Traverse the complete frame once. */
    for (row = 0u; row < SENSOR_ROWS; ++row) {
        uint16_t left_row_sum = 0u;
        uint16_t right_row_sum = 0u;

        uint8_t left_active_points = 0u;
        uint8_t right_active_points = 0u;

        for (col = 0u; col < SENSOR_COLS; ++col) {
            const uint32_t index = row * SENSOR_COLS + col;

            const uint8_t raw_sample = frame[index];
            const uint8_t sample = filter_sample(index, raw_sample);

            uint8_t previous = 0u;

            if (col < SENSOR_HALF_COLS) {
                left_row_sum = (uint16_t)(left_row_sum + sample);

                left_acc.column_sum[col] =
                    (uint16_t)(left_acc.column_sum[col] + sample);

                if (sample != 0u) {
                    ++left_active_points;
                }

                left_acc.pressure_sum += sample;

                if (!SENSOR_IS_BAD_PIXEL(index)) {
                    ++left_acc.valid_points;
                }
            }
            else {
                const uint32_t half_col = col - SENSOR_HALF_COLS;

                right_row_sum = (uint16_t)(right_row_sum + sample);

                right_acc.column_sum[half_col] =
                    (uint16_t)(right_acc.column_sum[half_col] + sample);

                if (sample != 0u) {
                    ++right_active_points;
                }

                right_acc.pressure_sum += sample;

                if (!SENSOR_IS_BAD_PIXEL(index)) {
                    ++right_acc.valid_points;
                }
            }

            /* Calculate motion difference. */
            if (had_previous != 0u &&
                !SENSOR_IS_BAD_PIXEL(index)) {

                previous = filter_sample(
                    index,
                    processor->previous_frame[index]
                );

                {
                    const uint8_t difference = (sample >= previous)
                        ? (uint8_t)(sample - previous)
                        : (uint8_t)(previous - sample);

                    difference_sum += difference;

                    if (difference >= 2u) {
                        ++changed_points;
                    }
                }
            }

            processor->previous_frame[index] = raw_sample;
        }

        calc_row(&left_acc, left_row_sum, left_active_points);
        calc_row(&right_acc, right_row_sum, right_active_points);
    }

    /* Calculate left and right features. */
    calc_half(&left_acc, &result->left);
    calc_half(&right_acc, &result->right);

    /* Update motion state. */
    if (had_previous == 0u) {
        processor->previous_frame_valid = 1u;
    }
    else {
        update_motion(
            processor,
            result,
            difference_sum,
            changed_points
        );
    }

    /* Calibration stage. */
    if (processor->calibration_done == 0u) {
        frame_sum = result->left.pressure_sum
                  + result->right.pressure_sum;

        if (processor->calibration_count == 0u &&
            frame_sum > BOOT_PERSON_PRESSURE_SUM) {

            processor->threshold_left = DETECT_MARGIN;
            processor->threshold_right = DETECT_MARGIN;
            processor->calibration_done = 1u;

            return SENSOR_STATUS_CALIBRATING;
        }

        if (processor->calibration_count < CALIBRATION_FRAMES) {
            processor->calibration_sum_left +=
                result->left.average_pressure;

            processor->calibration_sum_right +=
                result->right.average_pressure;

            ++processor->calibration_count;
        }

        if (processor->calibration_count >= CALIBRATION_FRAMES) {
            processor->threshold_left = (uint16_t)(
                processor->calibration_sum_left /
                CALIBRATION_FRAMES + DETECT_MARGIN
            );

            processor->threshold_right = (uint16_t)(
                processor->calibration_sum_right /
                CALIBRATION_FRAMES + DETECT_MARGIN
            );

            processor->calibration_done = 1u;
        }

        return SENSOR_STATUS_CALIBRATING;
    }

    /* Generate detection votes. */
    result->left.single_frame_vote =
        make_vote(&result->left, processor->threshold_left);

    result->right.single_frame_vote =
        make_vote(&result->right, processor->threshold_right);

    processor->vote_left[processor->vote_index] =
        result->left.single_frame_vote;

    processor->vote_right[processor->vote_index] =
        result->right.single_frame_vote;

    ++processor->vote_index;

    if (processor->vote_index >= SENSOR_VOTE_WINDOW) {
        processor->vote_index = 0u;
    }

    /* Update occupied states. */
    left_votes = sum_votes(processor->vote_left);
    right_votes = sum_votes(processor->vote_right);

    update_side(
        &processor->left_occupied,
        &processor->leave_count_left,
        result->left.single_frame_vote,
        left_votes
    );

    update_side(
        &processor->right_occupied,
        &processor->leave_count_right,
        result->right.single_frame_vote,
        right_votes
    );

    result->left.occupied = processor->left_occupied;
    result->right.occupied = processor->right_occupied;

    result->person_count = (uint8_t)(
        processor->left_occupied + processor->right_occupied
    );

    result->in_motion = processor->in_motion;

    return SENSOR_STATUS_READY;
}
