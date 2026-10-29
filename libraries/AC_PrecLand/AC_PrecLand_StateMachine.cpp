#include "AC_PrecLand_config.h"

#if AC_PRECLAND_ENABLED

#include <AC_PrecLand/AC_PrecLand.h>
#include "AC_PrecLand_StateMachine.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <GCS_MAVLink/GCS.h>

static const float MAX_POS_ERROR_M = 0.75f;  // Maximum position error for retry locations
static const uint32_t FAILSAFE_INIT_TIMEOUT_MS = 7000;   // Timeout in ms before failsafe measures are started. During this period vehicle is completely stopped to give user the time to take over
static const float RETRY_OFFSET_ALT_M = 1.5f;  // This gets added to the altitude of the retry location

// Miss-abort thresholds. The climb reuses RETRY_OFFSET_ALT_M so the retry still goes up about 1.5 m.
static constexpr float PRECLAND_LEAN_ABORT_DEG = 10.0f;
static constexpr float PRECLAND_LEAN_HOLD_DEG = 20.0f;
static constexpr float PRECLAND_ATT_ERR_ABORT_DEG = 10.0f;
static constexpr uint32_t PRECLAND_LEAN_PERSIST_MS = 150;
static constexpr float PRECLAND_CLIMB_ARRIVE_M = 0.30f;   // close enough to the climb altitude to start translating
static constexpr float PRECLAND_BESIDE_ARRIVE_M = 0.30f;  // close enough to the offset to start the final descent

// Initialize the state machine. This is called every time vehicle switches mode
void AC_PrecLand_StateMachine::init()
{
    // init is only called ONCE per mode change. So in a particular mode we can retry only a finite times.
    // The counter will be reset if the statemachine is called from a different mode
    _retry_count = 0;
    _miss_phase = MissPhase::NONE;
    _beside_phase = MissPhase::NONE;
    _lean_hold = false;
    _precland_suppressed = false;
    _lean_alerted = false;
    _lean_since_ms = 0;
    // reset every other statemachine
    reset_failed_landing_statemachine();

    AC_PrecLand *_precland = AP::ac_precland();
    if (_precland == nullptr || !_precland->enabled()) {
        // precland is not enabled, prec land state machine methods should not be called!
        return;
    }
    _precland->reset_miss_state();
    _precland->clear_final_drop_freeze();
}

// Reset the landing statemachines. This needs to be called every time the landing target is back in sight.
// So that if the landing target goes out of sight again, we can start the failed landing procedure back from the beginning stage
void AC_PrecLand_StateMachine::reset_failed_landing_statemachine()
{
    landing_target_lost_action = TargetLostAction::INIT;
    _retry_state = RetryLanding::INIT;
    failsafe_initialized = false;
}

// Run Prec Land State Machine. During Prec Landing, we might encounter four scenarios:
// 1. We had the target in sight, but have lost it now. 2. We never had the target in sight and user wants to land.
// 3. We have the target in sight and can continue landing. 4. The sensor is out of range
// This method deals with all of these scenarios
// Returns the action needed to be done by the vehicle.
// Parameters: Vector3f "retry_pos_m" is filled with the required location if we need to retry landing.
AC_PrecLand_StateMachine::Status AC_PrecLand_StateMachine::update(Vector3p &retry_pos_m, bool landed)
{

    // grab the current status of Landing Target
    AC_PrecLand *_precland = AP::ac_precland();
    if (_precland == nullptr) {
        // should never happen
        return Status::ERROR;
    }

    if (!_precland->enabled()) {
        // precland is not enabled, prec land state machine should not be called!
        return Status::ERROR;
    }

    // A real settle must not be treated as a miss, and must not start another climb.
    if (landed) {
        _miss_phase = MissPhase::NONE;
        _lean_hold = false;
        _lean_since_ms = 0;
        return Status::DESCEND;
    }

    if (_lean_hold) {
        if (excessive_lean()) {
            retry_pos_m = _hold_pos_ned_m;
            return Status::RETRYING;
        }
        _lean_hold = false;
        _lean_alerted = false;
    }

    // An in-progress miss or beside-pad manoeuvre is not cancelled by the target coming back.
    if (_beside_phase != MissPhase::NONE) {
        if ((_beside_phase == MissPhase::CLIMB || _beside_phase == MissPhase::TRANSLATE) && excessive_lean()) {
            begin_lean_hold(retry_pos_m);
            return Status::RETRYING;
        }
        return update_beside(retry_pos_m);
    }

    if (_miss_phase == MissPhase::CLIMB || _miss_phase == MissPhase::TRANSLATE) {
        if (excessive_lean()) {
            begin_lean_hold(retry_pos_m);
            return Status::RETRYING;
        }
        return update_miss_retry(retry_pos_m);
    }

    if (_precland->do_miss_abort()) {
        update_lean_timer();
        if (_precland->in_miss_abort_window() &&
            (_precland->missed_below_pad() || lean_abort_latched())) {
            // Below the pad, or an uncommanded lean, retries immediately. The 4 s
            // "keep descending" stage is for a lost target, not for a missed pin.
            return begin_abort(retry_pos_m);
        }
    }

    AC_PrecLand::TargetState precland_target_state =  _precland->get_target_state();

    Status status = Status::ERROR;
    switch (precland_target_state) {
    case AC_PrecLand::TargetState::TARGET_RECENTLY_LOST:
        // we have lost the target but had it in sight at least once recently
        // action will depend on what user wants
        status = get_target_lost_actions(retry_pos_m);
        break;

    case AC_PrecLand::TargetState::TARGET_NEVER_SEEN:
        // we have no clue where we are supposed to be landing
        // let user decide how strict our failsafe actions need to be
        status = Status::FAILSAFE;
        break;

    case AC_PrecLand::TargetState::TARGET_OUT_OF_RANGE:
        // The target isn't in sight, but we can't run any fail safe measures or do landing retry
        // Therefore just descend for now, and check again later if retry is allowed
    case AC_PrecLand::TargetState::TARGET_FOUND:
        // no action required, target is in sight
        reset_failed_landing_statemachine();
        status = Status::DESCEND;
        break;
    }

    // With the miss-abort option, exhausting retries lands beside the pad instead of
    // hovering or landing vertically on the last position. No last target keeps the
    // existing strictness failsafe.
    if (status == Status::FAILSAFE && _precland->do_miss_abort() && _precland->has_last_target()) {
        return begin_beside(retry_pos_m);
    }
    return status;
}


// Target is lost (i.e we had it in sight some time back), this method helps decide on what needs to be done next
// The chosen action depends on user set landing strictness and will be returned by this function
// Parameters: Vector3f "retry_pos_m" is filled with the required location if we need to retry landing.
AC_PrecLand_StateMachine::Status AC_PrecLand_StateMachine::get_target_lost_actions(Vector3p &retry_pos_m)
{
    AC_PrecLand *_precland = AP::ac_precland();
    if (_precland == nullptr) {
        // should never happen
        return Status::ERROR;
    }

    switch (landing_target_lost_action) {
    case TargetLostAction::INIT: {
        // figure out how strict the user is with the landing
        const RetryStrictness strictness =_precland->get_retry_strictness();
        switch (strictness) {
            case RetryStrictness::NORMAL:
            case RetryStrictness::VERY_STRICT:
                // We eventually want to retry landing, but lets descend for some time and hope the target gets in sight
                // If not, we will retry landing
                landing_target_lost_action = TargetLostAction::DESCEND;
                break;
            case RetryStrictness::NOT_STRICT:
                // User just wants to land, prec land isn't a priority
                landing_target_lost_action = TargetLostAction::LAND_VERTICALLY;
                break;
        }
        // at this stage we will be descending no matter what
        // so no special action required
        return Status::DESCEND;
    }

    case TargetLostAction::DESCEND:
        if (AP_HAL::millis() - _precland->get_last_valid_target_ms() >=_precland->get_min_retry_time_sec() * 1000) {
            // we have descended for some time and the target still isn't in sight
            // lets retry
            landing_target_lost_action = TargetLostAction::RETRY_LANDING;
            _retry_state = RetryLanding::INIT;
        }
        // still descending, no other action
        return Status::DESCEND;

    case TargetLostAction::RETRY_LANDING:
        // retry the landing by going to another position
        return retry_landing(retry_pos_m);

    case TargetLostAction::LAND_VERTICALLY:
        // Just land vertically
        // we will not be retrying to any location here on, so return false
        return Status::DESCEND;
    }

    // should never reach here, all cases are handled above
    return Status::ERROR;
}

// Retry landing based on a previously known location of the landing target
// Returns the action that should be taken by the vehicle
// Vector3f "retry_pos_m" is filled with the required location.
AC_PrecLand_StateMachine::Status AC_PrecLand_StateMachine::retry_landing(Vector3p &retry_pos_m)
{
    AC_PrecLand *_precland = AP::ac_precland();
    if (_precland == nullptr) {
        // should never happen
        return Status::ERROR;
    }

    if (_precland->get_max_retry_allowed() == 0) {
        // user does not want retry
        return Status::FAILSAFE;
    }

    if (_retry_count > _precland->get_max_retry_allowed()) {
        // we have exhausted the amount of times vehicle was allowed to retry landing
        // do failsafe measure so the vehicle isn't stuck in a constant loop
        return Status::FAILSAFE;
    }

    // get the retry position. This depends on what retry behavior has been set by user
    Vector3p go_to_pos;
    const RetryAction retry_action = _precland->get_retry_behaviour();
    if (retry_action == RetryAction::GO_TO_TARGET_LOC) {
        _precland->get_last_detected_landing_pos_NED_m(go_to_pos);
    } else if (retry_action == RetryAction::GO_TO_LAST_LOC) {
        _precland->get_last_vehicle_pos_when_target_detected_NED_m(go_to_pos);
    }

    // add a little bit offset so the vehicle climbs slightly higher than where it was
    // remember this is "D" frame and in meters's
    go_to_pos.z -= RETRY_OFFSET_ALT_M;

    switch (_retry_state) {
    case RetryLanding::INIT:
        // Init the Retry
        _retry_count ++;
        _retry_state = RetryLanding::IN_PROGRESS;
        // inform the user what we are doing
        if (_retry_count <= _precland->get_max_retry_allowed()) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Retrying");
        }
        retry_pos_m = go_to_pos;
        return Status::RETRYING;

    case RetryLanding::IN_PROGRESS: {
        // continue converging towards the target till we are close by
        retry_pos_m = go_to_pos;
        Vector3p pos;
        if (!AP::ahrs().get_relative_position_NED_origin(pos)) {
            return Status::ERROR;
        }
        const float dist_to_target = (go_to_pos-pos).length();
        if ((dist_to_target < MAX_POS_ERROR_M)) {
            // we have approx reached landing location previously detected
            _retry_state = RetryLanding::DESCEND;
        }
        return Status::RETRYING;
    }

    case RetryLanding::DESCEND: {
        // descend a little bit before completing the retry
        // This will descend to the original height of where landing target was first detected
        Vector3p pos;
        if (!AP::ahrs().get_relative_position_NED_origin(pos)) {
            return Status::ERROR;
        }
        // z_target is in "D" frame
        const float z_target = go_to_pos.z + RETRY_OFFSET_ALT_M;
        retry_pos_m = Vector3p{pos.x, pos.y, z_target};
#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wabsolute-value"
#endif
        if (fabsf(pos.z - retry_pos_m.z) < MAX_POS_ERROR_M) {
#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#pragma clang diagnostic pop
#endif
            // we have descended to the original height where we started the climb from
            _retry_state = RetryLanding::COMPLETE;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Retry Completed");
        }
        return Status::RETRYING;
    }

    case RetryLanding::COMPLETE:
        // Vehicle has completed a retry, and most likely the landing location still isn't sight
        // we have no choice but to force a failsafe action
        return Status::FAILSAFE;
    }

    // should never reach here
    return Status::ERROR;
}

// Lean the position controller did not ask for, or a large attitude-error angle.
bool AC_PrecLand_StateMachine::uncommanded_lean() const
{
    const AP_AHRS &ahrs = AP::ahrs();
    const float lean_deg = MAX(fabsf(degrees(ahrs.get_roll_rad())), fabsf(degrees(ahrs.get_pitch_rad())));

    float commanded_deg = 0.0f;
    const AC_PosControl *pos_control = AC_PosControl::get_singleton();
    if (pos_control != nullptr) {
        commanded_deg = MAX(fabsf(degrees(pos_control->get_roll_rad())), fabsf(degrees(pos_control->get_pitch_rad())));
    }

    float att_err_deg = 0.0f;
    const AC_AttitudeControl *attitude_control = AC_AttitudeControl::get_singleton();
    if (attitude_control != nullptr) {
        att_err_deg = attitude_control->get_att_error_angle_deg();
    }

    const bool uncommanded = lean_deg > PRECLAND_LEAN_ABORT_DEG && commanded_deg < PRECLAND_LEAN_ABORT_DEG;
    return uncommanded || att_err_deg > PRECLAND_ATT_ERR_ABORT_DEG;
}

bool AC_PrecLand_StateMachine::excessive_lean() const
{
    const AP_AHRS &ahrs = AP::ahrs();
    const float lean_deg = MAX(fabsf(degrees(ahrs.get_roll_rad())), fabsf(degrees(ahrs.get_pitch_rad())));
    return lean_deg > PRECLAND_LEAN_HOLD_DEG;
}

void AC_PrecLand_StateMachine::update_lean_timer()
{
    AC_PrecLand *precland = AP::ac_precland();
    if (precland == nullptr || !precland->in_miss_abort_window() || !uncommanded_lean()) {
        _lean_since_ms = 0;
        return;
    }
    if (_lean_since_ms == 0) {
        _lean_since_ms = AP_HAL::millis();
    }
}

bool AC_PrecLand_StateMachine::lean_abort_latched() const
{
    return _lean_since_ms != 0 && (AP_HAL::millis() - _lean_since_ms) >= PRECLAND_LEAN_PERSIST_MS;
}

void AC_PrecLand_StateMachine::begin_lean_hold(Vector3p &retry_pos_m)
{
    if (!_lean_hold) {
        Vector3p pos_ned;
        if (AP::ahrs().get_relative_position_NED_origin(pos_ned)) {
            _hold_pos_ned_m = pos_ned;
        }
        _lean_hold = true;
        if (!_lean_alerted) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PrecLand: Lean too high, holding");
            _lean_alerted = true;
        }
    }
    retry_pos_m = _hold_pos_ned_m;
}

bool AC_PrecLand_StateMachine::begin_miss_retry(Vector3p &retry_pos_m)
{
    AC_PrecLand *precland = AP::ac_precland();
    if (precland == nullptr) {
        return false;
    }
    if (_retry_count >= precland->get_max_retry_allowed()) {
        return false;
    }
    Vector3p pos_ned;
    if (!AP::ahrs().get_relative_position_NED_origin(pos_ned)) {
        return false;
    }

    _retry_count++;
    _lean_since_ms = 0;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Missed pin, retrying");
    precland->clear_final_drop_freeze();
    precland->reset_below_pad_timer();
    _miss_phase = MissPhase::CLIMB;
    // Latch the climb. Rebuilding "current height + 1.5 m" every frame would climb forever.
    _climb_pos_ned_m = pos_ned;
    _climb_pos_ned_m.z -= RETRY_OFFSET_ALT_M;
    retry_pos_m = _climb_pos_ned_m;
    return true;
}

AC_PrecLand_StateMachine::Status AC_PrecLand_StateMachine::begin_beside(Vector3p &retry_pos_m)
{
    AC_PrecLand *precland = AP::ac_precland();
    if (precland == nullptr) {
        return Status::ERROR;
    }
    if (!precland->has_last_target()) {
        return Status::FAILSAFE;
    }
    Vector3p pos_ned;
    if (!AP::ahrs().get_relative_position_NED_origin(pos_ned)) {
        return Status::ERROR;
    }

    _lean_since_ms = 0;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PrecLand: landing beside pad");
    precland->clear_final_drop_freeze();
    precland->reset_below_pad_timer();
    _miss_phase = MissPhase::NONE;
    _precland_suppressed = false;
    _climb_pos_ned_m = pos_ned;
    _climb_pos_ned_m.z -= RETRY_OFFSET_ALT_M;
    _beside_phase = MissPhase::CLIMB;
    retry_pos_m = _climb_pos_ned_m;
    return Status::RETRYING;
}

AC_PrecLand_StateMachine::Status AC_PrecLand_StateMachine::begin_abort(Vector3p &retry_pos_m)
{
    if (excessive_lean()) {
        begin_lean_hold(retry_pos_m);
        return Status::RETRYING;
    }
    if (begin_miss_retry(retry_pos_m)) {
        return Status::RETRYING;
    }
    return begin_beside(retry_pos_m);
}

// Climb in place, then move to the last target at the climb altitude, then hand back to precision landing.
AC_PrecLand_StateMachine::Status AC_PrecLand_StateMachine::update_miss_retry(Vector3p &retry_pos_m)
{
    AC_PrecLand *precland = AP::ac_precland();
    if (precland == nullptr) {
        return Status::ERROR;
    }
    Vector3p pos_ned;
    if (!AP::ahrs().get_relative_position_NED_origin(pos_ned)) {
        return Status::ERROR;
    }

    if (_miss_phase == MissPhase::CLIMB) {
        retry_pos_m = _climb_pos_ned_m;
        if (fabsf(float(pos_ned.z) - float(_climb_pos_ned_m.z)) >= PRECLAND_CLIMB_ARRIVE_M) {
            return Status::RETRYING;
        }
        Vector3p target_ned;
        precland->get_last_detected_landing_pos_NED_m(target_ned);
        _climb_pos_ned_m.x = target_ned.x;
        _climb_pos_ned_m.y = target_ned.y;
        _miss_phase = MissPhase::TRANSLATE;
    }

    if (_miss_phase == MissPhase::TRANSLATE) {
        retry_pos_m = _climb_pos_ned_m;
        const float dist_xy_m = Vector2f(float(_climb_pos_ned_m.x - pos_ned.x), float(_climb_pos_ned_m.y - pos_ned.y)).length();
        const float dist_z_m = fabsf(float(_climb_pos_ned_m.z) - float(pos_ned.z));
        if (dist_xy_m < MAX_POS_ERROR_M && dist_z_m < MAX_POS_ERROR_M) {
            _miss_phase = MissPhase::DESCEND;
            // The climb made the rangefinder increase. Do not treat that as another miss.
            precland->reset_below_pad_timer();
            return Status::DESCEND;
        }
        return Status::RETRYING;
    }

    return Status::DESCEND;
}

// Same climb-then-translate shape, then a vertical landing at PLND_SAFE_BRG / PLND_SAFE_DIST.
AC_PrecLand_StateMachine::Status AC_PrecLand_StateMachine::update_beside(Vector3p &retry_pos_m)
{
    AC_PrecLand *precland = AP::ac_precland();
    if (precland == nullptr) {
        return Status::ERROR;
    }
    Vector3p pos_ned;
    if (!AP::ahrs().get_relative_position_NED_origin(pos_ned)) {
        return Status::ERROR;
    }

    if (_beside_phase == MissPhase::CLIMB) {
        retry_pos_m = _climb_pos_ned_m;
        if (fabsf(float(pos_ned.z) - float(_climb_pos_ned_m.z)) >= PRECLAND_CLIMB_ARRIVE_M) {
            return Status::RETRYING;
        }
        Vector3p target_ned;
        precland->get_last_detected_landing_pos_NED_m(target_ned);
        const float bearing_rad = radians(precland->get_safe_brg_deg());
        const float dist_m = precland->get_safe_dist_m();
        // Bearing is clockwise from north: north = cos, east = sin.
        _climb_pos_ned_m.x = target_ned.x + dist_m * cosf(bearing_rad);
        _climb_pos_ned_m.y = target_ned.y + dist_m * sinf(bearing_rad);
        _beside_phase = MissPhase::TRANSLATE;
    }

    if (_beside_phase == MissPhase::TRANSLATE) {
        retry_pos_m = _climb_pos_ned_m;
        const float dist_xy_m = Vector2f(float(_climb_pos_ned_m.x - pos_ned.x), float(_climb_pos_ned_m.y - pos_ned.y)).length();
        const float dist_z_m = fabsf(float(_climb_pos_ned_m.z) - float(pos_ned.z));
        if (dist_xy_m < PRECLAND_BESIDE_ARRIVE_M && dist_z_m < PRECLAND_CLIMB_ARRIVE_M) {
            _beside_phase = MissPhase::DESCEND;
            _precland_suppressed = true;
            return Status::DESCEND;
        }
        return Status::RETRYING;
    }

    _precland_suppressed = true;
    return Status::DESCEND;
}

bool AC_PrecLand_StateMachine::get_beside_target_NE_m(Vector2p &pos_ne_m) const
{
    if (_beside_phase != MissPhase::DESCEND || !_precland_suppressed) {
        return false;
    }
    pos_ne_m.x = _climb_pos_ned_m.x;
    pos_ne_m.y = _climb_pos_ned_m.y;
    return true;
}

// This is only called when the current status of the state machine returns "failsafe" and will return the action that the vehicle should do
// At the moment this method only allows you to stop in air permanently, or land vertically
// Failsafe will only trigger as a last resort
AC_PrecLand_StateMachine::FailSafeAction AC_PrecLand_StateMachine::get_failsafe_actions()
{
    AC_PrecLand *_precland = AP::ac_precland();
    if (_precland == nullptr) {
        // should never happen, just descend
        return FailSafeAction::DESCEND;
    }

    if (!failsafe_initialized) {
        // start the timer
        failsafe_start_ms = AP_HAL::millis();
        failsafe_initialized = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Failsafe Measures");
    }

    // Depending on the strictness we will either land vertically, wait for some time and then land vertically, not land at all
    const RetryStrictness strictness= _precland->get_retry_strictness();
    switch (strictness) {
    case RetryStrictness::VERY_STRICT:
        // user does not want to land on anything but the target
        // stop landing (hover)
        return FailSafeAction::HOLD_POS;

    case RetryStrictness::NORMAL:
        if (AP_HAL::millis() - failsafe_start_ms < FAILSAFE_INIT_TIMEOUT_MS) {
            // stop the vehicle for at least a few seconds before descending
            // this might give user the chance to take over
            // we do not want to be too linent in landing vertically because of the strictness set by the user
            return FailSafeAction::HOLD_POS;
        }
        // land the vehicle vertically
        return FailSafeAction::DESCEND;

    case RetryStrictness::NOT_STRICT:
        // User wants to prioritize landing over staying in the air
        return FailSafeAction::DESCEND;
    }

    // should never reach here
    return FailSafeAction::DESCEND;
}

#endif // AC_PRECLAND_ENABLED
