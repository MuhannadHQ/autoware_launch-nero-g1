# NERO_* switches for the Nero code paths in autoware_universe-nero-g1 (and the bridge-side
# parking/pull-out), exactly as nero-jetson runs them. Each line's comment says what it does.
# Unset = the stock Autoware behaviour; delete or comment out a line to switch that change off.
#
# Sourced by ~/.autoware_env.sh on the vehicle, which the Autoware launcher GUI sources before every
# launch (nero-autoware-launcher-gui command_builder.DEFAULT_ENV_SETUP). Change it here, by PR, never
# by hand on the car: the vehicle picks it up at the next build from autoware.repos.
#
# Taken verbatim from nero-jetson ~/.autoware_env.sh on 2026-10-26 (after the 17:27 kit install),
# minus the RMW/CycloneDDS lines, which are machine settings and stay in that file.

# intersection (session 2 + 4)
export NERO_IX_ADJACENT_FIX=1 NERO_IX_EGO_ACC=0.35 NERO_IX_MERGE=1.0,2.0,2.0,6.0
export NERO_IX_CREEP=2.0,0.5,2.0,0.3,2.0,0.5,1.5 NERO_IX_CREEP_COMMIT_M=2.0
# dynamic_obstacle_stop (session 4)
export NERO_DOS_CREEP=2.0,0.5,2.0,0.3,2.0
# lane change (session 3)
export NERO_LC_MIN_SAFE_CYCLES=5
export NERO_LC_WAIT_DECEL=0.4 NERO_LC_WAIT_CRAWL=1.2 NERO_LC_WAIT_ROLL=15
export NERO_LC_INCH_SHIFT=0.3 NERO_LC_INCH_RAMP=30 NERO_LC_INCH_REACT=0.5 NERO_LC_INCH_DECEL=3.0
export NERO_LC_INCH_BACK_DECEL=4.5 NERO_LC_INCH_BACK_LEN=4.0
export NERO_LC_FRENET_JOIN_FIX=5.0
export NERO_LC_PRED_LAUNCH_ACC=0.33
export NERO_LC_HOLD_V=2.5 NERO_LC_HOLD_OVERHANG=0.0 NERO_LC_HOLD_SAFE_CYCLES=5
# lane change (session 4): keep right after the U-turn
export NERO_LC_SAUDI_DIR=1 NERO_LC_STOPPED_MIN_INTRUSION=0.5 NERO_LC_APPROVED_NO_DELAY=1 NERO_LC_YIELD_FRONT=1.5,1.5,2,0
# session 5: keep right, lane change direction / adjacency, static avoidance
export NERO_KEEP_RIGHT=1 NERO_KEEP_RIGHT_EARLY=1 NERO_LC_HOLD_DIR=1 NERO_LC_ADJ_LIMIT=1
export NERO_SOA_FIX_10794=1 NERO_SOA_NO_CHATTER=1 NERO_SOA_APPROVAL_KEEPS_SAFETY=1 NERO_SOA_HOLD_IN_LANE=1
export NERO_SOA_START_SAFE_CYCLES=10 NERO_SOA_STILL_SHARP_S=3 NERO_SOA_BUMP_V=1.0 NERO_SOA_GAP=0.4,1.0,0.25,10,3

# ---- overtake v6 2026-10-25 (static_obstacle_avoidance; v4's items of 2026-10-22 + the oncoming-late items of v5 + the pass-start yield;
#      DELIVERY.md in /work/vehicle-enhancement/passstart/package on NeroStation)
export NERO_SOA_CLOSE_VEHICLE_POLICY=auto          # plan the move from standstill next to a car (upstream #12105)
export NERO_SOA_AMBIGUOUS_IN_LANE=1                # a stopped car in our lane on the oncoming side may be overtaken
export NERO_SOA_AMBIGUOUS_STOP_LINE_M=20           # ... but not within 20 m before a stop line / junction
export NERO_SOA_AMBIGUOUS_QUEUE_M=15               # ... and not when queued behind another vehicle (15 m, as Apollo)
export NERO_SOA_SIDE_BY_SPACE=1                    # pass a stopped car / still object on the side that has room
export NERO_SOA_UNKNOWN_STABLE_S=3.0               # a still UNKNOWN is judged by its place, not its track id
export NERO_SOA_SAME_SIDE_RULE_HARMLESS=1          # the same-side rule vetoes only objects that limit the move
export NERO_SOA_ONCOMING_PAUSE=0.5                 # oncoming car during the shift: back 0.5 m inside the lane, stop before the parked car
export NERO_SOA_ONCOMING_COMMITTED=1.0,3.0         # beside / past it: go through a car that slows, stop at once for one that keeps coming
export NERO_SOA_ONCOMING_CORRIDOR=1                # do not pull out into an oncoming-lane corridor blocked by a standing object
export NERO_SOA_ONCOMING_START_YIELD=1             # unsafe in the last metres before the pull-out (a car just seen): wait in the lane
# [nero 2026-10-24] minor-road stop lines (intersection module, package minorroad_1024): where the robot's footprint at
# [nero 2026-10-24] the map stop line already touches the major road, the module still checks the crossing traffic and
# [nero 2026-10-24] holds the robot at the stop line (was: no check at all, "occlusion stop line is null")
export NERO_IX_PEEK_INVALID_KEEP_CHECK=2.0
# [nero 2026-10-24] silent junctions (intersection module, package silentjunction_1024): where the robot's footprint
# [nero 2026-10-24] never touches the first attention lane's centre line, the module still checks every attention lane,
# [nero 2026-10-24] stops before it and commits only once in the last one (was: no check at all, "failed to generate intersection_stoplines")
export NERO_IX_NO_CENTERLINE_KEEP_CHECK=1
# pullaway_1024 (2026-10-24): intersection module - a vehicle in an attention lane that starts from standstill is also
# checked as accelerating at 3.0 m/s2 along its lane (for up to 4 s while it keeps speeding up), not only at its current
# speed. Remove this line (and restart Autoware) to switch it off.
export NERO_IX_PULLAWAY_ACC=3.0
export NERO_KEEP_RIGHT_LANES=60,40
export NERO_LC_KEEP_RIGHT_GATE=120,1.0,50,30
# [nero 2026-10-24] parking in the registered spot (package parking_1024): Autoware's Parking scenario
# [nero 2026-10-24] (freespace planner); every line below acts only while Autoware parks
export NERO_COSTMAP_POINTS_FLOOR=1
export NERO_FREESPACE_HALF_CELL=1
export NERO_FREESPACE_GOAL_SLIDE_M=0.6
export NERO_FREESPACE_STRAIGHT_IN=1.2
export NERO_FREESPACE_STRAIGHT_IN_STOP=1
export NERO_FREESPACE_STRAIGHT_IN_WAIT_S=10
export NERO_FREESPACE_REPARK_DEG=3
export NERO_FREESPACE_PREDICTED_HOLD_S=8
export NERO_MISSION_ARRIVAL_WAIT_PARKING=1
# [nero 2026-10-25] pull-out from the registered spot (package pullout_1025, on top of parking_1024): every line acts
# [nero 2026-10-25] only while the robot starts in its parking lot / Autoware drives in its Parking scenario
export NERO_MISSION_PARKING_START_M=4
export NERO_FREESPACE_STRAIGHT_FWD_MIN_M=1.2
export NERO_FREESPACE_PREDICTED_HOLD_MOVING=1
export NERO_LC_DEBUG_LANES=0 NERO_LC_ABL_IDLE_GUARD=1 NERO_LC_FRENET_CACHE=1 NERO_PO_FIX_10726=1   # mrm_1025: planner cycle at a lane-change wait (MRM 10-25 12:43)
# tjswing_1025 (T-junction priority rule, 2026-10-25): oncoming cars that must yield at a T junction are judged from their motion
export NERO_IX_PRIORITY_FROM_SIGNS=1
export NERO_IX_PRIORITY_CAN_STOP=1.5
# Ahmed's 13:2x rule (braking alone, without turning, does not make the robot yield; answered 10-25 "loosen it up"); commented out = tjviolator_1024
export NERO_IX_PRIORITY_SWING=12
export NERO_IX_STANDSTILL_REJUDGE=1.5,0.5
export NERO_SOA_GAP_FAST=1 NERO_SOA_REAR_START_YIELD=1 NERO_SOA_RETURN_AWAY=1   # yieldmrm_1026: the gap search without repeated work (the MRMs of 10-25 20:29 / 20:47) + wait in the lane for a car coming up behind + return away from it
export NERO_FREESPACE_MIN_SEGMENT=1.5,1.0,0.5,20 NERO_FREESPACE_STALL_S=5 NERO_FREESPACE_PARKED_BOX=0.6,0.9,15 NERO_FREESPACE_START_ESCAPE=1,3   # parkfix_1025b: parking moves the car can start, stall guard, parked by footprint, leave from a pose inside the margin (only once it stayed blocked 3 s)

# ---- 2026-10-27 ready set (universe feature/vehicle-planning-2026-10-27; packages from NeroStation /work/vehicle-enhancement/<job>/package)
# ixghost_1026: a flat UNKNOWN (never reported >= 0.2 m tall) reporting < 3.5 m/s is not a collision target of the intersection module
export NERO_IX_UNKNOWN_FLAT=0.2,3.5
export NERO_SOA_RETURN_NO_NEED=1 NERO_SOA_LANE_BLOCKED=1 NERO_SOA_RETURN_CLEAR=1   # narrowweave_1026 (= returnshift_1026 fixed + the clear way back): returns / passes clear of standing objects
export NERO_LC_ROUTE_GATE=1 NERO_LC_FIX_10819=1 NERO_LC_FRENET_ROOM=1 NERO_LC_STOP_COMFORT=1.0 NERO_LC_SHORT_WINDOW=1 NERO_LC_STOP_AT_TARGET=1.0 NERO_LC_FIX_11545=1 NERO_LC_MULTI_STOP_OFFSET=2.0 NERO_LC_MULTI_WAIT_CRAWL=2.5 NERO_LC_MULTI_YIELD_FRONT=1 NERO_LC_WAIT_COMFORT=1.0 NERO_LC_INCH_CLEAR=1.0 NERO_LC_DRIVABLE_DEDUP=1 NERO_LC_TRAILING_LIKELY=0.3 NERO_LC_MULTI_STAND_OFFSET=10 NERO_LC_MULTI_STAND_PREPARE=1 NERO_LC_DELAY_NO_ROOM=0 NERO_LC_MULTI_ROLL=1 NERO_LC_SIGNALLED_PREPARE=1 NERO_LC_MULTI_FRONT_HEADWAY=1 NERO_LC_CONT_SHORT_PREPARE=0.0 NERO_LC_CONT_FINISH=0.3,3   # streamjoin_1027 (replaces lcfinish_1027 / routelc_1026): route-required lane changes + joining a stream from the waiting place (lane change module)
export NERO_CRUISE_CUTIN_YIELD=3.0,12,1.2,2.0,2.0,2 NERO_CRUISE_CUTIN_STEEP=40,90,3.0,6.0,2.5   # cutin_1027: obstacle_cruise eases off for a car squeezing in beside / just ahead, brakes for a car turning in from the side that can no longer stop (QUEUE CUT)
export NERO_OSD_NOT_IN_WAY=3.0,1.1,30,0.2,0.3,1.0,0.5 NERO_CRUISE_YIELD_GAIN=0.2,0.5   # mergeahead_1027: obstacle_slow_down no longer slows for a moving vehicle that is not in the way (separating / keeping its lane beside), cutin_1027's squeeze yield brakes at its maximum only if that buys clearance (QUEUE SPD / CUTY)
export NERO_CRUISE_CUTIN_EARLY=1.5,0.5,0.5,0.05,0.5,1.0,2.5 NERO_CRUISE_CUTIN_TAPER=0.1,0.05   # cutinyield_1027: obstacle_cruise's squeeze yield starts early and shallow for a faster car moving in from beside (measured against its own lane), and asks only the braking that buys clearance (no crawl for a car that passes and pulls away; QUEUE C56M)
export NERO_IX_SIGHT=70,1.5,2.0,0.8,0.4,3.0,12.0,0.5,0.5   # leftsight_1027: at a stop-sign turn across a priority lane, wait for a car the lidar grid sees before perception publishes it; never stop inside that lane for a car it can clear (QUEUE LTP)
# give-way off (Ahmed 2026-11-01 13:5x: "turn off the giveway its not ready yet"): export NERO_SOA_CORRIDOR_GIVEWAY=6.0,12.0,0.3,1.0   # corridor_1027: static avoidance gives way to a car standing in / coming down the free width of a narrow street (keep right, wait or pass slowly, then go on)
export NERO_SOA_PATH_FROM_EGO=0.1,0.3   # pathstep_1028: static avoidance never steps its plan sideways under the robot (a jump > 0.1 m or the robot > 0.3 m off starts the plan from the robot)
export NERO_IX_CROSSING_HOLD=2.0,12.0,2.0,1.0,0.5,0.5,0.5,0.5,25,3.0,5.0,2.0   # crosshold_1028: at a stop-sign lane without a mapped crossing road, hold at the line while traffic is predicted across the robot's way
export NERO_SOA_ROUTE_LC=28 NERO_LC_MULTI_HOLD_V=6.0,3.0   # lca_1028: a car parked near a lane end is left to the route's own lane change; a double change is held in lane while the strict check says unsafe
export NERO_SCSEL_FAR_STOP=0.72,0.8,0.7,0.45,1.0,5.56,2.4,0.5,0.65   # farstop_1028: a stop the comfort profile can still make is braked earlier and gentler; stops needing more keep today's braking
export NERO_USB2_HUB=1   # usb2_hub (sensor kit sensing.launch.xml): run the external USB hub (PCAN, Xsens, Kingston) at USB 2.0; its USB 3 link jams the GNSS
export NERO_SOA_PROJ_FAST=1   # trajgap_1028: static avoidance projects its NERO loops' points without copying the path per call (the same numbers; the five-fixes build fell under 5 Hz -> MRMs)
# give-way off (Ahmed 2026-11-01 13:5x: "turn off the giveway its not ready yet"): export NERO_SOA_CORRIDOR_GIVEWAY_TWOWAY=0.7,2.78,2.5,0.8,1.1   # twoway_1029: corridor_1027's give-way on an ordinary two-way street: no slow-down for an oncoming car whose line stays >= 0.7 m off the robot's keep-right body (keep right for 1.2 m where it can), the pass speed from the room it leaves; only real parked rows; the one-lane corridor unchanged
export NERO_SOA_DECEL_SOFT=1 NERO_MOT_REBIRTH=2,0.5,3.0,1.0   # tbx_1029: a parked car whose box comes late is passed with a shift that keeps the hard margin instead of a hard static-avoidance stop; the tracker continues a standing UNKNOWN as the vehicle it becomes (one object, published sooner)
export NERO_IX_CROSSING_HOLD_YIELD=2.0,0.5,12   # crosswait_1029: crosshold_1028's slow-mover wait lets the robot go when the vehicle stopped near the mouth (2 s) or is behind it pointing away; one that moves again holds / stops the robot again (QUEUE STK1846)
export NERO_VEL_LIMIT_CLEAR_DEFAULT=1   # vlz_1029: autoware_external_velocity_limit_selector, a clear on an empty table: the first clear after an Autoware start (the bridge's link watchdog at the engage) gives max_vel instead of the never-set 0 km/h (upstream #11463; QUEUE VLZ)
export NERO_POSE_INIT_GATE=2.3,0.1 NERO_NDT_ALIGN_YAW_WINDOW=1.0,30   # linit_1029: an operator / reinitializer init whose NDT align is unreliable, or worse than a healthy running localization, is refused and the localization kept; a set-pose keeps its heading (+-30 deg)
export NERO_E1R_NEAR_WEAK=1.5,3,0.07,10   # aebnear_1028: the front E1R drops returns < 1.5 m from it that are weak (intensity <= 3) AND isolated (<= 10 other returns within 7 % of their range): dust / near-range noise, never a surface (the 10-27 20:47:49.7 phantom AEB stop; QUEUE AEBD). Sensing only.
export NERO_MOT_SPAWN_PRUNE=1   # flatos_1029: a new UNKNOWN track inside a moving vehicle's box at its measurement time is a piece of that vehicle (its left-behind rear) and is removed
export NERO_CP_SHUFFLE_FIX=1   # cpshuffle_1031: perception lidar_centerpoint: the network sees every lidar point of the frame (upstream #11711 shuffle index check); today it sees ~12 % of them plus zeros and stale points of earlier frames (ghosts at old places): cars recognised much further out (10-27 20:47 replay: an oncoming car first at 56 m instead of 18 m, confident boxes x1.8, beyond 30 m x3.3)
export NERO_IX_CROSSING_HOLD_CREEP=0.5,2.0,1,1 NERO_IX_CROSSING_HOLD_VANISHED=2.0,1.0 NERO_IX_CROSSING_HOLD_AGAIN_COMMIT=1   # latecrosser_1031: intersection module, crosshold: after a hold the robot leaves at <= 0.5 m/s until its nose is 2.0 m past the line (a crossing car published late can still be stopped for clear of its line), except where only the creep would hold it in a stream gap or a driver stands waiting for it; a moving vehicle that vanishes beside the robot's way is judged as moving on for <= 2.0 s; a moving robot committed for a car is not stopped inside that car's way
export NERO_MOT_UNKNOWN_MOVER=3.0,0.3,0.5,4,1.0,0.5,3.0,0.6,2   # oncomingtrack_1031 + crossmover_1101: multi_object_tracker: an oncoming car the clustering already sees as an UNKNOWN is tracked by its own measured positions (4 on a straight line within 0.5 s, >= 3 m/s, against the robot's heading) and reaches the planners moving at its speed from there; CenterPoint's later CAR continues the same track (10-27 20:47:43 / 47: from 31 / 36 m instead of 10.7 m). crossmover_1101 (fields 7-9): the same for a car crossing the robot's way (60-120 deg, >= 3 m/s, steady leading end, 2 passing tests): published moving instead of ~0 m/s (10-29 15:51:49 51377 'why didnt it stop')
export NERO_SOA_RETURN_CLEAR_PLACE=1.0,1.0,2.0,1,0.4   # kerbhold_1031: behavior_path_planner static_obstacle_avoidance: narrowweave's RETURN_CLEAR hold remembers the standing object it held for by PLACE (any track id, the largest outline seen there) until the robot's rear is 1 m past it, checks it also once back in the lane, eases toward the lane instead of staying out, never steps toward a car alongside (10-27 20:47 #7: a kerb fragment changed id, the hold let go, return 0.32 m from it, operator + obstacle_stop STOP)
export NERO_GROUND_BAND_ROAD=0.52,0.15,0.3,0.15,0.3,2.0,30,40 NERO_GROUND_BAND_ROAD_DEBUG=1   # rampground_1031: perception ground_segmentation scan_ground_filter: on a ramp / crest the obstacle band ends at the road ahead + the crop height (0.52) as on level ground, not at the tilted body: within 30 m and 40 deg ahead, points > 0.67 m over the road with nothing under them are not obstacles (10-27 20:48:03.8 obstacle_stop 17 m for a branch over the lane); _DEBUG=1: a log line per frame where it acts (first drive)
export NERO_MOT_FLIP_DEFER=1.0,-0.5,1.0,0.2,1.0   # flipspeed_1030: multi_object_tracker: a vehicle track whose speed suddenly turns reverse is not turned 180 deg at once; it stands at its heading until 0.2 s of measured positions show it moving (then as stock): no runaway ghost on a standing truck (10-29 17:07 #5 obstacle_cruise 0 km/h)
export NERO_PID_SLOPE_STOP=0.02,0.25,1.33,3.0   # slopestop_1029: pid_longitudinal_controller, a stop on a down-grade (measured pitch below -2 %, full from -4 %) adds 1.33 x g sin(grade) to the brake while stopping (and in the last 3 m while the robot runs ahead of the plan): it no longer speeds up through its stop point (10-29 51377)
# give-way off (Ahmed 2026-11-01 13:5x: "turn off the giveway its not ready yet"): export NERO_SOA_CORRIDOR_GIVEWAY_CRAMPED=1.5,0.3,1.0,2.5,4.0,1.2,6.0,0.2,30,1,1,0.5   # weave_1029: the give-way keeps right ONCE per stream of oncoming cars (stays 6 s after the last, one smooth return), only for cars heading within 30 deg of head-on, no early wait for far / slow cars, no hard stop on box jitter, never a line out into the oncoming lane while giving way
# give-way off (Ahmed 2026-11-01 13:5x: "turn off the giveway its not ready yet"): export NERO_SOA_CORRIDOR_GIVEWAY_PARKED=1   # weave_1029: while keeping right for an oncoming car, parked cars on the right keep the module's own margins (never inside its hard margin)
export NERO_SOA_AMBIGUOUS_FRAGMENT=0.5,1.0,0.1   # retaa_1030: a standing car first reported as an UNKNOWN fragment keeps the fragment's standing history (a target at once, moved distance from its own place); the fragment's line counts as approved for it only if it keeps the car's hard margin + 0.1 m, else the module shifts, or stops before it (firm, >= -2.0 m/s2) until a line does (QUEUE RETAA)
export NERO_IX_PRIORITY_NO_EXIT=1.3   # neverslow_1030: intersection module, at a stop-sign T mouth whose every way on is a turn, a car faster than all of them, not braking, that cannot stop at its line is a violator from first sight: the robot stops 1.3 m clear of its straight path (10-29 15:55 #3, 0.51 m)
export NERO_IX_CROSSING_HOLD_FOLLOWER=1.2,25   # followturn_1030: intersection module, crosshold: a car behind the robot along its own trail through a stop-sign turn (within 1.2 m of the trail, heading within 25 deg of it) is a follower, not a crosser: no stop mid-turn for the car following it (10-29 15:57 #5, 3.0 s at -3.4)
export NERO_IX_CROSSING_HOLD_STOOD=2.0,0.1,1.0,1.3   # besidebus_1030: intersection module, crosshold: a vehicle that stood beside the robot's approach counts as moving (crosser, no-start, slow mover, re-arm) only once both ends of its box move beyond its own noise: no stop mid-turn for a standing bus whose tracker box / speed jitters (10-29 20:47 #17, stopped at +0.5 m)
# give-way off (Ahmed 2026-11-01 13:5x: "turn off the giveway its not ready yet"): export NERO_SOA_CORRIDOR_GIVEWAY_ROWSIDE=1   # cgwside_1101: static avoidance corridor give-way: a parked object standing between an oncoming car and the robot's lane does not push that car toward the robot (a parked car far beside a turn exit was taken as an oncoming car and the plan left the lanes)
export NERO_IX_YIELD_INTENT=2.0,6.0,0.5,0.3,1.5,1.0,0.3,6,1.0,0.5   # yieldsignal_1101: intersection module: a car that braked and now stands / crawls clear of the robot's way while the robot waits is letting it go: the robot leaves without the 1.5 s collision hold (mapped junctions) or crosswait's 2 s wait (stop-sign mouths); judged again every cycle, revoked the moment that car speeds up (QUEUE YSIG)
export NERO_VAL_BODY=0.25,1.0,10,0.8,0,0,0.3,25,3,2.5,0,25,0.1,0.5,3,7,45   # valrange_1101: perception obstacle_pointcloud_based_validator: a CenterPoint vehicle box whose 0.52 m obstacle band is empty (ground filter / raised road / far range) is still validated by its lower body in the raw lidar cloud (>= 10 points 0.25-1.0 m over its own bottom, >= 7 from 45 m, spread >= 0.8 m; within 25 m >= 0.3 m in height; beyond 25 m also >= 3 points 0.1-0.5 m over its bottom = bumper / sills / wheels, which a bar or branch over the road does not have), and only if a vehicle box stood within 2.5 m in one of the previous 3 frames (one-frame boxes on a kerb / shrub / cabinet never validate); boxes the stock count keeps are unchanged. Far oncoming and standing cars reach the tracker from about CenterPoint's first frames (10-29 trips: median 42.0 m instead of 34.6 m). The validator then runs inside /pointcloud_container (raw cloud intra-process)
export NERO_PID_START_LEAD=0.6,3.0,6.0,-1.5   # pulloff_1101: pid_longitudinal_controller, a departure from a stop in LaneDriving: the velocity target is the plan's own speed 0.6 s ahead (faded out by 3 m/s; only when higher than the car's target; not with a stop point within 6 m); a departure the plan takes back stops at <= -1.5 m/s2; a place where a lead departure was cut short gets the car's start until the robot has left (QUEUE PULLOFF)
export NERO_PID_ROLLING_RESET=1   # pulloff_1101: pid_longitudinal_controller, upstream #10684 (d1b44201c): a release while still rolling no longer keeps the brake on for ~1.2-1.7 s (QUEUE PULLOFF)
export NERO_MPC_CONVERGE_WINDOW=0.3   # pulloff_1101: mpc_lateral_controller, the steering-steadiness window before the robot may start moves from 1.0 s to 0.3 s (same 0.01 rad test) in LaneDriving; Parking keeps 1.0 s (QUEUE PULLOFF)
# stagewatch_1102 (trajstall, QUEUE TRJ2, 2026-11-02): a read-only observer of the planning chain. When the trajectory
# stalls (the 11-01 MRMs) it names the stage and what the planning processes' threads did: [NERO_STAGE_WATCH] lines in
# /rosout (captures). Publishes nothing. Needs the universe commit (package autoware_nero_stage_watch + the group in
# tier4_planning_launch planning.launch.xml). Rollback: comment this line out (the node is then not launched).
export NERO_STAGE_WATCH=1
export NERO_DBT_VISIBLE_END=0.3   # parkghost_1102: perception detection_by_tracker: a vehicle box made from the tracker's merged clusters may not stand more than 0.3 m in front of the vehicle's visible end on the side facing the lidar (else it slides back along its length, <= 1.5 m), and points of a tracked pedestrian / bicycle / motorcycle are not used for a vehicle's box: a parked car's track is no longer walked forward by its own prediction while CenterPoint misses it (11-02 12:53 phantom across the road, QUEUE GHOSTX)
# bppstall_1103 (bppstall, QUEUE TRJ2, 2026-11-03): behavior_path_planner's cycle without per-call path copies (11-02 13:24-13:28: BPP at
# 3-7 Hz in the 72467 -> 44311 street and at the 153664 lane-change hold -> trajectory-rate WARN -> emergency stops). The same numbers
# (check mode: 0 mismatches). Needs the universe commit (BPP common + static avoidance + lane change). Rollback: comment this line out.
export NERO_BPP_PROJ_FAST=1 NERO_LC_FRENET_REF_MEMO=1
# lidphase_1104 (QUEUE LIDPHASE / CONCATPHASE, 2026-11-04): the lidar concat node groups the four scans by their stamps (advanced
# matching, offsets 0, +-0.04 s noise window per lidar) instead of by arrival. With the gPTP lock (nero-ptp4l-lidar, applied 11-04
# 15:49) the four scans start together and group exactly; it also ends the next-scan pairing after a lost scan (car 11-04 17:0x:
# 1 % of groups > 80 ms, 3 missing of 302). Offline on window 074: locked span 0.3 ms / age 114 ms, pairing 0 %; unlocked == the
# car (span 75.7 / age 182.6, no split). Needs launch 9b20dba. Rollback: comment this line out (unset = naive, as before).
export NERO_LIDAR_CONCAT_MATCH=advanced,0.04
export NERO_MOT_STAND_SPEED=0,2.0,1   # vstoplag_1104: multi_object_tracker: the PUBLISHED speed of a vehicle whose boxes stand is 0 (a hard stop read at once instead of a slowly decaying tail) and a car starting after a stand is published at its measured speed (11-02 #28: a hard stop read ~1.2 s late); output only, the tracker's filter, association and positions untouched
export NERO_MOT_FAST_BIRTH=3,0.15,0,3.0,30,1.0   # realize_1104: multi_object_tracker: a vehicle track whose measured boxes move >= 3 m/s within 30 deg of their heading, with no published vehicle within half lengths + 1.0 m, is published from 3 distinct detection frames (the last within 0.15 s) instead of the stock count; the filter, association, states and positions are the car's (publication only); needs vstoplag_1104 (QUEUE REALIZE)
export NERO_VAL_MOVER=4,0.7,0.3,3.0,30,1.0,0,0,3,4000,0.35,20,3,0.3,0.8,4.5,2.0 NERO_VAL_PED=5,4000,0.9,6,1.5,0.8,0.25,0.5,8,1.0,0.5,2.2,3.5,0.5,2,0.15   # realize_1104: perception obstacle_pointcloud_based_validator: a CenterPoint vehicle box the point count drops is kept when its own CenterPoint chain moves on a line (>= 4 boxes in 0.7 s, >= 3 m/s, within 30 deg of its heading) on a road-standing raw body that left its place 0.3 s earlier (no twin overlapping it across); a dropped pedestrian / bicycle box with a raw body that walks on a line or stands isolated is kept (people tracked from ~25 m instead of ~12 m); far oncoming / crossing cars reach the tracker 0.3-2 s sooner (QUEUE REALIZE, VALPED)
export NERO_MOT_SIGN_BELIEF=3.0,0.25,0.5,1.0,30,0.1,1.389,10.0,1.3,0.2,0,0,0,4.0,0.8   # signbelief_1106: perception multi_object_tracker: a vehicle track's heading sign (front / back) from the votes of all its boxes and its measured motion (port of upstream #13050) instead of its first box; the only 180 deg turn of a vehicle track (NERO_MOT_FLIP_DEFER then only holds a sudden reverse speed at 0 while 0.2 s of positions decide): parked cars no longer shown backwards for life, no mirrored newborn walked toward the path (11-04 b38, 11-02 GHOSTX)
