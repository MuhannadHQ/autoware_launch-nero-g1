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
export NERO_SOA_CORRIDOR_GIVEWAY=6.0,12.0,0.3,1.0   # corridor_1027: static avoidance gives way to a car standing in / coming down the free width of a narrow street (keep right, wait or pass slowly, then go on)
export NERO_SOA_PATH_FROM_EGO=0.1,0.3   # pathstep_1028: static avoidance never steps its plan sideways under the robot (a jump > 0.1 m or the robot > 0.3 m off starts the plan from the robot)
export NERO_IX_CROSSING_HOLD=2.0,12.0,2.0,1.0,0.5,0.5,0.5,0.5,25,3.0,5.0,2.0   # crosshold_1028: at a stop-sign lane without a mapped crossing road, hold at the line while traffic is predicted across the robot's way
export NERO_SOA_ROUTE_LC=28 NERO_LC_MULTI_HOLD_V=6.0,3.0   # lca_1028: a car parked near a lane end is left to the route's own lane change; a double change is held in lane while the strict check says unsafe
export NERO_SCSEL_FAR_STOP=0.72,0.8,0.7,0.45,1.0,5.56,2.4,0.5,0.65   # farstop_1028: a stop the comfort profile can still make is braked earlier and gentler; stops needing more keep today's braking
export NERO_USB2_HUB=1   # usb2_hub (sensor kit sensing.launch.xml): run the external USB hub (PCAN, Xsens, Kingston) at USB 2.0; its USB 3 link jams the GNSS
export NERO_SOA_PROJ_FAST=1   # trajgap_1028: static avoidance projects its NERO loops' points without copying the path per call (the same numbers; the five-fixes build fell under 5 Hz -> MRMs)
export NERO_SOA_CORRIDOR_GIVEWAY_TWOWAY=0.7,2.78,2.5,0.8,1.1   # twoway_1029: corridor_1027's give-way on an ordinary two-way street: no slow-down for an oncoming car whose line stays >= 0.7 m off the robot's keep-right body (keep right for 1.2 m where it can), the pass speed from the room it leaves; only real parked rows; the one-lane corridor unchanged
export NERO_SOA_DECEL_SOFT=1 NERO_MOT_REBIRTH=2,0.5,3.0,1.0   # tbx_1029: a parked car whose box comes late is passed with a shift that keeps the hard margin instead of a hard static-avoidance stop; the tracker continues a standing UNKNOWN as the vehicle it becomes (one object, published sooner)
export NERO_IX_CROSSING_HOLD_YIELD=2.0,0.5,12   # crosswait_1029: crosshold_1028's slow-mover wait lets the robot go when the vehicle stopped near the mouth (2 s) or is behind it pointing away; one that moves again holds / stops the robot again (QUEUE STK1846)
export NERO_VEL_LIMIT_CLEAR_DEFAULT=1   # vlz_1029: autoware_external_velocity_limit_selector, a clear on an empty table: the first clear after an Autoware start (the bridge's link watchdog at the engage) gives max_vel instead of the never-set 0 km/h (upstream #11463; QUEUE VLZ)
export NERO_POSE_INIT_GATE=2.3,0.1 NERO_NDT_ALIGN_YAW_WINDOW=1.0,30   # linit_1029: an operator / reinitializer init whose NDT align is unreliable, or worse than a healthy running localization, is refused and the localization kept; a set-pose keeps its heading (+-30 deg)
