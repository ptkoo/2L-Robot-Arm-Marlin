/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * llrobotarm.cpp
 */

#include "../inc/MarlinConfig.h"

#if IS_ROBOT_ARM_2L

#include "robotarm2l.h"
#include "motion.h"
#include "planner.h"
#include "endstops.h"

float delta_segments_per_second = ROBOT_ARM_2L_SEGMENTS_PER_SECOND;
float rot, high, low;

void ROBOT_ARM_2L_set_axis_is_at_home(const AxisEnum axis) {
    SERIAL_ECHOLNPAIR(
      "ROBOT_ARM_2L ROBOT_ARM_2L_set_axis_is_at_home a=", axis
    );
    /**
     * ROBOT_ARM_2L homes XYZ at the same time
     */
    xyz_pos_t homeposition;
    LOOP_XYZ(i) homeposition[i] = base_home_pos((AxisEnum)i);

    
      SERIAL_ECHOLNPAIR(
      "ROBOT_ARM_2L_set_axis_is_at_home homeposition.x=", homeposition.x,
      " homeposition.y=", homeposition.y,
      " homeposition.z=", homeposition.z
    );
      inverse_kinematics(homeposition);
      forward_kinematics_ROBOT_ARM_2L(delta.a, delta.b, delta.c);
      current_position[axis] = cartes[axis];
    

    // SERIAL_ECHOPGM("Cartesian");
    // SERIAL_ECHOLNPAIR_P(SP_X_LBL, current_position.x, SP_Y_LBL, current_position.y);
    update_software_endstops(axis);
}

static constexpr xyz_pos_t ROBOT_ARM_2L_offset = { ROBOT_ARM_2L_OFFSET_X, ROBOT_ARM_2L_OFFSET_Y, ROBOT_ARM_2L_OFFSET_Z };

/**
 * ROBOT_ARM_2L Forward Kinematics. Results in 'cartes'.
 */
//2L TODO: split linkages to have different length, now assuming the same
void forward_kinematics_ROBOT_ARM_2L(const float &a, const float &b, const float &c) {
  
  const float rot_sin = sin(RADIANS(a)),
              rot_cos = cos(RADIANS(a)),
              low_sin = sin(RADIANS(b)),
              low_cos = cos(RADIANS(b)),
              high_sin = sin(RADIANS(c)),
              high_cos = cos(RADIANS(c)),
              h_min_l_sin = sin(RADIANS(c-b)),
              h_min_l_cos = cos(RADIANS(c-b));

  float rot_ee = ROBOT_ARM_2L_LINKAGE * (low_sin + h_min_l_sin) + ROBOT_ARM_2L_EE_OFFSET;

  float y = rot_ee * rot_sin;

  float x = rot_ee * rot_cos;

  float z = ROBOT_ARM_2L_LINKAGE * (low_cos - h_min_l_cos);

  cartes.set(x + ROBOT_ARM_2L_offset.x, y + ROBOT_ARM_2L_offset.y, z + ROBOT_ARM_2L_offset.z);

  
   /* SERIAL_ECHOLNPAIR(
      "ROBOT_ARM_2L FK x=", x,
      " y=", y,
      " z=", z
    );*/
   // SERIAL_ECHOLNPAIR(" cartes (X,Y) = "(cartes.x, ", ", cartes.y, ")"));
  
}

void inverse_kinematics(const xyz_pos_t &raw) {
  /*SERIAL_ECHOLNPAIR(
      "ROBOT_ARM_2L IK raw.x=", raw.x,
      " raw.y=", raw.y,
      " raw.z=", raw.z
    );*/

 
  float rrot =  hypot(raw.x, raw.y) - ROBOT_ARM_2L_EE_OFFSET;    //radius from Top View
  float rrot_ee = rrot + ROBOT_ARM_2L_EE_OFFSET;
  float rside = hypot(rrot, raw.z);  //radius from Side View.

  //if(x > 0) { 
    rot = acos(raw.x/ rrot_ee);
  //} else {
  //  rot = 
  //}
  high = acos((rside * 0.5) / ROBOT_ARM_2L_LINKAGE) * 2.0;
   
  //Angle of Lower Stepper Motor  (asin()=Angle To Gripper)
  if (raw.z > 0) {
    low =      asin(rrot / rside) + ((PI - high) / 2.0) - (PI / 2.0);
  } else {
    low = PI - asin(rrot / rside) + ((PI - high) / 2.0) - (PI / 2.0);
  }
  //correct higher Angle as it is mechanically bounded width lower Motor
  high = high + low;

  delta.set(DEGREES(rot), DEGREES(low), DEGREES(high));

  
  //SERIAL_ECHOLNPAIR("ROBOT_ARM_2L IK:", raw);
  //SERIAL_ECHOLNPAIR("ROBOT_ARM_2L IK:", delta);
  //SERIAL_ECHOLNPAIR("ROBOT_ARM_2L (rot,lowhigh) ", rot, ",", low, ",", high);
  /*SERIAL_ECHOLNPAIR(
      "ROBOT_ARM_2L IK Angle rot=", rot,
      " low=", low,
      " high=", high
    );
  SERIAL_ECHOLNPAIR(
      "ROBOT_ARM_2L IK delta.x=", delta.x,
      " delta.y=", delta.y,
      " delta.z=", delta.z
    );*/
  

}

void ROBOT_ARM_2L_report_positions() {
  SERIAL_ECHOLNPAIR("ROBOT_ARM_2L rot:", planner.get_axis_position_degrees(A_AXIS), "  low", planner.get_axis_position_degrees(B_AXIS), " high: ", planner.get_axis_position_degrees(C_AXIS));
  SERIAL_EOL();
}


void home_ROBOT_ARM_2L() {
    /*if(current_position.z < 0) {
      xyz_float_t safe_start_position = {current_position.x, current_position.y, 100};
      destination = safe_start_position;
      prepare_line_to_destination();
      planner.synchronize();
    }*/
    disable_all_steppers();

    homeaxis(Y_AXIS);
    homeaxis(Z_AXIS);  
    homeaxis(X_AXIS);  
    
    constexpr xyz_float_t endstop_backoff = {ROBOT_ARM_2L_X_AT_ENDSTOP, ROBOT_ARM_2L_Y_AT_ENDSTOP, ROBOT_ARM_2L_Z_AT_ENDSTOP};
    current_position = endstop_backoff;

    sync_plan_position();

    move_after_homing_ROBOT_ARM_2L();

    //TERN_(IMPROVE_HOMING_RELIABILITY, end_slow_homing(slow_homing));
}


void move_after_homing_ROBOT_ARM_2L() {
  constexpr xyz_float_t endstop_backoff = {MANUAL_X_HOME_POS, MANUAL_Y_HOME_POS, MANUAL_Z_HOME_POS};
  //do_blocking_move_to(endstop_backoff, HOMING_FEEDRATE_XY);
  current_position.set(MANUAL_X_HOME_POS, MANUAL_Y_HOME_POS, MANUAL_Z_HOME_POS);
  line_to_current_position(100);
  sync_plan_position();

}

bool position_is_reachable_ROBOT_ARM_2L(const float &rx, const float &ry, const float &rz, const float inset) {
      //SERIAL_ECHOPAIR("position_is_reachable? rx: ", rx, ", ry: ", ry, ", rz:", rz, "\n");

      float rrot =  hypot(rx, ry) - ROBOT_ARM_2L_EE_OFFSET;    //radius from Top View
      float rrot_ee = rrot + ROBOT_ARM_2L_EE_OFFSET;
      float rside = hypot(rrot, rz);  //radius from Side View. 

      //2L TODO check inset
      bool retVal = (
          rside <= (ROBOT_ARM_2L_MAX_RADIUS - inset) &&
          rside >= ROBOT_ARM_2L_MIN_RADIUS && 
          rz >= Z_MIN_POS && 
          rz <= Z_MAX_POS
        );
      //if(!retVal) {
        //SERIAL_ECHOPAIR("rside:  ", rside, ", RMAX:", ROBOT_ARM_2L_MAX_RADIUS, ", RMIN", ROBOT_ARM_2L_MIN_RADIUS, 
        //              ", ROBOT_ARM_2L_Z_MIN: ", Z_MIN_POS, ", ROBOT_ARM_2L_Z_MAX",Z_MAX_POS,"\n");
      //}
      //SERIAL_ECHOPAIR("position_is_reachable: ", retVal, "\n");
      return retVal;
}

#endif // IS_ROBOT_ARM_2L