/**
 ******************************************************************************
 * AprilTagPoseSelfTest.ino
 *
 * Projects the four corners of a known tag pose through a known camera and
 * checks that the pose solver recovers the pose within 2 mm and 0.5 deg. Each
 * case prints ok or FAIL, and the RESULT line gives the number of failures.
 *
 * Hardware: Neuro N6.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#include <n6_apriltag_pose.h>
#include <math.h>

#define DEG (3.14159265358979f / 180.0f)

static void rot_xyz(float rx, float ry, float rz, float R[9])
{
  const float cx = cosf(rx*DEG), sx = sinf(rx*DEG);
  const float cy = cosf(ry*DEG), sy = sinf(ry*DEG);
  const float cz = cosf(rz*DEG), sz = sinf(rz*DEG);
  R[0]=cz*cy; R[1]=cz*sy*sx - sz*cx; R[2]=cz*sy*cx + sz*sx;
  R[3]=sz*cy; R[4]=sz*sy*sx + cz*cx; R[5]=sz*sy*cx - cz*sx;
  R[6]=-sy;   R[7]=cy*sx;            R[8]=cy*cx;
}

static int failures = 0;

static void check(const char *name, float tx, float ty, float tz,
                  float rx, float ry, float rz, float size)
{
  const int W = 640, H = 480;

  n6_camera_cal_t cal;
  memset(&cal, 0, sizeof(cal));
  cal.magic = N6_CAMERA_CAL_MAGIC; cal.version = N6_CAMERA_CAL_VERSION;
  cal.width = W; cal.height = H;
  cal.fx = 520.0f; cal.fy = 518.0f; cal.cx = W * 0.5f; cal.cy = H * 0.5f;

  float R[9]; rot_xyz(rx, ry, rz, R);
  const float s = size * 0.5f;
  const float obj[4][2] = { {-s, s}, {s, s}, {s, -s}, {-s, -s} };

  n6_apriltag_t tag;
  memset(&tag, 0, sizeof(tag));
  for (int i = 0; i < 4; i++) {
    const float X = obj[i][0], Y = obj[i][1];
    const float X_ = R[0]*X + R[1]*Y + tx;
    const float Y_ = R[3]*X + R[4]*Y + ty;
    const float Z_ = R[6]*X + R[7]*Y + tz;
    tag.corners[i][0] = (cal.fx * (X_ / Z_) + cal.cx) / (float)W;
    tag.corners[i][1] = (cal.fy * (Y_ / Z_) + cal.cy) / (float)H;
  }

  n6_apriltag_pose_t p;
  if (!n6_apriltag_pose(&tag, W, H, &cal, size, &p)) {
    Serial.print("  "); Serial.print(name); Serial.println("  SOLVER REFUSED");
    failures++;
    return;
  }

  const float et = fabsf(p.tx-tx) + fabsf(p.ty-ty) + fabsf(p.tz-tz);
  const float er = fabsf(p.rx-rx) + fabsf(p.ry-ry) + fabsf(p.rz-rz);
  const bool bad = (et > 0.002f) || (er > 0.5f);
  if (bad) failures++;

  Serial.print("  ");
  Serial.print(name);
  Serial.print("  t=("); Serial.print(p.tx,3); Serial.print(",");
  Serial.print(p.ty,3); Serial.print(","); Serial.print(p.tz,3);
  Serial.print(")  r=("); Serial.print(p.rx,1); Serial.print(",");
  Serial.print(p.ry,1); Serial.print(","); Serial.print(p.rz,1);
  Serial.print(")  err t="); Serial.print(et,4);
  Serial.print(" r="); Serial.print(er,2);
  Serial.println(bad ? "  FAIL" : "  ok");
}

static void run(void)
{
  failures = 0;
  Serial.println();
  Serial.println("=== AprilTag pose self-test (synthetic projections) ===");
  Serial.println("  80 mm tag, 640x480, f~520 unless stated");

  check("head-on 0.5 m     ", 0, 0, 0.50f,  0,   0,   0,  0.080f);
  check("head-on 2.0 m     ", 0, 0, 2.00f,  0,   0,   0,  0.080f);
  check("offset            ", 0.10f, -0.05f, 0.80f, 0, 0, 0, 0.080f);
  check("tilt 20 about x   ", 0, 0, 0.60f, 20,   0,   0,  0.080f);
  check("tilt 25 about y   ", 0, 0, 0.60f,  0,  25,   0,  0.080f);
  check("roll 40 about z   ", 0, 0, 0.60f,  0,   0,  40,  0.080f);
  check("compound          ", 0.05f, 0.03f, 0.75f, 15, -20, 30, 0.080f);
  check("160 mm tag near   ", 0, 0, 0.35f, 10,  10,  10,  0.160f);

  Serial.println();
  Serial.print("RESULT: ");
  if (failures == 0) Serial.println("all cases recover the pose they were built from");
  else { Serial.print(failures); Serial.println(" case(s) FAILED"); }
}

void setup() { Serial.begin(115200); delay(1500); }

void loop() { run(); delay(5000); }
