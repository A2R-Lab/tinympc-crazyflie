#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "tinympc_path_tunnel.h"

static bool near(float actual, float expected, float tolerance) {
  return fabsf(actual - expected) <= tolerance;
}

static void assertOrthonormal(TinyMpcTunnelFrame frame) {
  assert(frame.valid);
  assert(near(tinyMpcTunnelDot(frame.tangent, frame.tangent), 1.0f, 1.0e-5f));
  assert(near(tinyMpcTunnelDot(frame.normal_1, frame.normal_1), 1.0f, 1.0e-5f));
  assert(near(tinyMpcTunnelDot(frame.normal_2, frame.normal_2), 1.0f, 1.0e-5f));
  assert(near(tinyMpcTunnelDot(frame.tangent, frame.normal_1), 0.0f, 1.0e-5f));
  assert(near(tinyMpcTunnelDot(frame.tangent, frame.normal_2), 0.0f, 1.0e-5f));
  assert(near(tinyMpcTunnelDot(frame.normal_1, frame.normal_2), 0.0f, 1.0e-5f));
}

static void testLevelPathUsesLateralAndVerticalNormals(void) {
  const TinyMpcTunnelFrame frame = tinyMpcPathTunnelFrame(
      tinyMpcTunnelVector(1.0f, 0.0f, 0.0f), NULL);
  assertOrthonormal(frame);
  assert(near(frame.normal_1.y, 1.0f, 1.0e-6f));
  assert(near(frame.normal_2.z, 1.0f, 1.0e-6f));
}

static void testArbitrary3dTangentIsOrthonormal(void) {
  assertOrthonormal(tinyMpcPathTunnelFrame(
      tinyMpcTunnelVector(1.0f, -2.0f, 3.0f), NULL));
}

static void testNormalsAreTransportedWithoutSignFlip(void) {
  const TinyMpcTunnelFrame first = tinyMpcPathTunnelFrame(
      tinyMpcTunnelVector(1.0f, 0.0f, 0.0f), NULL);
  const TinyMpcTunnelFrame second = tinyMpcPathTunnelFrame(
      tinyMpcTunnelVector(0.98f, 0.15f, 0.12f), &first.normal_1);
  assertOrthonormal(second);
  assert(tinyMpcTunnelDot(first.normal_1, second.normal_1) > 0.9f);
}

static void testVerticalTangentHasAValidFallbackFrame(void) {
  assertOrthonormal(tinyMpcPathTunnelFrame(
      tinyMpcTunnelVector(0.0f, 0.0f, 1.0f), NULL));
}

int main(void) {
  testLevelPathUsesLateralAndVerticalNormals();
  testArbitrary3dTangentIsOrthonormal();
  testNormalsAreTransportedWithoutSignFlip();
  testVerticalTangentHasAValidFallbackFrame();
  puts("tinympc path tunnel tests passed");
  return 0;
}
