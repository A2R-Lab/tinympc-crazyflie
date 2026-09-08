// Benchmark harness; never writes motor commands. Generated from controller solve block.
static float benchWorst[16];
static uint8_t benchDone;
static const struct log_s benchLogs[] __attribute__((section(".log.mpcBench"), used)) = {
 {LOG_GROUP | LOG_START, const_cast<char*>("mpcBench"), nullptr},
 {LOG_FLOAT, const_cast<char*>("us1"), &benchWorst[0]},
 {LOG_FLOAT, const_cast<char*>("us2"), &benchWorst[1]},
 {LOG_FLOAT, const_cast<char*>("us3"), &benchWorst[2]},
 {LOG_FLOAT, const_cast<char*>("us4"), &benchWorst[3]},
 {LOG_FLOAT, const_cast<char*>("us5"), &benchWorst[4]},
 {LOG_FLOAT, const_cast<char*>("us6"), &benchWorst[5]},
 {LOG_FLOAT, const_cast<char*>("us7"), &benchWorst[6]},
 {LOG_FLOAT, const_cast<char*>("us8"), &benchWorst[7]},
 {LOG_FLOAT, const_cast<char*>("us9"), &benchWorst[8]},
 {LOG_FLOAT, const_cast<char*>("us10"), &benchWorst[9]},
 {LOG_FLOAT, const_cast<char*>("us11"), &benchWorst[10]},
 {LOG_FLOAT, const_cast<char*>("us12"), &benchWorst[11]},
 {LOG_FLOAT, const_cast<char*>("us13"), &benchWorst[12]},
 {LOG_FLOAT, const_cast<char*>("us14"), &benchWorst[13]},
 {LOG_FLOAT, const_cast<char*>("us15"), &benchWorst[14]},
 {LOG_FLOAT, const_cast<char*>("us16"), &benchWorst[15]},
 {LOG_UINT8, const_cast<char*>("done"), &benchDone},
 {LOG_GROUP | LOG_STOP, const_cast<char*>("stop_mpcBench"), nullptr},
};
static void benchmarkMpcSolver() {
 const UBaseType_t priorPriority=uxTaskPriorityGet(nullptr);
 vTaskPrioritySet(nullptr, STABILIZER_TASK_PRI);
 TickType_t wake=xTaskGetTickCount();
 for (int iterations=8; iterations<=8; ++iterations) {
  stgs.max_iter=iterations;
  stgs.check_termination=0; // Always measure the full iteration budget.
  for (int mode=0; mode<4; ++mode) for (int repeat=0; repeat<100; ++repeat) {
   x0.setZero(); if(mode==2) x0(6)=3; if(mode==3){x0(6)=1.3f; x0(4)=.05f;}
   for(int k=0;k<NHORIZON;++k){Xref[k].setZero();if(mode==1||mode==3){Xref[k](0)=3*.02f*k+(mode==3?4.4f:0);Xref[k](6)=3;}}
   startTimestamp=usecTimestamp();
    // Cold-start both input and state ADMM variables together. Reusing only
    // input warm starts with reset state duals failed the high-rate regression.
    resetMpcInputWarmStart();
    /* Reinitialize state slacks in the newly recentered frame; never reuse
     * position/attitude duals from the previous local frame. Measured x0 stays
     * untouched, including when it is already outside the predicted bounds. */
    ZX_new[0] = x0; YX[0].setZero();
    for (int k = 1; k < NHORIZON; ++k) {
      ZX_new[k] = A * ZX_new[k-1] + B * ZU_new[k-1] + f;
      YX[k].setZero();
    }
    for (int k = 1; k < NHORIZON; ++k)
      ZX_new[k] = ZX_new[k].cwiseMin(ucx).cwiseMax(lcx);
    /* MPC solve */
    // Solve optimization problem using ADMM
    tiny_UpdateLinearCost(&work);
    tiny_SolveAdmm(&work);
    mpc_constraints[0] = mpc_constraints[1] = 0.0f;
    // Roll out the projected actuator commands, not just the state slack.
    VectorNf predicted = x0;
    for (int k = 1; k < NHORIZON; ++k) {
      predicted = (A * predicted + B * ZU_new[k-1] + f).eval();
      for (int i = 3; i <= 4; ++i)
        mpc_constraints[0] = fmaxf(mpc_constraints[0], fabsf(predicted(i)) - ucx(i));
      for (int i = 9; i <= 11; ++i)
        mpc_constraints[1] = fmaxf(mpc_constraints[1], fabsf(predicted(i)) - bodyRateBound);
    }
    mpc_constraints[2] = info.pri_res;
    mpc_constraints[3] = (float)(usecTimestamp() - startTimestamp);

   benchWorst[iterations-1]=fmaxf(benchWorst[iterations-1],(float)(usecTimestamp()-startTimestamp));
   vTaskDelayUntil(&wake, M2T(10)); // Sustain 100Hz with idle/watchdog/radio active.
  }
 }
 vTaskPrioritySet(nullptr, priorPriority);
 benchDone=1;
}
