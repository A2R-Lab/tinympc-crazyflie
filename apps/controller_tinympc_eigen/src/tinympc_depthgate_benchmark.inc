// Disarmed DG-only timing harness. Production cold-start block copied below.
#if !TINYMPC_ESPNET_STRAIGHT_TEST
#error DepthGate benchmark requires espnet_straight_stop profile
#endif
static uint32_t dgBenchMin[4],dgBenchMax[4],dgBenchMean[4];
static uint8_t dgBenchDone;
static uint8_t dgBenchStage;
static uint32_t dgBenchStack;
static Eigen::Vector2f dgBenchRefs[NHORIZON];
static VectorNf dgBenchPredicted;
static const struct log_s dgBenchLogs[] __attribute__((section(".log.dgBench"),used)) = {
 {LOG_GROUP | LOG_START,const_cast<char*>("dgBench"),nullptr},
 {LOG_UINT32,const_cast<char*>("min1"),&dgBenchMin[0]},
 {LOG_UINT32,const_cast<char*>("max1"),&dgBenchMax[0]},
 {LOG_UINT32,const_cast<char*>("mean1"),&dgBenchMean[0]},
 {LOG_UINT32,const_cast<char*>("min2"),&dgBenchMin[1]},
 {LOG_UINT32,const_cast<char*>("max2"),&dgBenchMax[1]},
 {LOG_UINT32,const_cast<char*>("mean2"),&dgBenchMean[1]},
 {LOG_UINT32,const_cast<char*>("min3"),&dgBenchMin[2]},
 {LOG_UINT32,const_cast<char*>("max3"),&dgBenchMax[2]},
 {LOG_UINT32,const_cast<char*>("mean3"),&dgBenchMean[2]},
 {LOG_UINT32,const_cast<char*>("min4"),&dgBenchMin[3]},
 {LOG_UINT32,const_cast<char*>("max4"),&dgBenchMax[3]},
 {LOG_UINT32,const_cast<char*>("mean4"),&dgBenchMean[3]},
 {LOG_UINT8,const_cast<char*>("done"),&dgBenchDone},
 {LOG_UINT8,const_cast<char*>("stage"),&dgBenchStage},
 {LOG_UINT32,const_cast<char*>("stack"),&dgBenchStack},
 {LOG_GROUP | LOG_STOP,const_cast<char*>("stop_dgBench"),nullptr},
};
static void benchmarkMpcSolver() {
 mpc_solver_ready=false;
 dgBenchStage=1;
 vTaskDelay(M2T(2000)); // Let radio/system startup finish before timing work.
 selectDepthGateCache(true);
 stgs.max_iter=5;stgs.check_termination=0;
 const UBaseType_t priorPriority=uxTaskPriorityGet(nullptr);
 vTaskPrioritySet(nullptr,STABILIZER_TASK_PRI);
 // Cases1/2: one/two-plane moving references. Cases3/4: hold references.
 // Exactly five iterations and one solve in every case.
 for(int scenario=0;scenario<4;++scenario){
  uint64_t total=0;dgBenchMin[scenario]=UINT32_MAX;
  const int count=(scenario%2)+1;
  for(int repeat=0;repeat<20;++repeat){
   dgBenchStage=2+scenario;
   x0.setZero();x0(6)=.5f;
   const uint32_t started=usecTimestamp();
   Eigen::Vector2f *refs=dgBenchRefs;
   for(int k=0;k<NHORIZON;++k){
    data.en_hs[k]=0;data.count_xy_hs[k]=k?count:0;
    data.a_xy_hs[k][0]=count==1?Eigen::Vector2f(1,0):Eigen::Vector2f(.70710678f,.70710678f);
    data.a_xy_hs[k][1]=Eigen::Vector2f(.70710678f,-.70710678f);
    data.b_xy_hs[k][0]=data.b_xy_hs[k][1]=.1f;
    Eigen::Vector2f target(.5f*(k*DT),0);
    tiny_ProjectXY(target,data.a_xy_hs[k],data.b_xy_hs[k],count,
      Eigen::Vector2f(-100,-100),Eigen::Vector2f(100,100),Eigen::Vector2f(1,1),refs[k]);
   }
   for(int k=0;k<NHORIZON;++k){
    Xref[k].setZero();Xref[k].head<2>()=refs[k];
    if(k<NHORIZON-1){Xref[k].segment<2>(6)=(refs[k+1]-refs[k])/DT;Uref[k]=ug;}
   }
   if(scenario>=2) for(int k=0;k<NHORIZON;++k)Xref[k].setZero();
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

   VectorNf& predicted=dgBenchPredicted;predicted=x0;float violation=0;
   for(int k=1;k<NHORIZON;++k){
    predicted=(A*predicted+B*ZU_new[k-1]+f).eval();
    for(int j=0;j<count;++j)violation=fmaxf(violation,data.a_xy_hs[k][j].dot(predicted.head<2>())-data.b_xy_hs[k][j]);
   }
   dg_violation=violation;
   const uint32_t elapsed=usecTimestamp()-started;
   if(elapsed<dgBenchMin[scenario])dgBenchMin[scenario]=elapsed;
   if(elapsed>dgBenchMax[scenario])dgBenchMax[scenario]=elapsed;
   total+=elapsed;
   dgBenchStack=uxTaskGetStackHighWaterMark(nullptr);
   vTaskDelay(M2T(50)); // Always yield even when the measured solve overruns.
  }
  dgBenchMean[scenario]=(uint32_t)(total/20);
  DEBUG_PRINT("DG BENCH case=%d min=%lu max=%lu mean=%lu us\n",scenario+1,
   (unsigned long)dgBenchMin[scenario],(unsigned long)dgBenchMax[scenario],(unsigned long)dgBenchMean[scenario]);
 }
 vTaskPrioritySet(nullptr,priorPriority);
 mpc_solver_ready=false;dgBenchDone=1;dgBenchStage=6;
}
