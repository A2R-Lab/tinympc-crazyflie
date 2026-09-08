#include <assert.h>
#include <stdio.h>
#include "../src/tinympc_vision_resume.h"

int main(void) {
  tinympcVisionResumeState s = {0};
  assert(!tinympcVisionResumeUpdate(&s,true,false,10) && s.clear_frames==1);
  assert(!tinympcVisionResumeUpdate(&s,true,false,10) && s.clear_frames==1);
  assert(!tinympcVisionResumeUpdate(&s,true,false,11) && s.clear_frames==2);
  assert(tinympcVisionResumeUpdate(&s,true,false,12) && s.clear_frames==3);
  assert(!tinympcVisionResumeUpdate(&s,true,true,13) && s.clear_frames==0);
  assert(!tinympcVisionResumeUpdate(&s,true,false,14) && s.clear_frames==1);
  assert(!tinympcVisionResumeUpdate(&s,true,false,16) && s.clear_frames==1);
  assert(!tinympcVisionResumeUpdate(&s,false,false,17) && s.clear_frames==0);
  s=(tinympcVisionResumeState){65534,1};
  assert(!tinympcVisionResumeUpdate(&s,true,false,65535));
  assert(tinympcVisionResumeUpdate(&s,true,false,1));
  assert(!tinympcVisionResumeUpdate(&s,true,false,0));
  puts("vision resume tests passed");
}
