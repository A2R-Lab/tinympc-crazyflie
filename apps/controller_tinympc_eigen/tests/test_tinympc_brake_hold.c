#include <assert.h>
#include "../src/tinympc_brake_hold.h"
int main(void) {
  bool locked = false; float x = 0, y = 0;
  assert(!tinympcBrakeHoldUpdate(&locked,&x,&y,1,2,1,0));
  assert(x==1 && y==2 && !locked);
  // Small forward speed alone is insufficient with significant sideways speed.
  assert(!tinympcBrakeHoldUpdate(&locked,&x,&y,2,3,.01f,.2f));
  assert(tinympcBrakeHoldUpdate(&locked,&x,&y,3,4,.05f,.05f));
  assert(locked && x==3 && y==4);
  assert(!tinympcBrakeHoldUpdate(&locked,&x,&y,4,5,-.4f,.3f));
  assert(x==3 && y==4); // Reversal/drift cannot drag the position reference.
  locked=false; // A new brake captures a new position.
  assert(!tinympcBrakeHoldUpdate(&locked,&x,&y,9,9,NAN,0));
  assert(x==3 && y==4 && !locked);
  assert(tinympcBrakeHoldUpdate(&locked,&x,&y,5,6,0,0));
  assert(x==5 && y==6);
}
