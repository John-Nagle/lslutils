//
//  assert.lsl -- general assert include
//
//  Define DEBUG to enable this, and to enable DEBUGPRINT
//
#ifndef ASSERTLSL                                           // compile guard
#define ASSERTLSL

#ifdef DEBUG
#define DEBUGPRINT(s) // Nothing for now
#define DEBUGPRINT1(s) llOwnerSay((s))
#define assert(exp) { if (!(exp)) { llOwnerSay("Assertion failed at " + __SHORTFILE__ + " line " + (string) __LINE__); panic(); }}
#else // not debugging
#define DEBUGPRINT(s) {}
#define DEBUGPRINT1(s) {}
#define assert(exp) {}
#endif // DEBUG

#endif // ASSERTLSL
