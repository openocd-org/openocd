#ifndef _JTAG_TCL_H_
#define _JTAG_TCL_H_

int jim_jtag_configure(Jim_Interp *interp, int argc,
			      Jim_Obj * const *argv);
int jim_jtag_tap_enabler(Jim_Interp *interp, int argc,
				Jim_Obj * const *argv);

#endif
